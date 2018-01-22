//
// Created by rokid on 8/27/17.
//
#include <sys/time.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <android/log.h>
#include <jni.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>

#include "TinySLAM.h"

citrus::TinySLAM* slam = NULL;
bool isSLAMStarted     = false;
bool ifCapture         = false;
int  CapFrameId        = 0;
bool ifGlass           = false;
bool ifSLAMExit        = false;
bool ifTrackGetBlocked = true;

unsigned char* imgData;
unsigned char* imgDataUp;
unsigned char* imgDataInv;
int img_w;
int img_h;

cpu_set_t* _mask;

void callbackExit()
{
    __android_log_print(ANDROID_LOG_DEBUG, "JNI", "callbackExit destroy check slam: %d", (int)slam->checkValid());
//    usleep(500000);
    ifSLAMExit = true;
    __android_log_print(ANDROID_LOG_DEBUG, "JNI", "destroy callbackExit...");
}



//void setCurrentThreadAffinityMask(cpu_set_t* mask)
//{
//    int err, syscallres;
//    pid_t pid = gettid();
//    syscallres = sched_setaffinity(pid, sizeof(mask), mask);
//    if (syscallres)
//    {
//        err = errno;
//        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "Error in the syscall setaffinity: mask=%d=0x%x err=%d=0x%x", mask, mask, err, err);
//    }
//}

extern "C"{

// Get the VSLAM instance
JNIEXPORT void JNICALL
Java_com_rokid_alab_app_tinyslam_TinySLAM_nativeTinySLAMInit( JNIEnv* env, jobject thiz, jstring calib, jstring config, jboolean ifglass)
{
    if (slam != NULL) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMInit slam->checkValid()");
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMInit check slam: %d", (int) slam->checkValid());
    }

    if (slam != NULL) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMInit slam->checkValid()");
        if (slam->checkValid())
            return;
    }

    __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMInit...");
    isSLAMStarted   = false;
    ifCapture       = false;
    CapFrameId      = 0;
    const char *nativeCalibString  = env->GetStringUTFChars(calib, 0);
    const char *nativeConfigString = env->GetStringUTFChars(config, 0);
    ifGlass = (bool)ifglass;
    slam = NULL;
    slam = new citrus::TinySLAM(nativeCalibString, nativeConfigString);
    __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMInit...new slam instance");

    img_w = 640;
    img_h = 480;

    imgData = (unsigned char*)malloc(img_w*img_h);
    imgDataInv = (unsigned char*)malloc(img_w*img_h);


    if (slam == NULL) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMInit...new slam instance is NULL");
    }

    env->ReleaseStringUTFChars(calib, nativeCalibString);
    env->ReleaseStringUTFChars(config, nativeConfigString);

}

JNIEXPORT jboolean JNICALL
Java_com_rokid_alab_app_tinyslam_TinySLAM_nativeTinySLAMStartTracking( JNIEnv* env, jobject thiz)
{
    if (slam == NULL) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMStartTracking...slam instance is NULL");
        return false;
    }

    /******* lock the cpu **********/
//    if (ifGlass) {
//        if (false) {
//            _mask = CPU_ALLOC(8);
//            if (_mask == NULL) {
//                __android_log_print(ANDROID_LOG_DEBUG, "JNI", "CPU_ALLOC");
//                return -1;
//            }
//            CPU_ZERO(_mask);
//            CPU_SET(4, _mask);
//            CPU_SET(5, _mask);
//            setCurrentThreadAffinityMask(_mask);
//        }
//    }
    /**** lock the cpu finished ****/

    isSLAMStarted = true;
    if (!slam->checkValid()) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMStartTracking - run slam->checkValid()");
        return false;
    }

    ifSLAMExit = false;
    return isSLAMStarted;
}


// Stop the VSLAM intance
JNIEXPORT void JNICALL
Java_com_rokid_alab_app_tinyslam_TinySLAM_nativeTinySLAMStop( JNIEnv* env, jobject thiz)
{
    if (slam == NULL) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMStop...slam instance is NULL");
        return ;
    }

    __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMStop......Start");

    if (!isSLAMStarted)
        return;

    if (!slam->checkValid()) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMStop - run slam->checkValid()");
        return;
    }

    isSLAMStarted = false;

    int counter = 0;
    while (!ifTrackGetBlocked) {
        usleep(30000);
        counter++;
        if (counter > 10) {
            break;
        }
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMStop - wait for Track to be blocked)");
    }

    slam->destroy(callbackExit);
    while (!ifSLAMExit) {
        usleep(30000);
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMStop - wait for SLAM to stop)");
    }

    delete slam;
    slam = NULL;
    __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMStop......End");
}

// Update the IMU data to the VSLAM instance
JNIEXPORT jboolean JNICALL
Java_com_rokid_alab_app_tinyslam_TinySLAM_nativeTinySLAMUpdateIMU( JNIEnv* env, jobject thiz,
        jfloatArray imuval, jdouble timestamp)
{
    if (slam == NULL) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMUpdateIMU...slam instance is NULL");
        return false;
    }

    if (!isSLAMStarted)
        return false;

    if (!slam->checkValid()) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMUpdateIMU - run slam->checkValid()");
        return false;
    }

    float* pimuval = env->GetFloatArrayElements(imuval, 0);
    citrus::TinySLAM::SLAMResult result = slam->update_imu(pimuval, timestamp);

    // __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMUpdateIMU %f %f %f %f %f %f",
    //                     pimuval[0], pimuval[1], pimuval[2], pimuval[3], pimuval[4], pimuval[5]);

    env->ReleaseFloatArrayElements(imuval, pimuval, 0);

    return true;
}

JNIEXPORT int JNICALL
Java_com_rokid_alab_app_tinyslam_TinySLAM_nativeTinySLAMTrackMono( JNIEnv* env, jobject thiz,
                                                             jbyteArray imageArray, jint width, jint height, jfloatArray pose, jfloatArray plane)
{
    if (slam == NULL) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMTrackMono...slam instance is NULL");
        ifTrackGetBlocked = true;
        return -2;
    }

    if (!isSLAMStarted) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMTrackMono...slam is not started");
        ifTrackGetBlocked = true;
        return -1;
    }

    if (!slam->checkValid()) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMTrackMono - run slam->checkValid()");
        ifTrackGetBlocked = true;
        return -1;
    }

    if (ifSLAMExit) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMTrackMono - slam instance is being destroyed. waiting.");
        ifTrackGetBlocked = true;
        return -1;
    }

    ifTrackGetBlocked = false;

    // get image array data
    int len = env->GetArrayLength(imageArray);
    env->GetByteArrayRegion(imageArray, 0, width*height, (jbyte*)imgData);

    citrus::TinySLAM::SLAMResult result;

//    __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMTrackMono - update_camera start");

    result = slam->update_camera(imgData, int(width), int(height), 0.0);

    // Get result and return
    float* R_ = (float *)result.cameraRotationMat33;
    float* t_ = (float *)result.cameraTranslationVec3;

    float* ppose = env->GetFloatArrayElements(pose, 0);
    for (int i = 0; i < 9; i++) {
        ppose[i] = R_[i];
    }
    for (int i = 0; i < 3; i++) {
        ppose[i + 9] = t_[i];
    }

//    __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMTrackMono - update_camera done");

//    __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMTrackMono \n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n",
//                        ppose[0], ppose[1], ppose[2],
//                        ppose[3], ppose[4], ppose[5],
//                        ppose[6], ppose[7], ppose[8],
//                        ppose[9], ppose[10], ppose[11]);


    env->ReleaseFloatArrayElements(pose, ppose, 0);

    float* pplane = env->GetFloatArrayElements(plane, 0);
    pplane[0] = result.mainPlaneVec4[0];
    pplane[1] = result.mainPlaneVec4[1];
    pplane[2] = result.mainPlaneVec4[2];
    pplane[3] = result.mainPlaneVec4[3];
    env->ReleaseFloatArrayElements(plane, pplane, 0);
    ;

    return result.status;
}

JNIEXPORT jboolean JNICALL
Java_com_rokid_alab_app_tinyslam_TinySLAM_nativeTinySLAMSetOnlyTracking( JNIEnv* env, jobject thiz, jboolean ifOnlyTracking)
{
    if (slam == NULL)
        return false;

    if (!isSLAMStarted)
        return false;

    if (!slam->checkValid()) {
        return false;
    }

    // TrackMono
    slam->set_only_tracking(bool(ifOnlyTracking));

    return true;
}

JNIEXPORT jint JNICALL
Java_com_rokid_alab_app_tinyslam_TinySLAM_nativeTinySLAMSaveImages( JNIEnv* env, jobject thiz,
                                                                 jbyteArray imageArray, jint width, jint height, jfloatArray pose, jfloatArray plane, jstring imgName)
{
    if (slam == NULL)
        return false;

    if (!isSLAMStarted)
        return false;

    if (!slam->checkValid())
        return false;

    int len = env->GetArrayLength(imageArray);
    env->GetByteArrayRegion(imageArray, 0, width*height, (jbyte*)imgData);

    cv::Mat image2Save = cv::Mat(480, 640, CV_8UC1, imgData);

    const char *nativeImgName = env->GetStringUTFChars(imgName, 0);

//    __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMSaveImages %s", nativeImgName);

    cv::imwrite(nativeImgName, image2Save);

    env->ReleaseStringUTFChars(imgName, nativeImgName);

    return true;
}

int
trackOneFrame( cv::Mat& image, int width, int height, float* pose, float* plane)
{
    if (slam == NULL) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "trackOneFrame...slam instance is NULL");
        ifTrackGetBlocked = true;
        return -2;
    }

    if (!isSLAMStarted) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "trackOneFrame...slam is not started");
        ifTrackGetBlocked = true;
        return -1;
    }

    if (!slam->checkValid()) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "trackOneFrame - run slam->checkValid()");
        ifTrackGetBlocked = true;
        return -1;
    }

    if (ifSLAMExit) {
        __android_log_print(ANDROID_LOG_DEBUG, "JNI", "trackOneFrame - slam instance is being destroyed. waiting.");
        ifTrackGetBlocked = true;
        return -1;
    }

    ifTrackGetBlocked = false;

    citrus::TinySLAM::SLAMResult result;

//    __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMTrackMono - update_camera start");

    result = slam->update_camera(image.data, int(width), int(height), 0.0);

    // Get result and return
    float* R_ = (float *)result.cameraRotationMat33;
    float* t_ = (float *)result.cameraTranslationVec3;

    for (int i = 0; i < 9; i++) {
        pose[i] = R_[i];
    }
    for (int i = 0; i < 3; i++) {
        pose[i + 9] = t_[i];
    }

//    __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMTrackMono - update_camera done");

//    __android_log_print(ANDROID_LOG_DEBUG, "JNI", "nativeTinySLAMTrackMono \n %f %f %f\n %f %f %f\n %f %f %f\n %f %f %f\n",
//                        ppose[0], ppose[1], ppose[2],
//                        ppose[3], ppose[4], ppose[5],
//                        ppose[6], ppose[7], ppose[8],
//                        ppose[9], ppose[10], ppose[11]);

    plane[0] = result.mainPlaneVec4[0];
    plane[1] = result.mainPlaneVec4[1];
    plane[2] = result.mainPlaneVec4[2];
    plane[3] = result.mainPlaneVec4[3];

    ;

    return result.status;
}

}

