//
// Created by liming on 17-7-31. Modified by zhiyu on 17-8-20
//

#ifndef CITRUS_TINYSLAM_H
#define CITRUS_TINYSLAM_H

#include "citrus/api_definitions.hpp"

namespace citrus {

    class TinySLAM {

        private:
            void *slam;
            bool valid;

        public:

            /**
             * The status of the VSLAM algorithm instance. The system will only return normal and reliable tracking result when at the state of STATUS_TRACKING.
             * 1. NOTINITIALIZED means the algorithm has not been started.
             * 2. INITIALIZING means the algorithm has set the first image it captured as the first image for initialization and is checking if the current image has enough disparty to the first one. When the disparty is large enough the VSLAM algorithm will initialize the scense using the essential matrix computed by the two images and jump to the state of STATUS_TRACKING.
             * 3. TRACKING means the VSLAM algorithm is tracking the 6DOF (3DOF rotation and 3DOF translation) pose of the camera normally and has a high confidence on the tracking result.
             * 4. LOST means the VSLAM algorithm cannot track the 6DOF pose of the camera from the last frame and the map. It will then keep on matching the current frame with the the key frames stored before until one of the key frame can be successfully tracked by the current frame. Then the VSLAM algorithm will go back to the STATUS_TRACKING state.
             */
            enum SLAMStatus {
                NOTINITIALIZED,
                INITIALIZING,
                TRACKING,
                LOST
            };

            /**
             * The result returned by the VSLAM instance. The user should only get a valid result from the "update_camera()" API function!
             */
            struct SLAMResult {
                /**
                 * The 3*3 rotation matrix (in row major order) of the camera. The origin rotation (an identity matrix) is at the first frame captured by the VSLAM algorithm (the one captured at the state of STATUS_NOTINITIALIZED).
                 */
                float cameraRotationMat33[9];
                /**
                 * The 3-length vector translation of the camera. The origin translation (0, 0, 0) is at the first frame captured by the VSLAM algorithm (the one captured at the state of STATUS_NOTINITIALIZED).
                 */
                float cameraTranslationVec3[3];
                /**
                 * Four paramters main plane formula. ax + by + cz + d = 0;
                 * mainPlane = {a, b, c, d}
                 */
                float mainPlaneVec4[4];
                /**
                 * The VSLAM instance state returned by the update_camera function.
                 */
                SLAMStatus status;

                SLAMResult():status(NOTINITIALIZED) {};
            };

            CITRUS_API TinySLAM():slam(NULL), valid(false) { };

            /**
             * Construct a VSLAM algorithm instance using two configuration files.
             * This function should be called before you do every thing on slam and you must make sure that the two configuration files are valid.
             * arg1: the calibration result of the camera.
             * arg2: the configuration file for the VSLAM algorithm.
             */
            CITRUS_API TinySLAM(const char *calibration, const char *config);

            /**
             * Destroy the VSLAM algorithm instance and release the memory it used. You can do anything you want in the callback function, like shutting down the hardware.
             * arg: the pointer to the user defined callback function.
             */
            CITRUS_API void destroy(void (*callback) (void));

            /**
             * A function to check if the VSLAM algorithm instance is valid.
             */
            CITRUS_API bool checkValid();

            /**
             * Update the imu data to the VSLAM algorithm instance. Please be attention that this function will not return a valid camera tracking result.
             * arg1: a float type array of the IMU data. imudata = {acceleration_x, acceleration_y, acceleration_z, gyroscope_x, gyroscope_y, gyroscope_z}
             * arg2: the double type timeStamp which is the system time by second.
             */
            CITRUS_API SLAMResult update_imu(const float* imuData, double timeStamp);

            /**
             * Update the image data to the VSLAM algorithm instance.
             * arg1: a unsigned char type array of the grayscale image data (in row major order, 8-bits, size = width * height).
             * arg2: The width (column) of the image.
             * arg3: The height (row) of the image.
             * arg4: the double type timeStamp which is the system time by second.
             */
            CITRUS_API SLAMResult update_camera(const unsigned char* imgData, int width, int height, double timeStamp);

            /**
            * Set the VSLAM algorithm that if it will only tracking
            * arg1: if only tracking, then true, else false
            */
            CITRUS_API void set_only_tracking(bool ifOnlyTracking);

    }; // class TinySLAM

} // namespace citrus

#endif // CITRUS_TINYSLAM_H
