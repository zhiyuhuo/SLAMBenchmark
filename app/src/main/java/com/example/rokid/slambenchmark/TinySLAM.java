package com.example.rokid.slambenchmark;

/**
 * Created by Rokid on 8/29/17.
 */

public class TinySLAM {
    static {
        System.loadLibrary("TinySLAMHelper");
    }

    /**
     * All native methods that are implemented by the 'native-tiny-slam' native library,
     * which is packaged with this application.
     */
    public static native void    nativeTinySLAMInit(String calibPath, String configPath, boolean ifGlass);
    public static native void    nativeTinySLAMStop();
    public static native boolean nativeTinySLAMUpdateIMU(float[] imu, double timeStamp);
    public static native boolean nativeTinySLAMStartTracking();
    public static native boolean nativeTinySLAMSetOnlyTracking(boolean trackingOnly);
    public static native int     nativeTinySLAMTrackMono(String imageDir);
    public static native int     nativeTinySLAMTrackMono(byte[] imgBytes, int width, int height,
                                                         float[] pose, float[] plane);
    public static native int     nativeTinySLAMSaveImages(byte[] imgBytes, int width, int height,
                                                         float[] pose, float[] plane, String fname);

}
