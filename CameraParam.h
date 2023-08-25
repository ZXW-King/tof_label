#pragma once

#include <math.h>
#include <string.h>

namespace psl
{
struct CameraParam
{
    double _TSC[16];  // 4X4 camera to imu
    int _width;
    int _height;

    float b;
    float bf;

    // distortion_type:equidistant
    double _focal_length[2];     // fx,fy
    double _principal_point[2];  // cx,cy
    // Rectification matrix (stereo cameras only)
    // A rotation matrix aligning the camera coordinate system to the ideal
    // stereo image plane so that epipolar lines in both stereo images are
    // parallel.
    double _R[9];
    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    double _P[12];
    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    double _K[9];
    // The distortion parameters, size depending on the distortion model.
    // For us, the 4 parameters are: (k1, k2, t1, t2).
    double _D[4];

    CameraParam()
    {
    }
};
}  // namespace psl
