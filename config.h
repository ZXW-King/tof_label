//
// Created by xin on 2023/8/24.
//


#include <string>

const std::string CONFIG_FILE_NAME = "param.yaml";
//const psl::Resolution RESOLUTION = psl::Resolution::RES_640X400;

struct StructureParam
{
    float cameraHeight = 0;
    float cameraAngle = 0;
    float tofAngle = 0;
    float tofHeight = 0;
    std::vector<float> leftcamera2lidar;
    std::vector<float> rightcamera2lidar;
    std::vector<float> leftcamera2tof;
    std::vector<float> rightcamera2tof;
    std::vector<float> poseTof2Machine;
};

struct ConfigParam
{
    StructureParam structure;
};

enum CameraType
{
    LEFT = 0, RIGHT
};

