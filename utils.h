//
// Created by xin on 2023/8/24.
//

#ifndef TOF_LABEL_UTILS_H
#define TOF_LABEL_UTILS_H

#endif //TOF_LABEL_UTILS_H


struct SyncDataFile
{
    std::string imageLeft;
    std::string imageRight;
    std::string imagePose;
    std::string lidar;
    std::string lidarPose;
    std::string tof;
    std::string tofPose;
    std::string imageCam0;

    void Print();

    SyncDataFile& SetPrefix(const std::string message)
    {
        this->imageLeft = message + this->imageLeft;
        this->imageRight = message + this->imageRight;
        this->imagePose = message + this->imagePose;
        this->lidarPose = message + this->lidarPose;
        this->lidar = message + this->lidar;
        this->tof = message + this->tof;
        this->tofPose = message + this->tofPose;
        this->imageCam0 = this->imageCam0;

        return *this;
    }
    void AddCam01Path()
    {
        std::string imagePath = this->imageLeft;
        int id = imagePath.rfind('/');
        this->imageLeft = imagePath.substr(0,id) + "/cam0" + imagePath.substr(id) ;
        this->imageRight =  imagePath.substr(0,id) + "/cam1" + imagePath.substr(id);
        imagePath = this->imageCam0;
        id = imagePath.rfind('/');
        this->imageCam0 = imagePath.substr(0,id) + "/cam0" + imagePath.substr(id) ;
    }
};