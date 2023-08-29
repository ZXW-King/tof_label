//
// Created by xin on 2023/8/24.
//

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "TofDepthData.h"
#include <eigen3/Eigen/Dense>
#include "Resolution.h"
#include "CameraMoudleParam.h"
#include <yaml-cpp/yaml.h>
#include "config.h"
#include "file.h"
#include "utils.h"
#include "sstream"


#define EXIST(file) (access((file).c_str(), 0) == 0)
#define ERROR_PRINT(x) std::cout << "" << (x) << "" << std::endl
const float RADIAN_2_ANGLE = 180 / M_PI;
const psl::Resolution RESOLUTION = psl::Resolution::RES_640X400;
const int MAX_DEPTH = 700;
const int MAX_CHANNEL_VALUE = 255;
const int MAX_DOUBLE_CHANNEL_VALUE = 511;
const int WIDTH = 640;
const int HEIGHT = 400;

extern int access(const char *__name, int __type) __THROW __nonnull ((1));

void SunnyTof2Points(const psl::TofDepthData &tofDepthData, std::vector<Eigen::Vector3d> &points) {
    for (auto const &tof: tofDepthData.data) {
        points.push_back(Eigen::Vector3d(-tof.X, -tof.Y, tof.Z));
    }
}

void TofPointRotationAngle(std::vector<Eigen::Vector3d> &points, const float angle) {
// why this cost time more
//    Eigen::Matrix3d R;
//    R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());
//
//    for (auto &point : points)
//    {
//        point = R * point;
//    }

    for (auto &point: points) {
        float Y = point[1];
        float Z = point[2];
        point[1] = Y * cos(angle) - Z * sin(angle);
        point[2] = Z * cos(angle) + Y * sin(angle);
    }
}

void ExchangeXYZ(std::vector<Eigen::Vector3d> &points) {
    for (auto &point: points) {
        float tempX = point[0];
        float tempY = point[1];
        float tempZ = point[2];
        point[0] = tempZ;
        point[1] = tempX;
        point[2] = tempY;
    }
}


bool GetTofPoints(const psl::TofDepthData &tof, std::vector<Eigen::Vector3d> &points, const float angle) {
    SunnyTof2Points(tof, points); // --> x,y,z
    TofPointRotationAngle(points, angle);
    ExchangeXYZ(points);
}

void ReadArray(const YAML::Node &config, std::vector<float> &array) {
    try {
        array = config.as<std::vector<float>>();
    }
    catch (...) {
        for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
            array.push_back((*it).as<float>());
        }
    }
}

bool GetTof(std::string yamlFile, psl::TofDepthData &tof) {
    try {
        YAML::Node config;

        if (not access(yamlFile.c_str(), 0) == 0) {
            std::cout << "file not exist <" + yamlFile + ">" << std::endl;
        }
        config = YAML::LoadFile(yamlFile);

        tof.tofID = config["tofID"].as<int>();
        tof.time_stamp = config["time_stamp"].as<unsigned long>();
        tof.frameIndex = config["frameIndex"].as<unsigned long>();

        tof.height = config["height"].as<int>();
        tof.width = config["width"].as<int>();

        std::vector<float> sn(8);
        ReadArray(config["sn"], sn);

        for (int i = 0; i < 8; i++) {
            tof.sn[i] = sn[i];
        }

        static std::vector<float> data(tof.height * tof.width * 6);

        ReadArray(config["data"], data);

        int p = 0;
        for (int i = 0; i < tof.height * tof.width; i++) {
            tof.data[i].X = data.at(p++);
            tof.data[i].Y = data.at(p++);
            tof.data[i].Z = data.at(p++);
            tof.data[i].noise = data.at(p++);
            tof.data[i].grayValue = data.at(p++);
            tof.data[i].depthConfidence = data.at(p++);
        }

        return true;
    }
    catch (...) {
        std::cout << "GetTof data wrong!!!!" << std::endl;
        return false;
    }
}

bool GetCameraConfig(std::string file, psl::CameraMoudleParam &param) {
    cv::FileStorage fileStream = cv::FileStorage(file, cv::FileStorage::READ);

    if (not fileStream.isOpened()) {
        ERROR_PRINT("file not exist <" + file + ">");
        return false;
    }

    // TODO : the exception for lack option
    cv::Mat_<double> kl, dl, pl, rl;
    fileStream["Kl"] >> kl;
    fileStream["Dl"] >> dl;
    fileStream["Pl"] >> pl;
    fileStream["Rl"] >> rl;

    memcpy(param._left_camera[RESOLUTION]._K, kl.data, sizeof(param._left_camera[RESOLUTION]._K));
    memcpy(param._left_camera[RESOLUTION]._R, rl.data, sizeof(param._left_camera[RESOLUTION]._R));
    memcpy(param._left_camera[RESOLUTION]._P, pl.data, sizeof(param._left_camera[RESOLUTION]._P));
    memcpy(param._left_camera[RESOLUTION]._D, dl.data, sizeof(param._left_camera[RESOLUTION]._D));

    // TODO : the exception for lack option
    cv::Mat_<double> kr, dr, pr, rr;
    fileStream["Kr"] >> kr;
    fileStream["Dr"] >> dr;
    fileStream["Pr"] >> pr;
    fileStream["Rr"] >> rr;
    memcpy(param._right_camera[RESOLUTION]._K, kr.data, sizeof(param._right_camera[RESOLUTION]._K));
    memcpy(param._right_camera[RESOLUTION]._R, rr.data, sizeof(param._right_camera[RESOLUTION]._R));
    memcpy(param._right_camera[RESOLUTION]._P, pr.data, sizeof(param._right_camera[RESOLUTION]._P));
    memcpy(param._right_camera[RESOLUTION]._D, dr.data, sizeof(param._right_camera[RESOLUTION]._D));

    fileStream.release();

    return true;
}

bool GetParkerConfig(const std::string &configFile, ConfigParam &configParam) {
    YAML::Node configYaml = YAML::LoadFile(configFile);
    try {
        configParam.structure.leftcamera2lidar = configYaml["deeplearning"]["structure"]["leftcamera2lidar"].as<std::vector<float >>();
        configParam.structure.rightcamera2lidar = configYaml["deeplearning"]["structure"]["rightcamera2lidar"].as<std::vector<float >>();
        configParam.structure.cameraHeight = configYaml["deeplearning"]["structure"]["cameraHeight"].as<float>();
    }
    catch (const std::exception &e) {
        return ("config option <\nstructure/\n\tpose2Machine\n\tleftcamera2lidar\n\trightcamera2lidar"
                "\n\tlidarDirection\n\tlidarAngle\n\tcameraHeight\n\t> missed");
    }
    try {
        configParam.structure.cameraAngle = configYaml["deeplearning"]["structure"]["cameraAngle"].as<float>();
    }
    catch (const std::exception &e) {
        configParam.structure.cameraAngle = 10;
    }
    configParam.structure.cameraAngle = configParam.structure.cameraAngle / RADIAN_2_ANGLE;

    try {
        configParam.structure.tofAngle = configYaml["deeplearning"]["structure"]["tofAngle"].as<float>();
    }
    catch (const std::exception &e) {
        configParam.structure.tofAngle = 28;
    }
    configParam.structure.tofAngle = configParam.structure.tofAngle / RADIAN_2_ANGLE;

    try {
        configParam.structure.tofHeight = configYaml["deeplearning"]["structure"]["tofHeight"].as<float>();
    }
    catch (const std::exception &e) {
        configParam.structure.tofHeight = 0.1588;
    }
    try {
        configParam.structure.leftcamera2tof =
                configYaml["deeplearning"]["structure"]["leftcamera2tof"].as<std::vector<float >>();
        configParam.structure.rightcamera2lidar =
                configYaml["deeplearning"]["structure"]["rightcamera2tof"].as<std::vector<float >>();
        configParam.structure.poseTof2Machine =
                configYaml["deeplearning"]["structure"]["poseTof2Machine"].as<std::vector<float >>();
    }
    catch (const std::exception &e) {
        configParam.structure.leftcamera2tof = {0, 0, 0};
        configParam.structure.rightcamera2tof = {0, 0, 0};
        configParam.structure.poseTof2Machine = {0, 0, 0};
    }
}

void TofPointsInImage(const std::vector<Eigen::Vector3d> &points, std::vector<cv::Point> &imagePoints,
                      std::vector<Eigen::Vector3d> &pointsSelect, std::vector<float> leftcamera2tof,
                      Eigen::Matrix<double, 3, 4> P) {
    imagePoints.clear();
    pointsSelect.clear();
    long i = 0;
    for (const auto &point: points) {
        ++i;
        if (point[0] < 0.01)
            continue;
        cv::Point pointTemp;
        float x = -(point[1] - leftcamera2tof[1]);
        float y = -(point[2] - leftcamera2tof[2]);
//        if (y >0.15) continue;
        float z = point[0] - leftcamera2tof[0];
        float u = P(0, 0) * x + P(0, 2) * z;
        float v = P(1, 1) * y + P(1, 2) * z;
        float scale = z * P(2, 2);
        pointTemp.x = int(u / scale);
        pointTemp.y = int(v / scale);
        imagePoints.push_back(pointTemp);
        pointsSelect.push_back(point);
    }
}

void TofPointsInImage2DepthValues(const std::vector<Eigen::Vector3d> &points, std::vector<cv::Point> &imagePoints,
                                  std::vector<Eigen::Vector3d> &pointsSelect, std::vector<float> leftcamera2tof,
                                  Eigen::Matrix<double, 3, 4> P ,std::vector<int> &depthValues)
{
    depthValues.clear();
    TofPointsInImage(points,imagePoints,pointsSelect,leftcamera2tof,P);
//    points.clear();
//    for (int i = 0; i < pointsSelect.size(); ++i)
//    {
//        cv::Point imagePoint = imagePoints[i];
//        points.push_back(pointsSelect[i]);
//        pointsSelectedDraw.push_back(pointsSelect[i]);
//        imagePointsDraw.push_back(imagePoints[i]);
//
//    }
    for(auto const point : pointsSelect)
    {
        depthValues.push_back(int(point[0] * 100));
    }
}

void SetImageValueByPoint(cv::Mat &image, const cv::Point &point, const cv::Scalar &scalar)
{
    int a = image.channels();
    if (image.channels() ==1)
    {

    }
    else if (image.channels() == 3)
    {
        if (point.y >= 0 && point.y < image.rows && point.x < image.cols && point.x >= 0)
        {
            image.at<cv::Vec3b>(point.y, point.x)[0] = scalar(0);
            image.at<cv::Vec3b>(point.y, point.x)[1] = scalar(1);
            image.at<cv::Vec3b>(point.y, point.x)[2] = scalar(2);
        }
    }
}

void DepthValue2Scalar(const int depthValue, cv::Scalar &scalar)
{
    int value = MIN(depthValue, MAX_DEPTH);

    if (value > MAX_DOUBLE_CHANNEL_VALUE)
    {
        scalar = cv::Scalar(0, 0, value % MAX_CHANNEL_VALUE);
    }
    else if (value > MAX_CHANNEL_VALUE)
    {
        scalar = cv::Scalar(0, value % MAX_CHANNEL_VALUE, 0);
    }
    else
    {
        scalar = cv::Scalar(value , 0, 0);
    }
}

void DrawPoints(cv::Mat &image, const std::vector<cv::Point> &imagePoints
        , const std::vector<int> &depthValues, bool selected = false)
{
    int number = 0;
    for (int i = 0; i < imagePoints.size(); ++i)
    {
        cv::Scalar scalar;
        DepthValue2Scalar(depthValues[i], scalar);
        if (selected)
        {
            cv::rectangle(image, cv::Point(imagePoints[i].x - 1, imagePoints[i].y - 1)
                    , cv::Point(imagePoints[i].x + 1, imagePoints[i].y + 1), scalar, -1);
        }
        else
        {
//            cv::circle(image, imagePoints[i], 1, scalar, -1);
            SetImageValueByPoint(image, imagePoints[i], scalar);
        }
        if (imagePoints[i].y >=0 && imagePoints[i].y < image.rows && imagePoints[i].x >=0 && imagePoints[i].x < image.cols)
        {
            number++;
        }
    }
}
std::vector<std::string> split(const std::string &s, char delimiter) {
    std::vector<std::string> parts;
    std::istringstream ss(s);
    std::string part;
    while (std::getline(ss, part, delimiter)) {
        parts.push_back(part);
    }
    return parts;
}

std::string getLastTwoPathParts(const std::string &path) {
    std::vector<std::string> pathParts = split(path, '/');
    std::string result;

    if (pathParts.size() >= 2) {
        result = pathParts[pathParts.size() - 2] + "/" + pathParts[pathParts.size() - 1] + "/";
    }

    return result;
}
bool showTof(cv::Mat imageDisplay,std::vector<cv::Point> imagePoints, std::vector<int> depthValues,std::string fileName,std::string inputDir) {

    DrawPoints(imageDisplay, imagePoints, depthValues);
    {
        std::string lastTwoParts = getLastTwoPathParts(inputDir);
        std::string tofLabelPath = "/media/xin/data1/data/parker_data/tof_label/" + lastTwoParts;
        std::string file = tofLabelPath + "result_image_with_tof/" + fileName;
        file_op::File::MkdirFromFile(file);
        cv::imwrite(file, imageDisplay);
//        file = "result_image_with_tof_copy/" + imagePath;
//        file_op::File::MkdirFromFile(file);
//        cv::imwrite(file, imageDisplay);

        cv::Mat imageOut(imageDisplay.rows, imageDisplay.cols, CV_8UC3, cv::Scalar(0, 0, 0));
        DrawPoints(imageOut, imagePoints, depthValues);

        file = tofLabelPath + "result_tof/" + fileName;
        file_op::File::MkdirFromFile(file);
        cv::imwrite(file, imageOut);
    }

//    imagePoints.clear();
//    depthValues.clear();
//    instanceManager.GetSelectPoints(imagePoints, depthValues);
//    DrawPoints(imageDisplay, imagePoints, depthValues, true);
}


bool ReadFile(std::string srcFile, std::vector<std::string> &image_files)
{
    if (not access(srcFile.c_str(), 0) == 0)
    {
        ERROR_PRINT("no such File (" + srcFile + ")");
        return false;
    }

    std::ifstream fin(srcFile.c_str());

    if (!fin.is_open())
    {
        ERROR_PRINT("read file error (" + srcFile + ")");
        return false;
    }

    std::string s;
    while (getline(fin, s))
    {
        image_files.push_back(s);
    }

    fin.close();

    return true;
}

bool ReadSyncFile(std::string srcFile, std::vector<SyncDataFile> &files, const bool &flag)
{
    if (not access(srcFile.c_str(), 0) == 0)
    {
        ERROR_PRINT("no such File (" + srcFile + ")");
        return false;
    }

    std::ifstream fin(srcFile.c_str());

    if (!fin.is_open())
    {
        ERROR_PRINT("read file error (" + srcFile + ")");
        return false;
    }

    std::string s;
    SyncDataFile syncFile;

    do
    {
        fin >> syncFile.imageLeft >> syncFile.imagePose >> syncFile.lidar
            >> syncFile.lidarPose;
        if (flag)
        {
            fin >> syncFile.tof;
            fin >> syncFile.tofPose;
        }
        files.push_back(syncFile);
    } while (fin.get() != EOF);

    if (files.size() > 1) files.pop_back();

    fin.close();

    return true;
}

bool GetData(const std::string inputDir, std::vector<SyncDataFile>& dataset, const bool &flag)
{
    std::string imagesTxt = inputDir + "/image.txt";
    std::string lidarTxt = inputDir + "/lidar.txt";
    std::string syncTxt = inputDir + "/sync.txt";
    const bool synced = not EXIST(imagesTxt);
    bool binocular = false;
    std::vector<std::string> imageNameList, lidarNameList;
    std::vector<SyncDataFile> fileList;

    if (synced)
    {
        if (not ReadSyncFile(syncTxt, fileList, flag)) exit(0);
    }
    else
    {
        if (not ReadFile(imagesTxt, imageNameList)) exit(0);
        if (not ReadFile(lidarTxt, lidarNameList)) exit(0);
    }

    const size_t size =
            imageNameList.size() > 0 ? imageNameList.size() : fileList.size();

    for (size_t i = 0; i < size; ++i)
    {
        SyncDataFile item;

        if (synced)
        {
            std::string imageLeftName = fileList.at(i).imageLeft;
            fileList.at(i).imageCam0 = imageLeftName.replace(imageLeftName.find_last_of('.'), imageLeftName.length(), ".png");
            item = fileList.at(i).SetPrefix(inputDir + "/");
            if (not EXIST(item.imageLeft))
            {
                binocular = true;
                item.AddCam01Path();
            }
        }
        else
        {
            item.imageCam0 = imageNameList.at(i);
            item.imageLeft = inputDir + "/" + imageNameList.at(i);
            item.lidar = inputDir + "/" + lidarNameList[i];
            item.lidarPose = item.lidar;
            item.lidarPose = item.lidarPose.replace(item.lidar.find("lidar"), 5, "slam");
            item.imagePose = item.lidarPose;
        }

        dataset.push_back(item);
    }

    return binocular;
}

std::string GetFileNameFromPath(const std::string& path) {
    size_t lastSlashPos = path.find_last_of("/\\"); // 查找最后一个路径分隔符的位置

    if (lastSlashPos != std::string::npos) {
        std::string FileName = path.substr(lastSlashPos + 1);
        // 将 ".jpg" 替换为 ".png"
        size_t extensionPos = FileName.find(".jpg");
        return FileName.replace(extensionPos, 4, ".png");; // 提取文件名部分,并转为png
    } else {
        return path; // 如果没有路径分隔符，则返回整个路径作为文件名
    }
}

bool ReadPara(const psl::CameraParam &cameraParam
        , cv::Mat& remapX, cv::Mat& remapY)
{
    cv::Mat K, P, R, D;
    K = cv::Mat(3, 3, CV_64FC1, (unsigned char *) cameraParam._K);
    D = cv::Mat(4, 1, CV_64FC1, (unsigned char *) cameraParam._D);
//    R = cv::Mat::eye(3, 3, CV_64FC1);
    R = cv::Mat(3, 3, CV_64FC1, (unsigned char *) cameraParam._R);
    P = cv::Mat(3, 4, CV_64FC1, (unsigned char *) cameraParam._P);

    remapX.create(cv::Size(WIDTH, HEIGHT), CV_32FC1);
    remapY.create(cv::Size(WIDTH, HEIGHT), CV_32FC1);
    cv::fisheye::initUndistortRectifyMap(K, D, R, P.rowRange(0, 3).colRange(0, 3)
            , cv::Size(WIDTH, HEIGHT), CV_32F
            , remapX, remapY);
}

bool Remap(const CameraType type, cv::Mat &image,cv::Mat remapX,cv::Mat remapY)
{

    cv::setNumThreads(0);
    cv::Mat remap = image;
    cv::Mat rgb;

    if (LEFT == type)
    {
        cv::remap(image, remap, remapX, remapY
                , cv::INTER_LINEAR);
    }
    else
    {
        cv::remap(image, remap, remapX, remapY
                , cv::INTER_LINEAR);
    }
    if (1 == remap.channels())
    {
        std::vector<cv::Mat> grayGroup(3, remap);
        cv::merge(grayGroup, rgb);
    }
    else if (3 == remap.channels())
    {
        rgb = remap;
    }
    else
    {
        return false;
    }

    image = rgb;

    return true;
}



int main() {

    // 读取图像
    std::vector<SyncDataFile> dataset;
    std::string inputDir = "/media/xin/data1/data/parker_data/2022_08_22/louti/data_2023_0822_2"; //数据集路径
    psl::CameraMoudleParam param;
    std::string cameraConfigFile = inputDir + "/config.yaml"; //相机配置文件路径
    GetCameraConfig(cameraConfigFile, param);  // 获取相机配置数据

    const std::string parkerDir = "/home/xin/zhang/c_project/tof/tof_label/config/param_parker.yaml"; //parker配置文件路径
    ConfigParam configParam;
    GetParkerConfig(parkerDir, configParam);  //获取parker配置数据
    bool binocular = GetData(inputDir, dataset, true);

    const size_t size = dataset.size();

    for (size_t i = 0; i < size; ++i) {
        SyncDataFile item = dataset.at(i);

        std::string imageLeftPath(item.imageLeft);
        std::cout << "item.imageLeft: " << item.imageLeft << ", item.imageCam0: " << item.imageCam0 << std::endl;
        auto imageLeft = cv::imread(imageLeftPath, cv::IMREAD_GRAYSCALE);
        std::cout << "left image read: " << imageLeftPath << std::endl;
        std::string imageRightPath;
        cv::Mat imageRight;

        if (imageLeft.empty()) {
            ERROR_PRINT("empty data in file <" + imageLeftPath + ">");
            continue;
        }

        if (binocular) {
            imageRightPath = item.imageRight;
            imageRight = cv::imread(imageRightPath, cv::IMREAD_GRAYSCALE);
//            imageRight = cv::imread(imageRightPath, cv::IMREAD_COLOR);
            std::cout << "right image read: " << imageRightPath << std::endl;

            if (imageRight.empty()) {
                ERROR_PRINT("empty data in file <" + imageRightPath + ">");
                continue;
            }
        }

        // 从配置文件中读取tof数据
        psl::TofDepthData tof;
        std::vector<Eigen::Vector3d> points;
        std::string tofpath = item.tof; //tof配置文件路径
        GetTof(tofpath, tof);
        GetTofPoints(tof, points, configParam.structure.tofAngle); // tof点坐标转换

        // remap操作
        cv::Mat remapX = cv::Mat();
        cv::Mat remapY = cv::Mat();
        ReadPara(param._left_camera.at(RESOLUTION), remapX, remapY);
        ReadPara(param._right_camera.at(RESOLUTION), remapX, remapY);
        const CameraType letftype = LEFT;
        const CameraType righttype = RIGHT;
        Remap(letftype, imageLeft,remapX,remapY);
        Remap(righttype, imageRight,remapX,remapY);

        std::vector<Eigen::Vector3d> pointsSelected;
        std::vector<cv::Point> imagePoints;
        std::vector<int> depthValues;
        const double *p = param._left_camera.at(RESOLUTION)._P;
        Eigen::Matrix<double, 3, 4> P = Eigen::Matrix<double, 3, 4>::Zero();
        P << p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11];
        TofPointsInImage2DepthValues(points, imagePoints, pointsSelected, configParam.structure.leftcamera2tof, P,
                                     depthValues);
//        std::vector<Eigen::Vector3d> pointsSelectedDraw;
//        std::vector<cv::Point> imagePointsDraw;



        std::string fileName = GetFileNameFromPath(item.imageLeft);
        showTof(imageLeft,imagePoints, depthValues,fileName,inputDir);
    }
    return 0;
}



