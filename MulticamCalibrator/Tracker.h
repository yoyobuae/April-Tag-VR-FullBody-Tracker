#pragma once

#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <math.h>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "MyApp.h"

#include "Quaternion.h"
#include "Util.h"


struct TrackerStatus {
    cv::Vec3d boardRvec, boardTvec, boardTvecDriver;
    bool boardFound, boardFoundDriver;
    std::vector<std::vector<double>> prevLocValues;
    cv::Point2d maskCenter;
    std::chrono::milliseconds last_update_timestamp;
    int pose_valid;
    double a, b, c;
    double qw, qx, qy, qz;
};

class Connection;
class GUI;
class Parameters;

class Tracker
{
public:
    Tracker(Parameters*, Connection*, MyApp*);
    void StartCamera1(std::string id, int apiPreference);
    void StartCamera2(std::string id, int apiPreference);
    void StartCameraCalib();
    void StartTrackerCalib();
    void Start();

    bool mainThreadRunning = false;
    bool cameraRunning1 = false;
    bool cameraRunning2 = false;
    bool previewCamera1 = false;
    bool previewCamera2 = false;
    bool previewCameraCalibration1 = false;
    bool previewCameraCalibration2 = false;
    bool showTimeProfile = false;
    bool recalibrate = false;
    bool manualRecalibrate = false;
    bool multicamAutocalib = false;
    bool lockHeightCalib = false;
    bool disableOut = false;
    bool disableOpenVrApi = true;
    bool privacyMode = false;
    int messageDialogResponse = wxID_CANCEL;

    GUI* gui;

    cv::Mat wtranslation1 = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    cv::Mat wtranslation2 = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    Quaternion<double> wrotation1 = Quaternion<double>(1, 0, 0, 0);
    Quaternion<double> wrotation2 = Quaternion<double>(1, 0, 0, 0);

    double calibScale = 1;

private:
    void CameraLoop1();
    void CameraLoop2();
    void CopyFreshCameraImageTo1(cv::Mat& image);
    void CopyFreshCameraImageTo2(cv::Mat& image);
    void CalibrateCamera();
    void CalibrateCameraCharuco();
    void CalibrateTracker();
    void MainLoop1();
    void MainLoop2();

    int drawImgSize = 1385;

    cv::VideoCapture cap1;
    cv::VideoCapture cap2;

    // cameraImage and imageReady are protected by cameraImageMutex.
    // Use CopyFreshCameraImageTo in order to get the latest camera image.
    std::mutex cameraImageMutex1;
    std::mutex cameraImageMutex2;
    cv::Mat cameraImage1;
    cv::Mat cameraImage2;
    bool imageReady1 = false;
    bool imageReady2 = false;

    Parameters* parameters;
    Connection* connection;

    std::thread cameraThread1;
    std::thread cameraThread2;
    std::thread mainThread1;
    std::thread mainThread2;

    std::vector<cv::Ptr<cv::aruco::Board>> trackers;
    bool trackersCalibrated = false;

    //Quaternion

    //Quaternion<double> q;

    clock_t last_frame_time;

    MyApp* parentApp;

    std::mutex calibratorMutex;
    int frameCount1 = 0;
    int frameCount2 = 0;
    std::vector<cv::Point2d> calibratorProjected1;
    std::vector<cv::Point2d> calibratorProjected2;
    std::vector<cv::Point2d> calibratorReprojected1;
    std::vector<cv::Point2d> calibratorReprojected2;
    std::vector<cv::Point3d> calibratorPoints1;
    std::vector<cv::Point3d> calibratorPoints2;
    std::vector<long> calibratorTimes1;
    std::vector<long> calibratorTimes2;
    int pointsThreshold = 1000;

};
