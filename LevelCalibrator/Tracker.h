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
    void StartCamera(std::string id, int apiPreference);
    void StartCameraCalib();
    void StartTrackerCalib();
    void Start();

    bool mainThreadRunning = false;
    bool cameraRunning = false;
    bool previewCamera = false;
    bool previewCameraCalibration = false;
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

    cv::Mat wtranslation = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    Quaternion<double> wrotation = Quaternion<double>(1, 0, 0, 0);

    double calibScale = 1;

private:
    void CameraLoop();
    void CopyFreshCameraImageTo(cv::Mat& image);
    void CalibrateCamera();
    void CalibrateCameraCharuco();
    void CalibrateTracker();
    void MainLoop();

    int drawImgSize = 1385;

    cv::VideoCapture cap;

    // cameraImage and imageReady are protected by cameraImageMutex.
    // Use CopyFreshCameraImageTo in order to get the latest camera image.
    std::mutex cameraImageMutex;
    cv::Mat cameraImage;
    bool imageReady = false;

    Parameters* parameters;
    Connection* connection;

    std::thread cameraThread;
    std::thread mainThread;

    std::vector<cv::Ptr<cv::aruco::Board>> trackers;
    bool trackersCalibrated = false;

    //Quaternion

    //Quaternion<double> q;

    clock_t last_frame_time;

    MyApp* parentApp;

    std::mutex calibratorMutex;
    int frameCount = 0;
    std::vector<cv::Point2d> calibratorProjected;
    std::vector<cv::Point2d> calibratorReprojected;
    std::vector<cv::Point3d> calibratorPoints;
    std::vector<long> calibratorTimes;
    int pointsThreshold = 9;
    int pointsThresholdIncrement = 1;
    cv::Point3d prevPoint = cv::Point3d(0, 0, 0);
    double speed = 0.0;
    bool isResting = false;
    std::vector<cv::Point3d> avgPoints;
};
