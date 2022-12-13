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
    int searchSize;
};


struct FrameData
{
    bool ready = false;
    cv::Mat image;
    clock_t captureTime;
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
    void CopyFreshCameraImageTo(FrameData& frame);
    void CalibrateCamera();
    void CalibrateCameraCharuco();
    void CalibrateTracker();
    void MainLoop();

    int drawImgSize = 1385;

    cv::VideoCapture cap;

    // cameraFrame is protected by cameraImageMutex.
    // Use CopyFreshCameraFrameTo in order to get the latest camera frame.
    std::mutex cameraFrameMutex;
    FrameData cameraFrame;

    Parameters* parameters;
    Connection* connection;

    std::thread cameraThread;
    std::thread mainThread;

    std::vector<cv::Ptr<cv::aruco::Board>> trackers;
    bool trackersCalibrated = false;

    //Quaternion

    //Quaternion<double> q;

    MyApp* parentApp;
};
