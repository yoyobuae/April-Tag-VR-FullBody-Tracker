#pragma once

#include <chrono>
#include <condition_variable>
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

struct TrackerPose {
    int pose_valid;
    double a, b, c;
    double qw, qx, qy, qz;
};

static TrackerPose operator+(TrackerPose lhs, const TrackerPose &rhs)
{
    lhs.a += rhs.a;
    lhs.b += rhs.b;
    lhs.c += rhs.c;
    lhs.qw += rhs.qw;
    lhs.qx += rhs.qx;
    lhs.qy += rhs.qy;
    lhs.qz += rhs.qz;
    return lhs;
}

static TrackerPose operator-(TrackerPose lhs, const TrackerPose &rhs)
{
    lhs.a -= rhs.a;
    lhs.b -= rhs.b;
    lhs.c -= rhs.c;
    lhs.qw -= rhs.qw;
    lhs.qx -= rhs.qx;
    lhs.qy -= rhs.qy;
    lhs.qz -= rhs.qz;
    return lhs;
}

static TrackerPose operator/(TrackerPose lhs, double rhs)
{
    lhs.a /= rhs;
    lhs.b /= rhs;
    lhs.c /= rhs;
    lhs.qw /= rhs;
    lhs.qx /= rhs;
    lhs.qy /= rhs;
    lhs.qz /= rhs;
    return lhs;
}

static double sigmoid(double d)
{
    return 1.0 / (1.0 + std::exp(-d));
}

static TrackerPose compress(TrackerPose p)
{
    p.a = sigmoid(10.0*p.a)/2.5 -0.5/2.5;
    p.b = sigmoid(10.0*p.b)/2.5 -0.5/2.5;
    p.c = sigmoid(10.0*p.c)/2.5 -0.5/2.5;
    p.qw = sigmoid(10.0*p.qw)/2.5 -0.5/2.5;
    p.qx = sigmoid(10.0*p.qx)/2.5 -0.5/2.5;
    p.qy = sigmoid(10.0*p.qy)/2.5 -0.5/2.5;
    p.qz = sigmoid(10.0*p.qz)/2.5 -0.5/2.5;

    return p;
}

struct TrackerStatus {
    cv::Vec3d boardRvec, boardTvec, boardTvecDriver;
    bool boardFound, boardFoundDriver;
    std::vector<std::vector<double>> prevLocValues;
    std::vector<cv::Point2d> maskCenters;
    std::vector<int> maskSizes;
    std::chrono::milliseconds last_update_timestamp;
    int searchSize;
    int pose_valid;
    int idx;
    double a, b, c;
    double qw, qx, qy, qz;
    cv::Rect oldRoi;
    bool doImageMatching;
    cv::Point2f oldCenter;
    int pose_delta_index;
    TrackerPose pose_delta_history[10];
    TrackerPose pose_delta_average;
};


class FrameData
{
    cv::Mat image;
public:
    bool ready = false;
    clock_t captureTime;
    clock_t swapTime;
    clock_t copyFreshTime;
    clock_t toGrayTime;
    clock_t getPoseTime;
    clock_t processPoseTime;
    clock_t doMaskTime;
    clock_t detectTime;
    clock_t sendTrackerTime;

    void swap(cv::Mat &other);
    void swap(FrameData &other);
    void getImage(cv::Mat &out,
                  bool grayscale,
                  bool scale, int scale_num, int scale_denom,
                  bool useRoi, const cv::Rect &roi);
    cv::Size size() const;
    int flags() const;
    int rows() const;
    int cols() const;
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
    bool showTimingStats = true;
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
    std::condition_variable cameraFrameCondVar;
    FrameData cameraFrame;

    Parameters* parameters;
    Connection* connection;

    std::thread cameraThread;
    std::thread mainThread;

    std::vector<cv::Ptr<cv::aruco::Board>> trackers;
    bool trackersCalibrated = false;

    cv::Mat statsImg = cv::Mat(1000, 2000, CV_8UC3, cv::Scalar(0, 0, 0));
    int statsCurX = 0;

    bool rotate = false;
    int rotateFlag = -1;

    //Quaternion

    //Quaternion<double> q;

    MyApp* parentApp;
};
