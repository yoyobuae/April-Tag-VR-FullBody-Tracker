#pragma once

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <memory>
#include <math.h>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "v4l2wrapper.hpp"

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

    virtual ~FrameData() { }

    virtual void swap(FrameData& other) = 0;
    virtual void getImage(cv::Mat& out,
                          bool grayscale,
                          bool scale, int scale_num, int scale_denom,
                          bool useRoi, const cv::Rect& roi) = 0;
    virtual cv::Size size() const = 0;
    virtual int rows() const = 0;
    virtual int cols() const = 0;
};

class FrameDataOCV : public FrameData
{
    cv::Mat image;
public:
    virtual ~FrameDataOCV() { }

    void swap(cv::Mat& other);
    int flags() const;

    virtual void swap(FrameData& other) override;
    virtual void getImage(cv::Mat &out,
                          bool grayscale,
                          bool scale, int scale_num, int scale_denom,
                          bool useRoi, const cv::Rect& roi) override;
    virtual cv::Size size() const override;
    virtual int rows() const override;
    virtual int cols() const override;
};

class FrameDataV4L2 : public FrameData
{
    std::unique_ptr<V4L2Wrapper::Buffer> buf;
    int width = -1;
    int height = -1;

public:
    virtual ~FrameDataV4L2() { }

    void swap(std::unique_ptr<V4L2Wrapper::Buffer> &other);

    virtual void swap(FrameData& other) override;
    virtual void getImage(cv::Mat &out,
                          bool grayscale,
                          bool scale, int scale_num, int scale_denom,
                          bool useRoi, const cv::Rect& roi) override;
    virtual cv::Size size() const override;
    virtual int rows() const override;
    virtual int cols() const override;
};

class Camera {
public:
    virtual ~Camera() { }

    virtual void StartStop(std::string id, int apiPreference) = 0;
    virtual bool isRunning() = 0;
    virtual FrameData* MakeFrame() = 0;
    virtual void CopyFreshImageTo(FrameData& frame) = 0;
};

class Tracker;

class CameraOCV : public Camera {

    GUI *gui;
    Tracker* tracker;
    Parameters *parameters;
    std::string id;
    int apiPreference;
    cv::VideoCapture cap;
    bool cameraRunning;
    std::thread cameraThread;
    std::mutex cameraFrameMutex;
    std::condition_variable cameraFrameCondVar;
    std::unique_ptr<FrameDataOCV> cameraFrame;

    void CameraLoop();

public:
    CameraOCV(Tracker* tracker, GUI* gui, Parameters* parameters);
    virtual ~CameraOCV() { }

    virtual void StartStop(std::string id, int apiPreference) override;
    virtual bool isRunning() override;
    virtual FrameData* MakeFrame() override;
    virtual void CopyFreshImageTo(FrameData& frame) override;
};

class CameraV4L2 : public Camera {

    GUI *gui;
    Tracker* tracker;
    Parameters *parameters;
    std::string id;
    int apiPreference;
    V4L2Wrapper::Device dev;
    bool cameraRunning;
    std::thread cameraThread;
    std::mutex cameraFrameMutex;
    std::condition_variable cameraFrameCondVar;
    std::unique_ptr<FrameDataV4L2> cameraFrame;

    void CameraLoop();
    void setCameraParams();

public:
    CameraV4L2(Tracker* tracker, GUI* gui, Parameters* parameters);
    virtual ~CameraV4L2() { }

    virtual void StartStop(std::string id, int apiPreference) override;
    virtual bool isRunning() override;
    virtual FrameData* MakeFrame() override;
    virtual void CopyFreshImageTo(FrameData& frame) override;
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
    void SetGUI(GUI *gui);
    void StopCamera();

    bool mainThreadRunning = false;
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

    cv::Mat wtranslation = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    Quaternion<double> wrotation = Quaternion<double>(1, 0, 0, 0);

    double calibScale = 1;
    int drawImgSize = 1385;
    bool rotate = false;
    int rotateFlag = -1;

private:
    void CalibrateCamera();
    void CalibrateCameraCharuco();
    void CalibrateTracker();
    void MainLoop();


    GUI* gui;
    Parameters* parameters;
    Connection* connection;

    Camera *camera;

    std::thread mainThread;

    std::vector<cv::Ptr<cv::aruco::Board>> trackers;
    bool trackersCalibrated = false;

    cv::Mat statsImg = cv::Mat(1000, 2000, CV_8UC3, cv::Scalar(0, 0, 0));
    int statsCurX = 0;

    //Quaternion

    //Quaternion<double> q;

    MyApp* parentApp;
};
