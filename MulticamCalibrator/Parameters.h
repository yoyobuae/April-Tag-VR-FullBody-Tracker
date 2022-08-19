#pragma once
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include "Quaternion.h"

//#include "Language_English.h"
#include "Language.h"

class Parameters
{
public:
    std::string version = "0.5.5";
    std::string driverversion = "0.5.5";

    Parameters();
    void Load();
    void Save();
    std::string cameraAddr1 = "0";
    std::string cameraAddr2 = "0";
    int cameraApiPreference1 = 0;
    int cameraApiPreference2 = 0;
    cv::Mat camMat1;
    cv::Mat camMat2;
    cv::Mat distCoeffs1;
    cv::Mat distCoeffs2;
    cv::Mat stdDeviationsIntrinsics1;
    cv::Mat stdDeviationsIntrinsics2;
    std::vector<double> perViewErrors1;
    std::vector<double> perViewErrors2;
    std::vector<std::vector<cv::Point2f>> allCharucoCorners1;
    std::vector<std::vector<cv::Point2f>> allCharucoCorners2;
    std::vector<std::vector<int>> allCharucoIds1;
    std::vector<std::vector<int>> allCharucoIds2;
    std::vector<cv::Ptr<cv::aruco::Board>> trackers;
    int trackerNum = 1;
    double markerSize = 0.05;
    int numOfPrevValues = 5;
    double quadDecimate = 1;
    double searchWindow1 = 0.25;
    double searchWindow2 = 0.25;
    bool usePredictive = true;
    int calibrationTracker = 0;
    bool ignoreTracker0 = false;
    bool rotateCl1 = false;
    bool rotateCl2 = false;
    bool rotateCounterCl1 = false;
    bool rotateCounterCl2 = false;
    bool coloredMarkers = true;
    double calibOffsetX1 = 0;
    double calibOffsetX2 = 0;
    double calibOffsetY1 = 100;
    double calibOffsetY2 = 100;
    double calibOffsetZ1 = 100;
    double calibOffsetZ2 = 100;
    double calibOffsetA1 = 180;
    double calibOffsetA2 = 180;
    double calibOffsetB1 = 0;
    double calibOffsetB2 = 0;
    double calibOffsetC1 = 0;
    double calibOffsetC2 = 0;
    bool circularWindow = true;
    double smoothingFactor = 0.5;
    int camFps1 = 30;
    int camFps2 = 30;
    int camHeight1 = 0;
    int camHeight2 = 0;
    int camWidth1 = 0;
    int camWidth2 = 0;
    cv::Mat wtranslation1;
    cv::Mat wtranslation2;
    Quaternion<double> wrotation1;
    Quaternion<double> wrotation2;
    bool cameraSettings1 = false;
    bool cameraSettings2 = false;
    bool chessboardCalib = false;
    double camLatency = 0;
    bool circularMarkers = false;
    double trackerCalibDistance = 0.5;
    int cameraCalibSamples = 15;
    bool settingsParameters1 = false;
    bool settingsParameters2 = false;
    double cameraAutoexposure1 = 0;
    double cameraAutoexposure2 = 0;
    double cameraExposure1 = 0;
    double cameraExposure2 = 0;
    double cameraGain1 = 0;
    double cameraGain2 = 0;
    bool trackerCalibCenters = false;
    float depthSmoothing = 0;
    float additionalSmoothing = 0;
    int markerLibrary = 0;
    int markersPerTracker = 45;
    int languageSelection = 0;
    double calibScale = 1;
    std::string ipcAddr = "ApriltagPipeIn";


    cv::Ptr<cv::aruco::DetectorParameters> aruco_params = cv::aruco::DetectorParameters::create();

    Lang language;

};
