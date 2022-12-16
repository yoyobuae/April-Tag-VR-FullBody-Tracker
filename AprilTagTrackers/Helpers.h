#pragma once
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Quaternion.h"

void drawMarker(cv::Mat, std::vector<cv::Point2f>, cv::Scalar);
void transformMarkerSpace(std::vector<cv::Point3f>, cv::Vec3d, cv::Vec3d, cv::Vec3d, cv::Vec3d, std::vector<cv::Point3f>*);
void offsetFromBoardToCameraSpace(std::vector<cv::Point3f> points, cv::Vec3d boardRvec, cv::Vec3d boardTvec, std::vector<cv::Point3f>* out);
void getMedianMarker(std::vector<std::vector<cv::Point3f>>, std::vector<cv::Point3f>*);
Quaternion<double> rodr2quat(double, double, double);
cv::Mat getSpaceCalib(cv::Vec3d, cv::Vec3d, double, double, double);
cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f theta);
bool isRotationMatrix(cv::Mat& R);
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R);
Quaternion<double> mRot2Quat(const cv::Mat& m);
cv::Mat getSpaceCalibEuler(cv::Vec3d rvec, cv::Vec3d tvec, double xOffset, double yOffset, double zOffset);
cv::Vec3d quat2rodr(double qw, double qx, double qy, double qz);
void circleOnPostRotatedImg(cv::Mat &img, cv::Point center, int radius, const cv::Scalar &color, int rotate = -1, int thickness = 1, int lineType = cv::LINE_8, int shift = 0);
void rectangleOnPostRotatedImg(cv::Mat &img, cv::Point pt1, cv::Point pt2, const cv::Scalar &color, int rotate = -1, int thickness = 1, int lineType = cv::LINE_8, int shift = 0);
void drawAxisOnPostRotatedImg(cv::Mat &img, cv::Mat &camMat, cv::Mat &distCoeffs, cv::Vec3d &rvec, cv::Vec3d &tvec, float length, int rotate);
