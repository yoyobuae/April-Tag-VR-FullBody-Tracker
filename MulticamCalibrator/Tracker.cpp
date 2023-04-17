#define _USE_MATH_DEFINES

#include <mutex>
#include <random>
#include <sstream>
#include <vector>

#pragma warning(push)
#pragma warning(disable:4996)
#include <wx/wx.h>
#pragma warning(pop)

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <thread>

#include "AprilTagWrapper.h"
#include "Connection.h"
#include "GUI.h"
#include "Helpers.h"
#include "MessageDialog.h"
#include "Parameters.h"
#include "Tracker.h"
#include "EulerAngles.h"

namespace {

// Create a grid in front of the camera for visualization purposes.
std::vector<std::vector<cv::Point3f>> createXyGridLines(
    const int gridSizeX, // Number of units from leftmost to rightmost line.
    const int gridSizeY, // Number of units from top to bottom line.
    const int gridSubdivision, // Number of segments per line.
    const float z) // Z-coord of grid.
{
    std::vector<std::vector<cv::Point3f>> gridLines(gridSizeX + gridSizeY + 2);
    for (int i = 0; i <= gridSizeX; ++i)
    {
        auto& verticalLine = gridLines[i];
        verticalLine.reserve(gridSizeY * gridSubdivision + 1);
        const float x = float(i) - float(gridSizeX) * 0.5f;
        for (int j = 0; j <= gridSizeY * gridSubdivision; ++j)
        {
            const float y = float(j) / float(gridSubdivision) - float(gridSizeY) * 0.5f;
            verticalLine.push_back(cv::Point3f(x, y, z));
        }
    }
    for (int i = 0; i <= gridSizeY; ++i)
    {
        auto& horizontalLine = gridLines[gridSizeX + 1 + i];
        horizontalLine.reserve(gridSizeX * gridSubdivision + 1);
        const float y = float(i) - float(gridSizeY) * 0.5f;
        for (int j = 0; j <= gridSizeX * gridSubdivision; ++j)
        {
            const float x = float(j) / float(gridSubdivision) - float(gridSizeX) * 0.5f;
            horizontalLine.push_back(cv::Point3f(x, y, z));
        }
    }
    return gridLines;
}

void previewCalibration(
    cv::Mat& drawImg,
    const cv::Mat1d& cameraMatrix,
    const cv::Mat1d& distCoeffs,
    const cv::Mat1d& stdDeviationsIntrinsics,
    const std::vector<double>& perViewErrors,
    const std::vector<std::vector<cv::Point2f>>& allCharucoCorners,
    const std::vector<std::vector<int>>& allCharucoIds)
{
    if (!cameraMatrix.empty())
    {
        const float gridZ = 10.0f;
        const float width = drawImg.cols;
        const float height = drawImg.rows;
        const float fx = cameraMatrix(0, 0);
        const float fy = cameraMatrix(1, 1);
        const int gridSizeX = std::round(gridZ * width / fx);
        const int gridSizeY = std::round(gridZ * height / fy);
        const std::vector<std::vector<cv::Point3f>> gridLinesInCamera = createXyGridLines(gridSizeX, gridSizeY, 10, gridZ);
        std::vector<cv::Point2f> gridLineInImage; // Will be populated by cv::projectPoints.

        // The generator is static to avoid starting over with the same seed every time.
        static std::default_random_engine generator;
        std::normal_distribution<double> unitGaussianDistribution(0.0, 1.0);

        cv::Mat1d sampleCameraMatrix = cameraMatrix.clone();
        cv::Mat1d sampleDistCoeffs = distCoeffs.clone();
        if (!stdDeviationsIntrinsics.empty())
        {
            assert(sampleDistCoeffs.total() + 4 <= stdDeviationsIntrinsics.total());
            sampleCameraMatrix(0, 0) += unitGaussianDistribution(generator) * stdDeviationsIntrinsics(0);
            sampleCameraMatrix(1, 1) += unitGaussianDistribution(generator) * stdDeviationsIntrinsics(1);
            sampleCameraMatrix(0, 2) += unitGaussianDistribution(generator) * stdDeviationsIntrinsics(2);
            sampleCameraMatrix(1, 2) += unitGaussianDistribution(generator) * stdDeviationsIntrinsics(3);
            for (int i = 0; i < sampleDistCoeffs.total(); ++i)
            {
                sampleDistCoeffs(i) += unitGaussianDistribution(generator) * stdDeviationsIntrinsics(i + 4);
            }
        }

        for (const auto& gridLineInCamera : gridLinesInCamera)
        {
            cv::projectPoints(gridLineInCamera, cv::Vec3f::zeros(), cv::Vec3f::zeros(), sampleCameraMatrix, sampleDistCoeffs, gridLineInImage);
            for (size_t j = 1; j < gridLineInImage.size(); ++j)
            {
                const auto p1 = gridLineInImage[j - 1];
                const auto p2 = gridLineInImage[j];
                cv::line(drawImg, p1, p2, cv::Scalar(127, 127, 127));
            }
        }
    }

    if (allCharucoCorners.size() > 0)
    {
        // Draw all corners that we have so far
        cv::Mat colorsFromErrors;
        if (!perViewErrors.empty())
        {
            cv::Mat(perViewErrors).convertTo(colorsFromErrors, CV_8UC1, 255.0, 0.0);
            cv::applyColorMap(colorsFromErrors, colorsFromErrors, cv::COLORMAP_VIRIDIS);
        }
        for (int i = 0; i < allCharucoCorners.size(); ++i)
        {
            const auto& charucoCorners = allCharucoCorners[i];
            cv::Scalar color(200, 100, 0);
            if (colorsFromErrors.total() > i)
            {
                color = colorsFromErrors.at<cv::Vec3b>(i);
            }
            for (const auto& point : charucoCorners)
            {
                cv::circle(drawImg, point, 2, color, cv::FILLED);
            }
        }
    }
}

void previewCalibration(cv::Mat& drawImg, Parameters* parameters)
{
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat1d stdDeviationsIntrinsics;
    parameters->camMat1.copyTo(cameraMatrix);
    parameters->distCoeffs1.copyTo(distCoeffs);
    parameters->stdDeviationsIntrinsics1.copyTo(stdDeviationsIntrinsics);
    std::vector<double> perViewErrors = parameters->perViewErrors1;
    std::vector<std::vector<cv::Point2f>> allCharucoCorners = parameters->allCharucoCorners1;
    std::vector<std::vector<int>> allCharucoIds = parameters->allCharucoIds1;

    previewCalibration(
        drawImg,
        cameraMatrix,
        distCoeffs,
        stdDeviationsIntrinsics,
        perViewErrors,
        allCharucoCorners,
        allCharucoIds);
}

} // namespace

Tracker::Tracker(Parameters* params, Connection* conn, MyApp* app)
{
    parameters = params;
    connection = conn;
    parentApp = app;
    if (!parameters->trackers.empty())
    {
        trackers = parameters->trackers;
        trackersCalibrated = true;
    }
    if (!parameters->wtranslation1.empty())
    {
        wtranslation1 = parameters->wtranslation1;
        wrotation1 = parameters->wrotation1;
    }
#if 0
    if (!parameters->wtranslation2.empty())
    {
        wtranslation2 = parameters->wtranslation2;
        wrotation2 = parameters->wrotation2;
    }
#endif
    calibScale = parameters->calibScale;
}

void Tracker::StartCamera1(std::string id, int apiPreference)
{
    if (cameraRunning1)
    {
        cameraRunning1 = false;
        mainThreadRunning = false;
        //cameraThread.join();
        sleep_millis(1000);
        return;
    }
    if (id.length() <= 2)		//if camera address is a single character, try to open webcam
    {
        int i = std::stoi(id);	//convert to int
#if defined(__LINUX__)
        // On Linux cv::VideoCapture does not work when GStreamer backend is used and
        // camera is set to MJPG pixel format. As a work around we manually setup the
        // GStreamer pipeline with suitable decoding before feeding the stream into
        // application.
        if ((apiPreference == cv::CAP_ANY) || (apiPreference == cv::CAP_GSTREAMER))
        {
            std::stringstream ss;
            ss << "v4l2src device=/dev/video" << id << " ! image/jpeg";
            if(parameters->camWidth1 != 0)
                ss << ",width=" << parameters->camWidth1;
            if (parameters->camHeight1 != 0)
                ss << ",height=" << parameters->camHeight1;
            ss << ",framerate=" << parameters->camFps1 << "/1";
            ss << " ! jpegdec ! video/x-raw,format=I420 ! videoconvert ! appsink";
            cap1 = cv::VideoCapture(ss.str(), apiPreference);
        }
        else
#endif
        {
            cap1 = cv::VideoCapture(i, apiPreference);
        }

    }
    else
    {			//if address is longer, we try to open it as an ip address
        cap1 = cv::VideoCapture(id, apiPreference);
    }

    if (!cap1.isOpened())
    {
        wxMessageDialog dial(NULL,
            parameters->language.TRACKER_CAMERA_START_ERROR, wxT("Error"), wxOK | wxICON_ERROR);
        dial.ShowModal();
        return;
    }
    //Sleep(1000);

    // On Linux and when GStreamer backend is used we already setup the camera pixel format,
    // width, height and FPS above when the GStreamer pipeline was created.
#if defined(__LINUX__)
    if ((apiPreference != cv::CAP_ANY) && (apiPreference != cv::CAP_GSTREAMER))
#endif
    {
        //cap1.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('m', 'j', 'p', 'g'));
        //cap1.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        if(parameters->camWidth1 != 0)
            cap1.set(cv::CAP_PROP_FRAME_WIDTH, parameters->camWidth1);
        if (parameters->camHeight1 != 0)
            cap1.set(cv::CAP_PROP_FRAME_HEIGHT, parameters->camHeight1);
        cap1.set(cv::CAP_PROP_FPS, parameters->camFps1);
    }
    if(parameters->cameraSettings1)
        cap1.set(cv::CAP_PROP_SETTINGS, 1);
    if (parameters->settingsParameters1)
    {
        cap1.set(cv::CAP_PROP_AUTOFOCUS, 0);
        cap1.set(cv::CAP_PROP_AUTO_EXPOSURE, parameters->cameraAutoexposure1);
        cap1.set(cv::CAP_PROP_EXPOSURE, parameters->cameraExposure1);
        cap1.set(cv::CAP_PROP_GAIN, parameters->cameraGain1);
    }

    double codec = 0x47504A4D; //code by FPaul. Should use MJPEG codec to enable fast framerates.
    cap1.set(cv::CAP_PROP_FOURCC, codec);

    cameraRunning1 = true;
    cameraThread1 = std::thread(&Tracker::CameraLoop1, this);
    cameraThread1.detach();
}

void Tracker::StartCamera2(std::string id, int apiPreference)
{
    if (cameraRunning2)
    {
        cameraRunning2 = false;
        mainThreadRunning = false;
        //cameraThread.join();
        sleep_millis(1000);
        return;
    }
    if (id.length() <= 2)		//if camera address is a single character, try to open webcam
    {
        int i = std::stoi(id);	//convert to int
#if defined(__LINUX__)
        // On Linux cv::VideoCapture does not work when GStreamer backend is used and
        // camera is set to MJPG pixel format. As a work around we manually setup the
        // GStreamer pipeline with suitable decoding before feeding the stream into
        // application.
        if ((apiPreference == cv::CAP_ANY) || (apiPreference == cv::CAP_GSTREAMER))
        {
            std::stringstream ss;
            ss << "v4l2src device=/dev/video" << id << " ! image/jpeg";
            if(parameters->camWidth2 != 0)
                ss << ",width=" << parameters->camWidth2;
            if (parameters->camHeight2 != 0)
                ss << ",height=" << parameters->camHeight2;
            ss << ",framerate=" << parameters->camFps2 << "/1";
            ss << " ! jpegdec ! video/x-raw,format=I420 ! videoconvert ! appsink";
            cap2 = cv::VideoCapture(ss.str(), apiPreference);
        }
        else
#endif
        {
            cap2 = cv::VideoCapture(i, apiPreference);
        }

    }
    else
    {			//if address is longer, we try to open it as an ip address
        cap2 = cv::VideoCapture(id, apiPreference);
    }

    if (!cap2.isOpened())
    {
        wxMessageDialog dial(NULL,
            parameters->language.TRACKER_CAMERA_START_ERROR, wxT("Error"), wxOK | wxICON_ERROR);
        dial.ShowModal();
        return;
    }
    //Sleep(1000);

    // On Linux and when GStreamer backend is used we already setup the camera pixel format,
    // width, height and FPS above when the GStreamer pipeline was created.
#if defined(__LINUX__)
    if ((apiPreference != cv::CAP_ANY) && (apiPreference != cv::CAP_GSTREAMER))
#endif
    {
        //cap2.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('m', 'j', 'p', 'g'));
        //cap2.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        if(parameters->camWidth2 != 0)
            cap2.set(cv::CAP_PROP_FRAME_WIDTH, parameters->camWidth2);
        if (parameters->camHeight2 != 0)
            cap2.set(cv::CAP_PROP_FRAME_HEIGHT, parameters->camHeight2);
        cap2.set(cv::CAP_PROP_FPS, parameters->camFps2);
    }
    if(parameters->cameraSettings2)
        cap2.set(cv::CAP_PROP_SETTINGS, 1);
    if (parameters->settingsParameters2)
    {
        cap2.set(cv::CAP_PROP_AUTOFOCUS, 0);
        cap2.set(cv::CAP_PROP_AUTO_EXPOSURE, parameters->cameraAutoexposure2);
        cap2.set(cv::CAP_PROP_EXPOSURE, parameters->cameraExposure2);
        cap2.set(cv::CAP_PROP_GAIN, parameters->cameraGain2);
    }

    double codec = 0x47504A4D; //code by FPaul. Should use MJPEG codec to enable fast framerates.
    cap2.set(cv::CAP_PROP_FOURCC, codec);

    cameraRunning2 = true;
    cameraThread2 = std::thread(&Tracker::CameraLoop2, this);
    cameraThread2.detach();
}

void Tracker::CameraLoop1()
{
    bool previewShown = false;
    bool rotate = false;
    int rotateFlag = -1;
    if (parameters->rotateCl1 && parameters->rotateCounterCl1)
    {
        rotate = true;
        rotateFlag = cv::ROTATE_180;
    }
    else if (parameters->rotateCl1)
    {
        rotate = true;
        rotateFlag = cv::ROTATE_90_CLOCKWISE;
    }
    else if (parameters->rotateCounterCl1)
    {
        rotate = true;
        rotateFlag = cv::ROTATE_90_COUNTERCLOCKWISE;
    }
    cv::Mat img;
    cv::Mat drawImg;
    double fps = 0;
    last_frame_time = clock();
    bool frame_visible = false;
    int cols, rows;

    while (cameraRunning1)
    {
        if (!cap1.read(img))
        {
            gui->CallAfter([parameters=parameters] ()
                           {
                           wxMessageDialog dial(NULL,
                               parameters->language.TRACKER_CAMERA_ERROR, wxT("Error"), wxOK | wxICON_ERROR);
                           dial.ShowModal();
                           });
            cameraRunning1 = false;
            break;
        }
        clock_t curtime = clock();
        fps = 0.95*fps + 0.05/(double(curtime - last_frame_time) / double(CLOCKS_PER_SEC));
        last_frame_time = curtime;        
        if (rotate)
        {
            cv::rotate(img, img, rotateFlag);
        }
        std::string resolution = std::to_string(img.cols) + "x" + std::to_string(img.rows);
        if (previewCamera1 || previewCameraCalibration1)
        {
            if (img.cols < img.rows)
            {
                cols = img.cols * drawImgSize / img.rows;
                rows = drawImgSize;
            }
            else
            {
                cols = drawImgSize;
                rows = img.rows * drawImgSize / img.cols;
            }
            img.copyTo(drawImg);
            cv::putText(drawImg, std::to_string((int)(fps + (0.5))), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
            cv::putText(drawImg, resolution, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
            if (previewCameraCalibration1)
            {
                cv::Mat *outImg = new cv::Mat();
                previewCalibration(drawImg, parameters);
                cv::resize(drawImg, *outImg, cv::Size(cols, rows));
                gui->CallAfter([outImg] ()
                               {
                               cv::imshow("Preview1", *outImg);
                               cv::waitKey(1);
                               delete(outImg);
                               });
                previewShown = true;
            }
            else
            {
                cv::Mat *outImg = new cv::Mat();
                cv::resize(drawImg, *outImg, cv::Size(cols, rows));
                gui->CallAfter([outImg] ()
                               {
                               cv::imshow("Preview1", *outImg);
                               cv::waitKey(1);
                               delete(outImg);
                               });
                previewShown = true;
            }
            frame_visible = true;
        }
        else if (previewShown) 
        {
                gui->CallAfter([] ()
                               {
                               cv::destroyWindow("Preview1");
                               });
                previewShown = false;
        }
        {
            std::lock_guard<std::mutex> lock(cameraImageMutex1);
            // Swap avoids copying the pixel buffer. It only swaps pointers and metadata.
            // The pixel buffer from cameraImage can be reused if the size and format matches.
            cv::swap(img, cameraImage1);
            if (img.size() != cameraImage1.size() || img.flags != cameraImage1.flags)
            {
                img.release();
            }
            imageReady1 = true;
        }

        if (!disableOpenVrApi)
        {
            //process events. BETA TEST ONLY, MOVE TO CONNECTION LATER
            if (connection->status == connection->CONNECTED)
            {
                vr::VREvent_t event;
                while (connection->openvr_handle->PollNextEvent(&event, sizeof(event)))
                {
                    if (event.eventType == vr::VREvent_Quit)
                    {
                        connection->openvr_handle->AcknowledgeQuit_Exiting();       //close connection to steamvr without closing att
                        connection->status = connection->DISCONNECTED;
                        vr::VR_Shutdown();
                        mainThreadRunning = false;
                        break;
                    }
                }
            }
        }
    }
    gui->CallAfter([] ()
                   {
                   cv::destroyAllWindows();
                   });
    cap1.release();
}

void Tracker::CameraLoop2()
{
    bool previewShown = false;
    bool rotate = false;
    int rotateFlag = -1;
    if (parameters->rotateCl2 && parameters->rotateCounterCl2)
    {
        rotate = true;
        rotateFlag = cv::ROTATE_180;
    }
    else if (parameters->rotateCl2)
    {
        rotate = true;
        rotateFlag = cv::ROTATE_90_CLOCKWISE;
    }
    else if (parameters->rotateCounterCl2)
    {
        rotate = true;
        rotateFlag = cv::ROTATE_90_COUNTERCLOCKWISE;
    }
    cv::Mat img;
    cv::Mat drawImg;
    double fps = 0;
    last_frame_time = clock();
    bool frame_visible = false;
    int cols, rows;

    while (cameraRunning2)
    {
        if (!cap2.read(img))
        {
            gui->CallAfter([parameters=parameters] ()
                           {
                           wxMessageDialog dial(NULL,
                               parameters->language.TRACKER_CAMERA_ERROR, wxT("Error"), wxOK | wxICON_ERROR);
                           dial.ShowModal();
                           });
            cameraRunning2 = false;
            break;
        }
        clock_t curtime = clock();
        fps = 0.95*fps + 0.05/(double(curtime - last_frame_time) / double(CLOCKS_PER_SEC));
        last_frame_time = curtime;        
        if (rotate)
        {
            cv::rotate(img, img, rotateFlag);
        }
        std::string resolution = std::to_string(img.cols) + "x" + std::to_string(img.rows);
        if (previewCamera2 || previewCameraCalibration2)
        {
            if (img.cols < img.rows)
            {
                cols = img.cols * drawImgSize / img.rows;
                rows = drawImgSize;
            }
            else
            {
                cols = drawImgSize;
                rows = img.rows * drawImgSize / img.cols;
            }
            img.copyTo(drawImg);
            cv::putText(drawImg, std::to_string((int)(fps + (0.5))), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
            cv::putText(drawImg, resolution, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
            if (previewCameraCalibration2)
            {
                cv::Mat *outImg = new cv::Mat();
                previewCalibration(drawImg, parameters);
                cv::resize(drawImg, *outImg, cv::Size(cols, rows));
                gui->CallAfter([outImg] ()
                               {
                               cv::imshow("Preview2", *outImg);
                               cv::waitKey(1);
                               delete(outImg);
                               });
                previewShown = true;
            }
            else
            {
                cv::Mat *outImg = new cv::Mat();
                cv::resize(drawImg, *outImg, cv::Size(cols, rows));
                gui->CallAfter([outImg] ()
                               {
                               cv::imshow("Preview2", *outImg);
                               cv::waitKey(1);
                               delete(outImg);
                               });
                previewShown = true;
            }
            frame_visible = true;
        }
        else if (previewShown) 
        {
                gui->CallAfter([] ()
                               {
                               cv::destroyWindow("Preview2");
                               });
                previewShown = false;
        }
        {
            std::lock_guard<std::mutex> lock(cameraImageMutex2);
            // Swap avoids copying the pixel buffer. It only swaps pointers and metadata.
            // The pixel buffer from cameraImage can be reused if the size and format matches.
            cv::swap(img, cameraImage2);
            if (img.size() != cameraImage2.size() || img.flags != cameraImage2.flags)
            {
                img.release();
            }
            imageReady2 = true;
        }

        if (!disableOpenVrApi)
        {
            //process events. BETA TEST ONLY, MOVE TO CONNECTION LATER
            if (connection->status == connection->CONNECTED)
            {
                vr::VREvent_t event;
                while (connection->openvr_handle->PollNextEvent(&event, sizeof(event)))
                {
                    if (event.eventType == vr::VREvent_Quit)
                    {
                        connection->openvr_handle->AcknowledgeQuit_Exiting();       //close connection to steamvr without closing att
                        connection->status = connection->DISCONNECTED;
                        vr::VR_Shutdown();
                        mainThreadRunning = false;
                        break;
                    }
                }
            }
        }
    }
    gui->CallAfter([] ()
                   {
                   cv::destroyAllWindows();
                   });
    cap2.release();
}

void Tracker::CopyFreshCameraImageTo1(cv::Mat& image)
{
    // Sleep happens between each iteration when the mutex is not locked.
    for (;;sleep_millis(1))
    {
        std::lock_guard<std::mutex> lock(cameraImageMutex1);
        if (imageReady1)
        {
            imageReady1 = false;
            // Swap metadata and pointers to pixel buffers.
            cv::swap(image, cameraImage1);
            // We don't want to overwrite shared data so release the image unless we are the only user of it.
            if (!(cameraImage1.u && cameraImage1.u->refcount == 1))
            {
                cameraImage1.release();
            }
            return;
        }
    }
}

void Tracker::CopyFreshCameraImageTo2(cv::Mat& image)
{
    // Sleep happens between each iteration when the mutex is not locked.
    for (;;sleep_millis(1))
    {
        std::lock_guard<std::mutex> lock(cameraImageMutex2);
        if (imageReady2)
        {
            imageReady2 = false;
            // Swap metadata and pointers to pixel buffers.
            cv::swap(image, cameraImage2);
            // We don't want to overwrite shared data so release the image unless we are the only user of it.
            if (!(cameraImage2.u && cameraImage2.u->refcount == 1))
            {
                cameraImage2.release();
            }
            return;
        }
    }
}

void DialogOk(void *data)
{
    Tracker *tracker = static_cast<Tracker *>(data);
    tracker->messageDialogResponse = wxID_OK;
    tracker->mainThreadRunning = false;
}

void DialogCancel(void *data)
{
    Tracker *tracker = static_cast<Tracker *>(data);
    tracker->messageDialogResponse = wxID_CANCEL;
    tracker->mainThreadRunning = false;
}

void Tracker::StartCameraCalib()
{
    if (mainThreadRunning)
    {
        mainThreadRunning = false;
        return;
    }
    if (!cameraRunning1)
    {
        bool *mtr = &mainThreadRunning;
        wxString e = parameters->language.TRACKER_CAMERA_NOTRUNNING;
        gui->CallAfter([e, mtr] ()
                        {
                            wxMessageDialog dial(NULL, e, wxT("Error"), wxOK | wxICON_ERROR);
                            dial.ShowModal();
                            *mtr = false;
                        });
        return;
    }

    mainThreadRunning = true;
    if(!parameters->chessboardCalib)
    {
        mainThread1 = std::thread(&Tracker::CalibrateCameraCharuco, this);
    }
    else
    {
        mainThread1 = std::thread(&Tracker::CalibrateCamera, this);
    }
    mainThread1.detach();
}

void Tracker::CalibrateCameraCharuco()
{
    //function to calibrate our camera

    cv::Mat image;
    cv::Mat gray;
    cv::Mat drawImg;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

    //generate and show our charuco board that will be used for calibration
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(8, 7, 0.04f, 0.02f, dictionary);
    cv::Mat boardImage;
    //board->draw(cv::Size(1500, 1000), boardImage, 10, 1);
    //imshow("calibration", boardImage);
    //cv::imwrite("charuco_board.jpg", boardImage);
    //cv::waitKey(1);

    //set our detectors marker border bits to 1 since thats what charuco uses
    params->markerBorderBits = 1;

    //int framesSinceLast = -2 * parameters->camFps;
    clock_t timeOfLast = clock();

    int messageDialogResponse = wxID_CANCEL;
    std::thread th{ [this, &messageDialogResponse]() {
        wxString e = parameters->language.TRACKER_CAMERA_CALIBRATION_INSTRUCTIONS;
        int *mdr = &messageDialogResponse;
        bool *mtr = &mainThreadRunning;
        gui->CallAfter([e, mdr, mtr] ()
            {
            wxMessageDialog dial(NULL, e, wxT("Message"), wxOK | wxCANCEL);
            *mdr = dial.ShowModal();
            *mtr = false;
            });
        
    } };

    th.detach();

    cv::Mat cameraMatrix, distCoeffs, R, T;
    cv::Mat1d stdDeviationsIntrinsics, stdDeviationsExtrinsics;
    std::vector<double> perViewErrors;
    std::vector<std::vector<cv::Point2f>> allCharucoCorners;
    std::vector<std::vector<int>> allCharucoIds;

    int picsTaken = 0;
    while(mainThreadRunning && cameraRunning1)
    {
        CopyFreshCameraImageTo1(image);
        int cols, rows;
        if (image.cols < image.rows)
        {
            cols = image.cols * drawImgSize / image.rows;
            rows = drawImgSize;
        }
        else
        {
            cols = drawImgSize;
            rows = image.rows * drawImgSize / image.cols;
        }

        image.copyTo(drawImg);
        cv::putText(drawImg, std::to_string(picsTaken), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));

        previewCalibration(
            drawImg,
            cameraMatrix,
            distCoeffs,
            stdDeviationsIntrinsics,
            perViewErrors,
            allCharucoCorners,
            allCharucoIds);

        //check the highest per view error and remove it if its higher than 1px.

        if (perViewErrors.size() > 10)
        {
            double maxPerViewError = 0;
            int maxPerViewErrorIdx = 0;

            for (int i = 0; i < perViewErrors.size(); i++)
            {
                if (perViewErrors[i] > maxPerViewError)
                {
                    maxPerViewError = perViewErrors[i];
                    maxPerViewErrorIdx = i;
                }
            }

            if (maxPerViewError > 1)
            {
                perViewErrors.erase(perViewErrors.begin() + maxPerViewErrorIdx);
                allCharucoCorners.erase(allCharucoCorners.begin() + maxPerViewErrorIdx);
                allCharucoIds.erase(allCharucoIds.begin() + maxPerViewErrorIdx);

                // recalibrate camera without the problematic frame
                cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, board, cv::Size(image.rows, image.cols),
                    cameraMatrix, distCoeffs, R, T, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors,
                    cv::CALIB_USE_LU);

                picsTaken--;
            }
        }

        cv::Mat *outImg = new cv::Mat();
        cv::resize(drawImg, *outImg, cv::Size(cols, rows));
        char key;
        gui->CallAfter([outImg, &key] ()
                        {
                        cv::imshow("out", *outImg);
                        key = (char)cv::waitKey(1);
                        delete(outImg);
                        });

        //framesSinceLast++;
        if (key != -1 || double(clock() - timeOfLast) / (2*double(CLOCKS_PER_SEC)) > 1)
        {
            //framesSinceLast = 0;
            //if any button was pressed
            cvtColor(image, gray, cv::COLOR_BGR2GRAY);

            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            std::vector<std::vector<cv::Point2f>> rejectedCorners;

            //detect our markers
            cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, params, rejectedCorners);
            cv::aruco::refineDetectedMarkers(gray, board, markerCorners, markerIds, rejectedCorners);

            if (markerIds.size() > 0)
            {
                //if markers were found, try to add calibration data
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                //using data from aruco detection we refine the search of chessboard corners for higher accuracy
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, gray, board, charucoCorners, charucoIds);
                if (charucoIds.size() > 15)
                {
                    //if corners were found, we draw them
                    cv::aruco::drawDetectedCornersCharuco(drawImg, charucoCorners, charucoIds);
                    //we then add our corners to the array
                    allCharucoCorners.push_back(charucoCorners);
                    allCharucoIds.push_back(charucoIds);
                    picsTaken++;

                    cv::Mat *outImg = new cv::Mat();
                    cv::resize(drawImg, *outImg, cv::Size(cols, rows));
                    gui->CallAfter([outImg] ()
                                    {
                                    cv::imshow("out", *outImg);
                                    cv::waitKey(1);
                                    delete(outImg);
                                    });
                    
                    if (picsTaken >= 3)
                    {
                        try
                        {
                            // Calibrate camera using our data
                            cv::aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, board, cv::Size(image.rows, image.cols),
                                cameraMatrix, distCoeffs, R, T, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors,
                                cv::CALIB_USE_LU);
                        }
                        catch(const cv::Exception& e)
                        {
                            std::cerr << "Failed to calibrate: " << e.what();
                        }

                        int curI = perViewErrors.size();

                    }
                }
            }
            timeOfLast = clock();
        }
    }

    gui->CallAfter([] ()
                   {
                   cv::destroyWindow("out");
                   });
    mainThreadRunning = false;
    if (messageDialogResponse == wxID_OK)
    {
        if (cameraMatrix.empty())
        {
            wxString e = parameters->language.TRACKER_CAMERA_CALIBRATION_NOTDONE;
            gui->CallAfter([e] ()
                           {
                           wxMessageDialog dial(NULL, e, wxT("Info"), wxOK | wxICON_ERROR);
                           dial.ShowModal();
                           });

        }
        else
        {

            //some checks of the camera calibration values. The thresholds should be adjusted to prevent any false  negatives
            double avgPerViewError = 0;
            double maxPerViewError = 0;

            for (int i = 0; i < perViewErrors.size(); i++)
            {
                avgPerViewError += perViewErrors[i];
                if (perViewErrors[i] > maxPerViewError)
                    maxPerViewError = perViewErrors[i];
            }

            avgPerViewError /= perViewErrors.size();
            /*
            if (avgPerViewError > 0.5)          //a big reprojection error indicates that calibration wasnt done properly
            {
                wxMessageDialog dial(NULL, wxT("WARNING:\nThe avarage reprojection error is over 0.5 pixel. This usualy indicates a bad calibration."), wxT("Warning"), wxOK | wxICON_ERROR);
                dial.ShowModal();
            }
            if (maxPerViewError > 10)           //having one reprojection error very high indicates that one frame had missdetections
            {
                wxMessageDialog dial(NULL, wxT("WARNING:\nOne or more reprojection errors are over 10 pixels. This usualy indicates something went wrong during calibration."), wxT("Warning"), wxOK | wxICON_ERROR);
                dial.ShowModal();
            }
            
            volatile double test = stdDeviationsIntrinsics.at<double>(0);
            test = stdDeviationsIntrinsics.at<double>(1); 
            test = stdDeviationsIntrinsics.at<double>(2); 
            test = stdDeviationsIntrinsics.at<double>(3);
            
            if (stdDeviationsIntrinsics.at<double>(0) > 5 || stdDeviationsIntrinsics.at<double>(1) > 5)         //high uncertiancy is bad
            {
                wxMessageDialog dial(NULL, wxT("WARNING:\nThe calibration grid doesnt seem very stable. This usualy indicates a bad calibration."), wxT("Warning"), wxOK | wxICON_ERROR);
                dial.ShowModal();
            }
            */
            // Save calibration to our global params cameraMatrix and distCoeffs
            parameters->camMat1 = cameraMatrix;
            parameters->distCoeffs1 = distCoeffs;
            parameters->stdDeviationsIntrinsics1 = stdDeviationsIntrinsics;
            parameters->perViewErrors1 = perViewErrors;
            parameters->allCharucoCorners1 = allCharucoCorners;
            parameters->allCharucoIds1 = allCharucoIds;
            parameters->Save();
            wxString e = parameters->language.TRACKER_CAMERA_CALIBRATION_COMPLETE;
            gui->CallAfter([e] ()
                           {
                           wxMessageDialog dial(NULL, e, wxT("Info"), wxOK);
                           dial.ShowModal();
                           });
        }
    }
}

void Tracker::CalibrateCamera()
{

    int CHECKERBOARD[2]{ 7,7 };

    int blockSize = 125;
    int imageSizeX = blockSize * (CHECKERBOARD[0] + 1);
    int imageSizeY = blockSize * (CHECKERBOARD[1] + 1);
    cv::Mat chessBoard(imageSizeX, imageSizeY, CV_8UC3, cv::Scalar::all(0));
    unsigned char color = 0;

    for (int i = 0; i < imageSizeX-1; i = i + blockSize) {
        if(CHECKERBOARD[1]%2 == 1)
            color = ~color;
        for (int j = 0; j < imageSizeY-1; j = j + blockSize) {
            cv::Mat ROI = chessBoard(cv::Rect(j, i, blockSize, blockSize));
            ROI.setTo(cv::Scalar::all(color));
            color = ~color;
        }
    }
    //cv::namedWindow("Chessboard", cv::WINDOW_KEEPRATIO);
    //imshow("Chessboard", chessBoard);
    //cv::imwrite("chessboard.png", chessBoard);

    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints;
    std::vector<cv::Point3f> objp;

    for (int i{ 0 }; i < CHECKERBOARD[0]; i++)
    {
        for (int j{ 0 }; j < CHECKERBOARD[1]; j++)
        {
            objp.push_back(cv::Point3f(j, i, 0));
        }
    }

    std::vector<cv::Point2f> corner_pts;
    bool success;

    cv::Mat image;

    int i = 0;
    int framesSinceLast = -100;

    int picNum = parameters->cameraCalibSamples;

    while (i < picNum)
    {
        if (!mainThreadRunning || !cameraRunning1)
        {
            gui->CallAfter([] ()
                           {
                           cv::destroyAllWindows();
                           });
            return;
        }
        CopyFreshCameraImageTo1(image);
        cv::putText(image, std::to_string(i) + "/" + std::to_string(picNum), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
        int cols, rows;
        if (image.cols < image.rows)
        {
            cols = image.cols * drawImgSize / image.rows;
            rows = drawImgSize;
        }
        else
        {
            cols = drawImgSize;
            rows = image.rows * drawImgSize / image.cols;
        }
        {
            cv::Mat *outImg = new cv::Mat();
            cv::resize(image, *outImg, cv::Size(cols,rows));
            gui->CallAfter([outImg] ()
                           {
                           cv::imshow("out", *outImg);
                           cv::waitKey(1);
                           delete(outImg);
                           });
        }
        framesSinceLast++;
        if (framesSinceLast > 50)
        {
            framesSinceLast = 0;
            cv::cvtColor(image, image,cv:: COLOR_BGR2GRAY);

            success = findChessboardCorners(image, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts);

            if (success)
            {
                i++;
                cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

                cornerSubPix(image, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

                drawChessboardCorners(image, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

                objpoints.push_back(objp);
                imgpoints.push_back(corner_pts);
            }

            {
                cv::Mat *outImg = new cv::Mat();
                cv::resize(image, *outImg, cv::Size(cols,rows));
                gui->CallAfter([outImg] ()
                               {
                               cv::imshow("out", *outImg);
                               cv::waitKey(1);
                               delete(outImg);
                               });
            }
            sleep_millis(1000);
        }
    }

    cv::Mat cameraMatrix, distCoeffs, R, T;

    calibrateCamera(objpoints, imgpoints, cv::Size(image.rows, image.cols), cameraMatrix, distCoeffs, R, T);

    parameters->camMat1 = cameraMatrix;
    parameters->distCoeffs1 = distCoeffs;
    parameters->Save();
    mainThreadRunning = false;
    gui->CallAfter([] ()
                   {
                   cv::destroyAllWindows();
                   wxMessageDialog dial(NULL,
                       wxT("Calibration complete."), wxT("Info"), wxOK);
                   dial.ShowModal();
                   });
}

void Tracker::StartTrackerCalib()
{
    if (mainThreadRunning)
    {
        mainThreadRunning = false;
        return;
    }
    if (!cameraRunning1)
    {
        wxString e = parameters->language.TRACKER_CAMERA_NOTRUNNING;
        bool *mtr = &mainThreadRunning;
        gui->CallAfter([e, mtr] ()
                       {
                       wxMessageDialog dial(NULL, e, wxT("Error"), wxOK | wxICON_ERROR);
                       dial.ShowModal();
                       *mtr = false;
                       });
        return;
    }
    if (parameters->camMat1.empty())
    {
        wxString e = parameters->language.TRACKER_CAMERA_NOTCALIBRATED;
        bool *mtr = &mainThreadRunning;
        gui->CallAfter([e, mtr] ()
                       {
                       wxMessageDialog dial(NULL, e, wxT("Error"), wxOK | wxICON_ERROR);
                       dial.ShowModal();
                       *mtr = false;
                       });
        return;
    }

    mainThreadRunning = true;
    mainThread1 = std::thread(&Tracker::CalibrateTracker, this);
    mainThread1.detach();


    //make a new thread with message box, and stop main thread when we press OK
    std::thread th{ [=]() {
        wxString e = parameters->language.TRACKER_TRACKER_CALIBRATION_INSTRUCTIONS;
        bool *mtr = &mainThreadRunning;
        gui->CallAfter([e, mtr] ()
                        {
                        wxMessageDialog dial(NULL, e, wxT("Message"), wxOK);
                        dial.ShowModal();
                        *mtr = false;
                        });
    } };

    th.detach();
}

void Tracker::Start()
{
    if (mainThreadRunning)
    {
        mainThreadRunning = false;
        return;
    }
    if (!cameraRunning1)
    {
        wxMessageDialog dial(NULL,
            parameters->language.TRACKER_CAMERA_NOTRUNNING, wxT("Error"), wxOK | wxICON_ERROR);
        dial.ShowModal();
        mainThreadRunning = false;
        return;
    }
    if (parameters->camMat1.empty())
    {
        wxMessageDialog dial(NULL,
            parameters->language.TRACKER_CAMERA_NOTCALIBRATED, wxT("Error"), wxOK | wxICON_ERROR);
        dial.ShowModal();
        mainThreadRunning = false;
        return;
    }
    if (!cameraRunning2)
    {
        wxMessageDialog dial(NULL,
            parameters->language.TRACKER_CAMERA_NOTRUNNING, wxT("Error"), wxOK | wxICON_ERROR);
        dial.ShowModal();
        mainThreadRunning = false;
        return;
    }
    if (parameters->camMat2.empty())
    {
        wxMessageDialog dial(NULL,
            parameters->language.TRACKER_CAMERA_NOTCALIBRATED, wxT("Error"), wxOK | wxICON_ERROR);
        dial.ShowModal();
        mainThreadRunning = false;
        return;
    }
    if (!trackersCalibrated)
    {
        wxMessageDialog dial(NULL,
            parameters->language.TRACKER_TRACKER_NOTCALIBRATED, wxT("Error"), wxOK | wxICON_ERROR);
        dial.ShowModal();
        mainThreadRunning = false;
        return;
    }
    mainThreadRunning = true;
    mainThread1 = std::thread(&Tracker::MainLoop1, this);
    mainThread1.detach();
    mainThread2 = std::thread(&Tracker::MainLoop2, this);
    mainThread2.detach();
}

void Tracker::CalibrateTracker()
{
    std::vector<std::vector<int>> boardIds;
    std::vector<std::vector < std::vector<cv::Point3f >>> boardCorners;
    std::vector<bool> boardFound;

    //making a marker model of our markersize for later use
    std::vector<cv::Point3f> modelMarker;
    double markerSize = parameters->markerSize;
    modelMarker.push_back(cv::Point3f(-markerSize / 2, markerSize / 2, 0));
    modelMarker.push_back(cv::Point3f(markerSize / 2, markerSize / 2, 0));
    modelMarker.push_back(cv::Point3f(markerSize / 2, -markerSize / 2, 0));
    modelMarker.push_back(cv::Point3f(-markerSize / 2, -markerSize / 2, 0));

    AprilTagWrapper april{parameters};

    int markersPerTracker = parameters->markersPerTracker;
    int trackerNum = parameters->trackerNum;

    std::vector<cv::Vec3d> boardRvec, boardTvec;

    for (int i = 0; i < trackerNum; i++)
    {
        std::vector<int > curBoardIds;
        std::vector < std::vector<cv::Point3f >> curBoardCorners;
        curBoardIds.push_back(i * markersPerTracker);
        curBoardCorners.push_back(modelMarker);
        boardIds.push_back(curBoardIds);
        boardCorners.push_back(curBoardCorners);
        boardFound.push_back(false);
        boardRvec.push_back(cv::Vec3d());
        boardTvec.push_back(cv::Vec3d());
    }
    cv::Mat image;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    std::vector<int> idsList;
    std::vector<std::vector < std::vector<cv::Point3f >>> cornersList;

    trackers.clear();

    while (cameraRunning1 && mainThreadRunning)
    {
        CopyFreshCameraImageTo1(image);

        clock_t start;
        //clock for timing of detection
        start = clock();

        //detect and draw all markers on image
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        std::vector<cv::Point2f> centers;

        //cv::aruco::detectMarkers(image, dictionary, corners, ids, params);
        april.detectMarkers(image, &corners, &ids, &centers,trackers);
        if (showTimeProfile)
        {
            april.drawTimeProfile(image, cv::Point(10, 60));
        }

        cv::aruco::drawDetectedMarkers(image, corners, cv::noArray(), cv::Scalar(255, 0, 0));       //draw all markers blue. We will overwrite this with other colors for markers that are part of any of the trackers that we use

        //estimate pose of our markers
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, markerSize, parameters->camMat1, parameters->distCoeffs1, rvecs, tvecs);
        /*
        for (int i = 0; i < rvecs.size(); ++i) {
            //draw axis for each marker
            auto rvec = rvecs[i];	//rotation vector of our marker
            auto tvec = tvecs[i];	//translation vector of our marker

            //rotation/translation vectors are shown as offset of our camera from the marker

            cv::aruco::drawAxis(image, parameters->camMat, parameters->distCoeffs, rvec, tvec, parameters->markerSize);
        }
        */

        float maxDist = parameters->trackerCalibDistance;

        for (int i = 0; i < boardIds.size(); i++)           //for each of the trackers
        {
            cv::Ptr<cv::aruco::Board> arBoard = cv::aruco::Board::create(boardCorners[i], dictionary, boardIds[i]);         //create an aruco board object made out of already added markers to current tracker
            //cv::Vec3d boardRvec, boardTvec;
            //bool boardFound = false;
            try
            {
                if (cv::aruco::estimatePoseBoard(corners, ids, arBoard, parameters->camMat1, parameters->distCoeffs1, boardRvec[i], boardTvec[i], false) > 0)         //try to estimate current trackers pose
                {
                    cv::aruco::drawAxis(image, parameters->camMat1, parameters->distCoeffs1, boardRvec[i], boardTvec[i], 0.1f);       //if found, draw axis and mark it found
                    boardFound[i] = true;
                }
                else
                {
                    boardFound[i] = false;          //else, if none of the markers for this tracker are visible, mark it not found
                }
            }
            catch (std::exception&)             //on weird images or calibrations, we get an error
            {
                wxString e = parameters->language.TRACKER_CALIBRATION_SOMETHINGWRONG;
                bool *mtr = &mainThreadRunning;
                gui->CallAfter([e, mtr] ()
                                {
                                wxMessageDialog dial(NULL, e, wxT("Error"), wxOK | wxICON_ERROR);
                                dial.ShowModal();
                                cv::destroyWindow("out");
                                *mtr = false;
                                });
                return;
                
            }

            std::string testStr = std::to_string(boardTvec[i][0]) + " " + std::to_string(boardTvec[i][1]) + " " + std::to_string(boardTvec[i][2]);

            bool foundMarkerToCalibrate = false;

            for (int j = 0; j < ids.size(); j++)        //check all of the found markers
            {
                if (ids[j] >= i * markersPerTracker && ids[j] < (i + 1) * markersPerTracker)            //if marker is part of current tracker
                {
                    bool markerInBoard = false;
                    for (int k = 0; k < boardIds[i].size(); k++)        //check if marker is already part of the tracker
                    {
                        if (boardIds[i][k] == ids[j])          
                        {
                            markerInBoard = true;
                            break;
                        }
                    }
                    if (markerInBoard == true)          //if it is, draw it green and continue to next marker
                    {
                        drawMarker(image, corners[j], cv::Scalar(0, 255, 0));
                        continue;
                    }
                    if (boardFound[i])                  //if it isnt part of the current tracker, but the tracker was detected, we will attempt to add it
                    {
                        if (sqrt(tvecs[j][0] * tvecs[j][0] + tvecs[j][1] * tvecs[j][1] + tvecs[j][2] * tvecs[j][2]) > maxDist)          //if marker is too far away from camera, we just paint it purple as adding it could have too much error
                        {
                            drawMarker(image, corners[j], cv::Scalar(255, 0, 255));
                            continue;
                        }

                        drawMarker(image, corners[j], cv::Scalar(0, 255, 255));         //start adding marker, mark that by painting it yellow
 
                        if (foundMarkerToCalibrate)                     //only calibrate one marker at a time
                            continue;

                        foundMarkerToCalibrate = true;


                        std::vector<cv::Point3f> marker;
                        transformMarkerSpace(modelMarker, boardRvec[i], boardTvec[i], rvecs[j], tvecs[j], &marker);         //transform marker points to the coordinate system of the tracker

                        int listIndex = -1;
                        for (int k = 0; k < idsList.size(); k++)            //check whether the idsList and cornersList already contains data for this marker
                        {
                            if (idsList[k] == ids[j])
                            {
                                listIndex = k;
                            }
                        }
                        if (listIndex < 0)                  //if not, add and initialize it
                        {
                            listIndex = idsList.size();
                            idsList.push_back(ids[j]);
                            cornersList.push_back(std::vector<std::vector<cv::Point3f>>());
                        }

                        cornersList[listIndex].push_back(marker);       //add the current marker corners to the list
                        if (cornersList[listIndex].size() > 50)         //if we have 50 recorded instances in the list for current marker, we can add it to the tracker
                        {
                            std::vector<cv::Point3f> medianMarker;

                            getMedianMarker(cornersList[listIndex], &medianMarker);         //calculate median position of each corner to get rid of outliers

                            boardIds[i].push_back(ids[j]);                                  //add the marker to the tracker
                            boardCorners[i].push_back(medianMarker);
                        }
                    }
                    else
                    {
                        drawMarker(image, corners[j], cv::Scalar(0, 0, 255));
                    }
                }
            }
        }
        int cols, rows;
        if (image.cols < image.rows)
        {
            cols = image.cols * drawImgSize / image.rows;
            rows = drawImgSize;
        }
        else
        {
            cols = drawImgSize;
            rows = image.rows * drawImgSize / image.cols;
        }
        cv::Mat *outImg = new cv::Mat();
        cv::resize(image, *outImg, cv::Size(cols,rows));
        gui->CallAfter([outImg] ()
                        {
                        cv::imshow("out", *outImg);
                        cv::waitKey(1);
                        delete(outImg);
                        });
    }

    for (int i = 0; i < boardIds.size(); i++)
    {
        cv::Ptr<cv::aruco::Board> arBoard = cv::aruco::Board::create(boardCorners[i], dictionary, boardIds[i]);
        trackers.push_back(arBoard);
    }

    parameters->trackers = trackers;
    parameters->Save();
    trackersCalibrated = true;

    bool *mtr = &mainThreadRunning;
    gui->CallAfter([mtr] ()
                   {
                   cv::destroyWindow("out");
                   *mtr = false;
                   });
}

void Tracker::MainLoop1()
{

    int trackerNum = parameters->trackerNum;
    int numOfPrevValues = parameters->numOfPrevValues;

    //these variables are used to save detections of apriltags, so we dont define them every frame

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<cv::Point2f> centers;


    cv::Mat image, drawImg, ycc, gray, cr;

    cv::Mat  prevImg;


    //setup all variables that need to be stored for each tracker and initialize them
    std::vector<TrackerStatus> trackerStatus = std::vector<TrackerStatus>(trackerNum, TrackerStatus());
    for (int i = 0; i < trackerStatus.size(); i++)
    {
        trackerStatus[i].boardFound = false;
        trackerStatus[i].boardRvec = cv::Vec3d(0, 0, 0);
        trackerStatus[i].boardTvec = cv::Vec3d(0, 0, 0);
        trackerStatus[i].prevLocValues = std::vector<std::vector<double>>(7, std::vector<double>());
        trackerStatus[i].last_update_timestamp = std::chrono::milliseconds(0);
        trackerStatus[i].pose_valid = -1;
        trackerStatus[i].a = 0;
        trackerStatus[i].b = 0;
        trackerStatus[i].c = 0;
        trackerStatus[i].qw = 1;
        trackerStatus[i].qx = 0;
        trackerStatus[i].qy = 0;
        trackerStatus[i].qz = 0;
    }

    //previous values, used for moving median to remove any outliers.
    std::vector<double> prevLocValuesX;

    //the X axis, it is simply numbers 0-10 (or the amount of previous values we have)
    for (int j = 0; j < numOfPrevValues; j++)
    {
        prevLocValuesX.push_back(j);
    }


    AprilTagWrapper april{parameters};

    int framesSinceLastSeen = 0;
    int framesToCheckAll = 20;

    cv::Mat stationPos = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
    stationPos = wtranslation1 * stationPos;

    Quaternion<double> stationQ = Quaternion<double>(0, 0, 1, 0) * (wrotation1 * Quaternion<double>(1, 0, 0, 0));

    double a = -stationPos.at<double>(0, 0);
    double b = stationPos.at<double>(1, 0);
    double c = -stationPos.at<double>(2, 0);

    connection->SendStation(0, a, b, c, stationQ.w, stationQ.x, stationQ.y, stationQ.z);

    bool calibControllerPosActive = false;
    bool calibControllerAngleActive = false;
    clock_t calibControllerLastPress = clock();
    double calibControllerPosOffset[] = { 0,0,0 };
    double calibControllerAngleOffset[] = { 0,0,0 };

    //double tempScale = 1;

    std::vector<cv::Ptr<cv::aruco::Board>> trackers;
    std::vector<std::vector<int>> boardIds;
    std::vector<std::vector < std::vector<cv::Point3f >>> boardCorners;
    std::vector<cv::Point3f > boardCenters;

    {
        std::lock_guard<std::mutex> lock(calibratorMutex);
        frameCount1 = 0;
        calibratorProjected1.clear();
        calibratorReprojected1.clear();
        calibratorPoints1.clear();
    }

    for (int i = 0; i < this->trackers.size(); i++)
    {
        boardCorners.push_back(this->trackers[i]->objPoints);
        boardIds.push_back(this->trackers[i]->ids);
        boardCenters.push_back(cv::Point3f(0, 0, 0));

        int numOfCorners = 0;
        for (int j = 0; j < boardCorners[i].size(); j++)
        {
            for (int k = 0; k < boardCorners[i][j].size(); k++)
            {
                boardCenters[i].x += boardCorners[i][j][k].x;
                boardCenters[i].y += boardCorners[i][j][k].y;
                boardCenters[i].z += boardCorners[i][j][k].z;
                numOfCorners++;
            }
        }
        boardCenters[i] /= numOfCorners;
    }

    if (parameters->trackerCalibCenters)
    {
        for (int i = 0; i < this->trackers.size(); i++)
        {
            for (int j = 0; j < boardCorners[i].size(); j++)
            {
                for (int k = 0; k < boardCorners[i][j].size(); k++)
                {
                    boardCorners[i][j][k] -= boardCenters[i];
                    boardCenters[i] = cv::Point3f(0, 0, 0);
                }
            }

            cv::Ptr<cv::aruco::Board> arBoard = cv::aruco::Board::create(boardCorners[i], cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), boardIds[i]);
            trackers.push_back(arBoard);
        }
    }
    else
    {
        trackers = this->trackers;
    }

    gui->CallAfter([] ()
                   {
                   cv::namedWindow("out1");
                   });

    while (mainThreadRunning && cameraRunning1)
    {

        CopyFreshCameraImageTo1(image);

        drawImg = image;
        cv::Mat drawImgMasked = cv::Mat::zeros(drawImg.size(), drawImg.type());
        april.convertToSingleChannel(image, gray);

        clock_t start, end;
        //for timing our detection
        start = clock();

        bool circularWindow = parameters->circularWindow;

        //last is if pose is valid: 0 is valid, 1 is late (hasnt been updated for more than 0.2 secs), -1 means invalid and is only zeros
        std::vector<int> tracker_pose_valid = std::vector<int>(trackerNum, -1);

        for (int i = 0; i < trackerNum; i++)
        {
            if (!trackerStatus[i].boardFound)
            {
                framesSinceLastSeen++;
                if (framesSinceLastSeen > framesToCheckAll)
                    circularWindow = false;
                break;
            }
        }
        if (!circularWindow)
            framesSinceLastSeen = 0;

        for (int i = 0; i < trackerNum; i++)
        {

            double frameTime = double(clock() - last_frame_time) / double(CLOCKS_PER_SEC);

            double a; double b; double c;
            double qw; double qx; double qy; double qz;

            a = trackerStatus[i].a;
            b = trackerStatus[i].b;
            c = trackerStatus[i].c;
            qw = trackerStatus[i].qw;
            qx = trackerStatus[i].qx;
            qy = trackerStatus[i].qy;
            qz = trackerStatus[i].qz;
            tracker_pose_valid[i] = trackerStatus[i].pose_valid;

            cv::Vec3d rvec1;
            cv::Vec3d tvec1;

            tvec1 = trackerStatus[i].boardTvec;
            rvec1 = trackerStatus[i].boardRvec;

            std::vector<cv::Point3f> offsetin1, offsetout1;
            offsetin1.push_back(boardCenters[i]);
            offsetFromBoardToCameraSpace(offsetin1, rvec1, tvec1, &offsetout1);

            std::vector<cv::Point3d> point1;
            point1.push_back(cv::Point3d(trackerStatus[i].boardTvec));
            point1.push_back(cv::Point3d(trackerStatus[i].boardTvec) + cv::Point3d(offsetout1[0]));

            std::vector<cv::Point2d> projected1, projected2;
            cv::Vec3d identity_rvec, identity_tvec;

            cv::projectPoints(point1, identity_rvec, identity_tvec, parameters->camMat1, parameters->distCoeffs1, projected1);

            if (tracker_pose_valid[i] == 0)
            {
                cv::Vec3d rvec2;
                cv::Vec3d tvec2;

                //transform boards position based on our calibration data
                cv::Mat rpos = (cv::Mat_<double>(4, 1) << -a, b, -c, 1);

                //transform boards position based on our calibration data

                rpos.at<double>(3, 0) = 1;
                rpos = wtranslation2.inv() * rpos;

                Quaternion<double> q = Quaternion<double>(qw, qx, qy, qz);
                q = q.UnitQuaternion();

                //q = Quaternion<double>(0, 0, 1, 0) * (wrotation2 * q) * Quaternion<double>(0, 0, 1, 0);
                q = wrotation2.inverse() * Quaternion<double>(0, 0, 1, 0).inverse() * q * Quaternion<double>(0, 0, 1, 0).inverse();

                rvec2 = quat2rodr(q.w, q.x, q.y, q.z);
                tvec2[0] = rpos.at<double>(0, 0);
                tvec2[1] = rpos.at<double>(1, 0);
                tvec2[2] = rpos.at<double>(2, 0);

                std::vector<cv::Point3f> offsetin2, offsetout2;
                offsetin2.push_back(boardCenters[i]);
                offsetFromBoardToCameraSpace(offsetin2, rvec2, tvec2, &offsetout2);

                std::vector<cv::Point3d> point2;
                point2.push_back(cv::Point3d(tvec2));
                point2.push_back(cv::Point3d(tvec2) + cv::Point3d(offsetout2[0]));

                cv::projectPoints(point2, identity_rvec, identity_tvec, parameters->camMat2, parameters->distCoeffs2, projected2);
            }

            cv::circle(drawImg, projected1[0], 5, cv::Scalar(0, 0, 255), 2, 8, 0);
            cv::circle(drawImgMasked, projected1[0], 5, cv::Scalar(0, 0, 255), 2, 8, 0);
            cv::circle(drawImg, projected1[1], 5, cv::Scalar(255, 0, 255), 2, 8, 0);
            cv::circle(drawImgMasked, projected1[1], 5, cv::Scalar(255, 0, 255), 2, 8, 0);

            std::lock_guard<std::mutex> lock(calibratorMutex);

            if (trackerStatus[i].boardFound)
            {
                trackerStatus[i].maskCenter = projected1[1];
                if (true) //(frameCount1++ % 5 == 0)
                {
                    calibratorProjected1.push_back(projected1[1]);
                    if (tracker_pose_valid[i] == 0)
                    {
                        calibratorReprojected2.push_back(projected2[1]);
                    }

                    cv::Mat rpos = (cv::Mat_<double>(4, 1) << point1[1].x, point1[1].y, point1[1].z, 1);
                    rpos = wtranslation1 * rpos;

                    calibratorPoints1.push_back(cv::Point3d(rpos.at<double>(0,0), rpos.at<double>(1,0), rpos.at<double>(2,0)));
                    calibratorTimes1.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

                    if (calibratorPoints1.size() > pointsThreshold && calibratorPoints2.size() > pointsThreshold)
                    {
                        cv::Mat wtranslation = transformFromPoints(calibratorPoints2, calibratorPoints1, calibratorTimes2, calibratorTimes1);
                        wtranslation2 = wtranslation * wtranslation2;
                        cv::Mat R = (cv::Mat_<double>(3, 3) <<
                                                 wtranslation2.at<double>(0, 0), wtranslation2.at<double>(0, 1), wtranslation2.at<double>(0, 2),
                                                 wtranslation2.at<double>(1, 0), wtranslation2.at<double>(1, 1), wtranslation2.at<double>(1, 2),
                                                 wtranslation2.at<double>(2, 0), wtranslation2.at<double>(2, 1), wtranslation2.at<double>(2, 2));
                        wrotation2 =  mRot2Quat(R);
                        HMatrix H = { R.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(2, 0), 0,
                                      R.at<double>(0, 1), R.at<double>(1, 1), R.at<double>(2, 1), 0,
                                      R.at<double>(0, 2), R.at<double>(1, 2), R.at<double>(2, 2), 0,
                                      0,                  0,                  0,                  1 };
                        auto eulerAngles = Eul_FromHMatrix(H, EulOrdYXZr);

                        parameters->wrotation2 = wrotation2;
                        parameters->wtranslation2 = wtranslation2;
                        parameters->calibOffsetX2 = 100 * wtranslation2.at<double>(0, 3);
                        parameters->calibOffsetY2 = 100 * wtranslation2.at<double>(1, 3);
                        parameters->calibOffsetZ2 = 100 * wtranslation2.at<double>(2, 3);
                        parameters->calibOffsetA2 = -(180.0/M_PI) * eulerAngles.z;
                        parameters->calibOffsetB2 = -(180.0/M_PI) * eulerAngles.x;
                        parameters->calibOffsetC2 = (180.0/M_PI) * eulerAngles.y;
                        parameters->Save();

                        calibratorProjected1.clear();
                        calibratorProjected2.clear();
                        calibratorReprojected1.clear();
                        calibratorReprojected2.clear();
                        calibratorPoints1.clear();
                        calibratorPoints2.clear();
                        calibratorTimes1.clear();
                        calibratorTimes2.clear();
                        
                        pointsThreshold += 1000;
                    }
                }
            }

            if (calibratorProjected1.size() > 0)
            {
                for (int i = 0; i < calibratorProjected1.size(); ++i)
                {
                    if (i % 15 != 0)
                        continue;
                    const auto& position = calibratorProjected1[i];
                    cv::circle(drawImg, position, 5, cv::Scalar(0, 255, 255), 2, 8, 0);
                    cv::circle(drawImgMasked, position, 5, cv::Scalar(0, 255, 255), 2, 8, 0);
                }
            }

            if (calibratorReprojected1.size() > 0)
            {
                for (int i = 0; i < calibratorReprojected1.size(); ++i)
                {
                    if (i % 15 != 0)
                        continue;
                    const auto& position = calibratorReprojected1[i];
                    cv::circle(drawImg, position, 5, cv::Scalar(255, 0, 127), 2, 8, 0);
                    cv::circle(drawImgMasked, position, 5, cv::Scalar(255, 0, 127), 2, 8, 0);
                }
            }

            trackerStatus[i].boardFoundDriver = false;        //do we really need to do this? test later
        }

        //Then define your mask image
        cv::Mat mask = cv::Mat::zeros(gray.size(), gray.type());

        cv::Mat dstImage = cv::Mat::zeros(gray.size(), gray.type());

        int size = ((gray.rows > gray.cols) ? gray.rows : gray.cols) * parameters->searchWindow1;

        bool doMasking = false;

        //I assume you want to draw the circle at the center of your image, with a radius of 50
        for (int i = 0; i < trackerNum; i++)
        {
            if (trackerStatus[i].maskCenter.x <= 0 || trackerStatus[i].maskCenter.y <= 0 || trackerStatus[i].maskCenter.x >= image.cols || trackerStatus[i].maskCenter.y >= image.rows)
            {
                trackerStatus[i].boardFound = false;    //if detected tracker is out of view of the camera, we mark it as not found, as either the prediction is wrong or we wont see it anyway
                continue;
            }
            doMasking = true;
            if (circularWindow)
            {
                cv::circle(mask, trackerStatus[i].maskCenter, size, cv::Scalar(255, 0, 0), -1, 8, 0);
                cv::circle(drawImg, trackerStatus[i].maskCenter, size, cv::Scalar(255, 0, 0), 2, 8, 0);
                cv::circle(drawImgMasked, trackerStatus[i].maskCenter, size, cv::Scalar(255, 0, 0), 2, 8, 0);
            }
            else
            {
                rectangle(mask, cv::Point(trackerStatus[i].maskCenter.x - size, 0), cv::Point(trackerStatus[i].maskCenter.x + size, image.rows), cv::Scalar(255, 0, 0), -1);
                rectangle(drawImg, cv::Point(trackerStatus[i].maskCenter.x - size, 0), cv::Point(trackerStatus[i].maskCenter.x + size, image.rows), cv::Scalar(255, 0, 0), 3);
                rectangle(drawImgMasked, cv::Point(trackerStatus[i].maskCenter.x - size, 0), cv::Point(trackerStatus[i].maskCenter.x + size, image.rows), cv::Scalar(255, 0, 0), 3);
            }
        }

        //Now you can copy your source image to destination image with masking
        if (doMasking)
        {
            gray.copyTo(dstImage, mask);
            gray = dstImage;
        }

        //cv::imshow("test", image);

        april.detectMarkers(gray, &corners, &ids, &centers, trackers);
        std::chrono::milliseconds current_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        for (int i = 0; i < trackerNum; ++i) {

            trackerStatus[i].pose_valid = -1;

            //estimate the pose of current board

            try
            {
                trackerStatus[i].boardTvec /= calibScale;
                if (cv::aruco::estimatePoseBoard(corners, ids, trackers[i], parameters->camMat1, parameters->distCoeffs1, trackerStatus[i].boardRvec, trackerStatus[i].boardTvec, trackerStatus[i].boardFound && parameters->usePredictive) <= 0)
                {
                    for (int j = 0; j < 6; j++)
                    {
                        //push new values into previous values list end and remove the one on beggining
                        if (trackerStatus[i].prevLocValues[j].size() > 0)
                            trackerStatus[i].prevLocValues[j].erase(trackerStatus[i].prevLocValues[j].begin());
                    }
                    trackerStatus[i].boardFound = false;

                    continue;
                }
                else
                {
                    if ((tracker_pose_valid[i] != 0) || ((current_timestamp.count() - trackerStatus[i].last_update_timestamp.count()) <= 100.0))
                    {
                        trackerStatus[i].last_update_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
                    }

                }
            }
            catch (std::exception&)
            {
                wxString e = parameters->language.TRACKER_DETECTION_SOMETHINGWRONG;
                gui->CallAfter([e] ()
                               {
                               wxMessageDialog dial(NULL, e, wxT("Error"), wxOK | wxICON_ERROR);
                                dial.ShowModal();
                               cv::destroyWindow("out1");
                               });
                //apriltag_detector_destroy(td);
                mainThreadRunning = false;
                return;
            }
            trackerStatus[i].boardFound = true;

            trackerStatus[i].boardTvec *= calibScale;

            double posValues[6] = {
                trackerStatus[i].boardTvec[0],
                trackerStatus[i].boardTvec[1],
                trackerStatus[i].boardTvec[2],
                trackerStatus[i].boardRvec[0],
                trackerStatus[i].boardRvec[1],
                trackerStatus[i].boardRvec[2] };

            for (int j = 0; j < 6; j++)
            {
                //push new values into previous values list end and remove the one on beggining
                trackerStatus[i].prevLocValues[j].push_back(posValues[j]);
                if (trackerStatus[i].prevLocValues[j].size() > numOfPrevValues)
                {
                    trackerStatus[i].prevLocValues[j].erase(trackerStatus[i].prevLocValues[j].begin());
                }

                std::vector<double> valArray(trackerStatus[i].prevLocValues[j]);
                //sort(valArray.begin(), valArray.end());

                //posValues[j] = valArray[valArray.size() / 2];
                posValues[j] = valArray[valArray.size() - 1];

            }
            //save fitted values back to our variables
            trackerStatus[i].boardTvec[0] = posValues[0];
            trackerStatus[i].boardTvec[1] = posValues[1];
            trackerStatus[i].boardTvec[2] = posValues[2];
            trackerStatus[i].boardRvec[0] = posValues[3];
            trackerStatus[i].boardRvec[1] = posValues[4];
            trackerStatus[i].boardRvec[2] = posValues[5];

            cv::Mat rpos = cv::Mat_<double>(4, 1);

            //transform boards position based on our calibration data

            for (int x = 0; x < 3; x++)
            {
                rpos.at<double>(x, 0) = trackerStatus[i].boardTvec[x];
            }
            rpos.at<double>(3, 0) = 1;
            rpos = wtranslation1 * rpos;

            //convert rodriguez rotation to quaternion
            Quaternion<double> q = rodr2quat(trackerStatus[i].boardRvec[0], trackerStatus[i].boardRvec[1], trackerStatus[i].boardRvec[2]);

            cv::aruco::drawAxis(drawImg, parameters->camMat1, parameters->distCoeffs1, trackerStatus[i].boardRvec, trackerStatus[i].boardTvec, 0.05);
            cv::aruco::drawAxis(drawImgMasked, parameters->camMat1, parameters->distCoeffs1, trackerStatus[i].boardRvec, trackerStatus[i].boardTvec, 0.05);

            q = Quaternion<double>(0, 0, 1, 0) * (wrotation1 * q) * Quaternion<double>(0, 0, 1, 0);

            double a = -rpos.at<double>(0, 0);
            double b = rpos.at<double>(1, 0);
            double c = -rpos.at<double>(2, 0);

            double factor;
            factor = parameters->smoothingFactor;

            if (factor < 0)
                factor = 0;
            else if (factor >= 1)
                factor = 0.99;

            end = clock();
            double frameTime = double(end - last_frame_time) / double(CLOCKS_PER_SEC);

#if 1
            if (a < -1.5 || a > 1.5 || b < -0.5 || b > 2.5 || c < -1.5 || c > 1.5)
            {
                trackerStatus[i].boardFound = false;
            }
#endif
#if 1
            if ((current_timestamp.count() - trackerStatus[i].last_update_timestamp.count()) > 100.0)
                continue;
#endif

            //send all the values
            //frame time is how much time passed since frame was acquired.
            //connection->SendTracker(connection->connectedTrackers[i].DriverId, a, b, c, q.w, q.x, q.y, q.z, -frameTime - parameters->camLatency, factor);

            trackerStatus[i].pose_valid = 0;
            trackerStatus[i].a = a;
            trackerStatus[i].b = b;
            trackerStatus[i].c = c;
            trackerStatus[i].qw = q.w;
            trackerStatus[i].qx = q.x;
            trackerStatus[i].qy = q.y;
            trackerStatus[i].qz = q.z;
        }

        {
            cv::Mat mask = cv::Mat::zeros(image.size(), image.type());

            for (int i = 0; i < corners.size(); i++)
            {
                std::vector<cv::Point> points;
                for(int j = 0; j < corners[i].size(); j++){
                    points.push_back(corners[i].at(j));
                }
                cv::fillConvexPoly(mask, points, cv::Scalar(255, 255, 255), 8, 0);
            }

            drawImg.copyTo(drawImgMasked, mask);
        }


        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(drawImg, corners, ids);
            cv::aruco::drawDetectedMarkers(drawImgMasked, corners, ids);
        }

        end = clock();
        double frameTime = double(end - start) / double(CLOCKS_PER_SEC);

        if (!disableOut)
        {
            int cols, rows;
            if (image.cols < image.rows)
            {
                cols = image.cols * drawImgSize / image.rows;
                rows = drawImgSize;
            }
            else
            {
                cols = drawImgSize;
                rows = image.rows * drawImgSize / image.cols;
            }
            cv::Mat *outImg = new cv::Mat();
            if (privacyMode)
                cv::resize(drawImgMasked, *outImg, cv::Size(cols, rows));
            else
                cv::resize(drawImg, *outImg, cv::Size(cols, rows));
            cv::putText(*outImg, std::to_string(frameTime).substr(0, 5), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
            if (showTimeProfile)
            {
                april.drawTimeProfile(*outImg, cv::Point(10, 60));
            }            
            
            gui->CallAfter([outImg] ()
                           {
                           cv::imshow("out1", *outImg);
                           cv::waitKey(1);
                           delete(outImg);
                           });
        }
        //time of marker detection
    }
    gui->CallAfter([] ()
                   {
                   cv::destroyWindow("out1");
                   });
}

void Tracker::MainLoop2()
{

    int trackerNum = parameters->trackerNum;
    int numOfPrevValues = parameters->numOfPrevValues;

    //these variables are used to save detections of apriltags, so we dont define them every frame

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<cv::Point2f> centers;


    cv::Mat image, drawImg, ycc, gray, cr;

    cv::Mat  prevImg;


    //setup all variables that need to be stored for each tracker and initialize them
    std::vector<TrackerStatus> trackerStatus = std::vector<TrackerStatus>(trackerNum, TrackerStatus());
    for (int i = 0; i < trackerStatus.size(); i++)
    {
        trackerStatus[i].boardFound = false;
        trackerStatus[i].boardRvec = cv::Vec3d(0, 0, 0);
        trackerStatus[i].boardTvec = cv::Vec3d(0, 0, 0);
        trackerStatus[i].prevLocValues = std::vector<std::vector<double>>(7, std::vector<double>());
        trackerStatus[i].last_update_timestamp = std::chrono::milliseconds(0);
        trackerStatus[i].pose_valid = -1;
        trackerStatus[i].a = 0;
        trackerStatus[i].b = 0;
        trackerStatus[i].c = 0;
        trackerStatus[i].qw = 1;
        trackerStatus[i].qx = 0;
        trackerStatus[i].qy = 0;
        trackerStatus[i].qz = 0;
    }

    //previous values, used for moving median to remove any outliers.
    std::vector<double> prevLocValuesX;

    //the X axis, it is simply numbers 0-10 (or the amount of previous values we have)
    for (int j = 0; j < numOfPrevValues; j++)
    {
        prevLocValuesX.push_back(j);
    }


    AprilTagWrapper april{parameters};

    int framesSinceLastSeen = 0;
    int framesToCheckAll = 20;

    cv::Mat stationPos = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
    stationPos = wtranslation2 * stationPos;

    Quaternion<double> stationQ = Quaternion<double>(0, 0, 1, 0) * (wrotation2 * Quaternion<double>(1, 0, 0, 0));

    double a = -stationPos.at<double>(0, 0);
    double b = stationPos.at<double>(1, 0);
    double c = -stationPos.at<double>(2, 0);

    connection->SendStation(0, a, b, c, stationQ.w, stationQ.x, stationQ.y, stationQ.z);

    bool calibControllerPosActive = false;
    bool calibControllerAngleActive = false;
    clock_t calibControllerLastPress = clock();
    double calibControllerPosOffset[] = { 0,0,0 };
    double calibControllerAngleOffset[] = { 0,0,0 };

    //double tempScale = 1;

    std::vector<cv::Ptr<cv::aruco::Board>> trackers;
    std::vector<std::vector<int>> boardIds;
    std::vector<std::vector < std::vector<cv::Point3f >>> boardCorners;
    std::vector<cv::Point3f > boardCenters;

    {
        std::lock_guard<std::mutex> lock(calibratorMutex);
        frameCount2 = 0;
        calibratorProjected2.clear();
        calibratorReprojected2.clear();
        calibratorPoints2.clear();
    }

    for (int i = 0; i < this->trackers.size(); i++)
    {
        boardCorners.push_back(this->trackers[i]->objPoints);
        boardIds.push_back(this->trackers[i]->ids);
        boardCenters.push_back(cv::Point3f(0, 0, 0));

        int numOfCorners = 0;
        for (int j = 0; j < boardCorners[i].size(); j++)
        {
            for (int k = 0; k < boardCorners[i][j].size(); k++)
            {
                boardCenters[i].x += boardCorners[i][j][k].x;
                boardCenters[i].y += boardCorners[i][j][k].y;
                boardCenters[i].z += boardCorners[i][j][k].z;
                numOfCorners++;
            }
        }
        boardCenters[i] /= numOfCorners;
    }

    if (parameters->trackerCalibCenters)
    {
        for (int i = 0; i < this->trackers.size(); i++)
        {
            for (int j = 0; j < boardCorners[i].size(); j++)
            {
                for (int k = 0; k < boardCorners[i][j].size(); k++)
                {
                    boardCorners[i][j][k] -= boardCenters[i];
                    boardCenters[i] = cv::Point3f(0, 0, 0);
                }
            }

            cv::Ptr<cv::aruco::Board> arBoard = cv::aruco::Board::create(boardCorners[i], cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), boardIds[i]);
            trackers.push_back(arBoard);
        }
    }
    else
    {
        trackers = this->trackers;
    }

    gui->CallAfter([] ()
                   {
                   cv::namedWindow("out2");
                   });

    while (mainThreadRunning && cameraRunning2)
    {

        CopyFreshCameraImageTo2(image);

        drawImg = image;
        cv::Mat drawImgMasked = cv::Mat::zeros(drawImg.size(), drawImg.type());
        april.convertToSingleChannel(image, gray);

        clock_t start, end;
        //for timing our detection
        start = clock();

        bool circularWindow = parameters->circularWindow;

        //last is if pose is valid: 0 is valid, 1 is late (hasnt been updated for more than 0.2 secs), -1 means invalid and is only zeros
        std::vector<int> tracker_pose_valid = std::vector<int>(trackerNum, -1);

        for (int i = 0; i < trackerNum; i++)
        {
            if (!trackerStatus[i].boardFound)
            {
                framesSinceLastSeen++;
                if (framesSinceLastSeen > framesToCheckAll)
                    circularWindow = false;
                break;
            }
        }
        if (!circularWindow)
            framesSinceLastSeen = 0;

        for (int i = 0; i < trackerNum; i++)
        {

            double frameTime = double(clock() - last_frame_time) / double(CLOCKS_PER_SEC);

            double a; double b; double c;
            double qw; double qx; double qy; double qz;

            a = trackerStatus[i].a;
            b = trackerStatus[i].b;
            c = trackerStatus[i].c;
            qw = trackerStatus[i].qw;
            qx = trackerStatus[i].qx;
            qy = trackerStatus[i].qy;
            qz = trackerStatus[i].qz;
            tracker_pose_valid[i] = trackerStatus[i].pose_valid;

            cv::Vec3d rvec2;
            cv::Vec3d tvec2;

            tvec2 = trackerStatus[i].boardTvec;
            rvec2 = trackerStatus[i].boardRvec;

            std::vector<cv::Point3f> offsetin2, offsetout2;
            offsetin2.push_back(boardCenters[i]);
            offsetFromBoardToCameraSpace(offsetin2, rvec2, tvec2, &offsetout2);

            std::vector<cv::Point3d> point2;
            point2.push_back(cv::Point3d(trackerStatus[i].boardTvec));
            point2.push_back(cv::Point3d(trackerStatus[i].boardTvec) + cv::Point3d(offsetout2[0]));

            std::vector<cv::Point2d> projected1, projected2;
            cv::Vec3d identity_rvec, identity_tvec;

            cv::projectPoints(point2, identity_rvec, identity_tvec, parameters->camMat2, parameters->distCoeffs2, projected2);

            if (tracker_pose_valid[i] == 0)
            {
                cv::Vec3d rvec1;
                cv::Vec3d tvec1;

                //transform boards position based on our calibration data
                cv::Mat rpos = (cv::Mat_<double>(4, 1) << -a, b, -c, 1);

                //transform boards position based on our calibration data

                rpos.at<double>(3, 0) = 1;
                rpos = wtranslation1.inv() * rpos;

                Quaternion<double> q = Quaternion<double>(qw, qx, qy, qz);
                q = q.UnitQuaternion();

                //q = Quaternion<double>(0, 0, 1, 0) * (wrotation1 * q) * Quaternion<double>(0, 0, 1, 0);
                q = wrotation1.inverse() * Quaternion<double>(0, 0, 1, 0).inverse() * q * Quaternion<double>(0, 0, 1, 0).inverse();

                rvec1 = quat2rodr(q.w, q.x, q.y, q.z);
                tvec1[0] = rpos.at<double>(0, 0);
                tvec1[1] = rpos.at<double>(1, 0);
                tvec1[2] = rpos.at<double>(2, 0);

                std::vector<cv::Point3f> offsetin1, offsetout1;
                offsetin1.push_back(boardCenters[i]);
                offsetFromBoardToCameraSpace(offsetin1, rvec1, tvec1, &offsetout1);

                std::vector<cv::Point3d> point1;
                point1.push_back(cv::Point3d(tvec1));
                point1.push_back(cv::Point3d(tvec1) + cv::Point3d(offsetout1[0]));

                cv::projectPoints(point1, identity_rvec, identity_tvec, parameters->camMat1, parameters->distCoeffs1, projected1);
            }

            cv::circle(drawImg, projected2[0], 5, cv::Scalar(0, 0, 255), 2, 8, 0);
            cv::circle(drawImgMasked, projected2[0], 5, cv::Scalar(0, 0, 255), 2, 8, 0);
            cv::circle(drawImg, projected2[1], 5, cv::Scalar(255, 0, 255), 2, 8, 0);
            cv::circle(drawImgMasked, projected2[1], 5, cv::Scalar(255, 0, 255), 2, 8, 0);

            std::lock_guard<std::mutex> lock(calibratorMutex);

            if (trackerStatus[i].boardFound)
            {
                trackerStatus[i].maskCenter = projected2[1];
                if (true) //(frameCount1++ % 5 == 0)
                {
                    calibratorProjected2.push_back(projected2[1]);
                    if (tracker_pose_valid[i] == 0)
                    {
                        calibratorReprojected1.push_back(projected1[1]);
                    }

                    cv::Mat rpos = (cv::Mat_<double>(4, 1) << point2[1].x, point2[1].y, point2[1].z, 1);
                    rpos = wtranslation2 * rpos;

                    calibratorPoints2.push_back(cv::Point3d(rpos.at<double>(0,0), rpos.at<double>(1,0), rpos.at<double>(2,0)));
                    calibratorTimes2.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

                    if (calibratorPoints1.size() > pointsThreshold && calibratorPoints2.size() > pointsThreshold)
                    {
                        cv::Mat wtranslation = transformFromPoints(calibratorPoints2, calibratorPoints1, calibratorTimes2, calibratorTimes1);
                        wtranslation2 = wtranslation * wtranslation2;
                        cv::Mat R = (cv::Mat_<double>(3, 3) <<
                                                 wtranslation2.at<double>(0, 0), wtranslation2.at<double>(0, 1), wtranslation2.at<double>(0, 2),
                                                 wtranslation2.at<double>(1, 0), wtranslation2.at<double>(1, 1), wtranslation2.at<double>(1, 2),
                                                 wtranslation2.at<double>(2, 0), wtranslation2.at<double>(2, 1), wtranslation2.at<double>(2, 2));
                        wrotation2 =  mRot2Quat(R);
                        HMatrix H = { R.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(2, 0), 0,
                                      R.at<double>(0, 1), R.at<double>(1, 1), R.at<double>(2, 1), 0,
                                      R.at<double>(0, 2), R.at<double>(1, 2), R.at<double>(2, 2), 0,
                                      0,                  0,                  0,                  1 };
                        auto eulerAngles = Eul_FromHMatrix(H, EulOrdYXZr);

                        parameters->wrotation2 = wrotation2;
                        parameters->wtranslation2 = wtranslation2;
                        parameters->calibOffsetX2 = 100 * wtranslation2.at<double>(0, 3);
                        parameters->calibOffsetY2 = 100 * wtranslation2.at<double>(1, 3);
                        parameters->calibOffsetZ2 = 100 * wtranslation2.at<double>(2, 3);
                        parameters->calibOffsetA2 = -(180.0/M_PI) * eulerAngles.z;
                        parameters->calibOffsetB2 = -(180.0/M_PI) * eulerAngles.x;
                        parameters->calibOffsetC2 = (180.0/M_PI) * eulerAngles.y;
                        parameters->Save();

                        calibratorProjected1.clear();
                        calibratorProjected2.clear();
                        calibratorReprojected1.clear();
                        calibratorReprojected2.clear();
                        calibratorPoints1.clear();
                        calibratorPoints2.clear();
                        calibratorTimes1.clear();
                        calibratorTimes2.clear();

                        pointsThreshold += 1000;
                    }
                }
            }

            if (calibratorProjected2.size() > 0)
            {
                for (int i = 0; i < calibratorProjected2.size(); ++i)
                {
                    if (i % 15 != 0)
                        continue;
                    const auto& position = calibratorProjected2[i];
                    cv::circle(drawImg, position, 5, cv::Scalar(0, 255, 255), 2, 8, 0);
                    cv::circle(drawImgMasked, position, 5, cv::Scalar(0, 255, 255), 2, 8, 0);
                }
            }

            if (calibratorReprojected2.size() > 0)
            {
                for (int i = 0; i < calibratorReprojected2.size(); ++i)
                {
                    if (i % 15 != 0)
                        continue;
                    const auto& position = calibratorReprojected2[i];
                    cv::circle(drawImg, position, 5, cv::Scalar(255, 0, 127), 2, 8, 0);
                    cv::circle(drawImgMasked, position, 5, cv::Scalar(255, 0, 127), 2, 8, 0);
                }
            }

            trackerStatus[i].boardFoundDriver = false;        //do we really need to do this? test later
        }

        //Then define your mask image
        cv::Mat mask = cv::Mat::zeros(gray.size(), gray.type());

        cv::Mat dstImage = cv::Mat::zeros(gray.size(), gray.type());

        int size = ((gray.rows > gray.cols) ? gray.rows : gray.cols) * parameters->searchWindow2;

        bool doMasking = false;

        //I assume you want to draw the circle at the center of your image, with a radius of 50
        for (int i = 0; i < trackerNum; i++)
        {
            if (trackerStatus[i].maskCenter.x <= 0 || trackerStatus[i].maskCenter.y <= 0 || trackerStatus[i].maskCenter.x >= image.cols || trackerStatus[i].maskCenter.y >= image.rows)
            {
                trackerStatus[i].boardFound = false;    //if detected tracker is out of view of the camera, we mark it as not found, as either the prediction is wrong or we wont see it anyway
                continue;
            }
            doMasking = true;
            if (circularWindow)
            {
                cv::circle(mask, trackerStatus[i].maskCenter, size, cv::Scalar(255, 0, 0), -1, 8, 0);
                cv::circle(drawImg, trackerStatus[i].maskCenter, size, cv::Scalar(255, 0, 0), 2, 8, 0);
                cv::circle(drawImgMasked, trackerStatus[i].maskCenter, size, cv::Scalar(255, 0, 0), 2, 8, 0);
            }
            else
            {
                rectangle(mask, cv::Point(trackerStatus[i].maskCenter.x - size, 0), cv::Point(trackerStatus[i].maskCenter.x + size, image.rows), cv::Scalar(255, 0, 0), -1);
                rectangle(drawImg, cv::Point(trackerStatus[i].maskCenter.x - size, 0), cv::Point(trackerStatus[i].maskCenter.x + size, image.rows), cv::Scalar(255, 0, 0), 3);
                rectangle(drawImgMasked, cv::Point(trackerStatus[i].maskCenter.x - size, 0), cv::Point(trackerStatus[i].maskCenter.x + size, image.rows), cv::Scalar(255, 0, 0), 3);
            }
        }

        //Now you can copy your source image to destination image with masking
        if (doMasking)
        {
            gray.copyTo(dstImage, mask);
            gray = dstImage;
        }

        //cv::imshow("test", image);

        april.detectMarkers(gray, &corners, &ids, &centers, trackers);
        std::chrono::milliseconds current_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        for (int i = 0; i < trackerNum; ++i) {

            trackerStatus[i].pose_valid = -1;

            //estimate the pose of current board

            try
            {
                trackerStatus[i].boardTvec /= calibScale;
                if (cv::aruco::estimatePoseBoard(corners, ids, trackers[i], parameters->camMat2, parameters->distCoeffs2, trackerStatus[i].boardRvec, trackerStatus[i].boardTvec, trackerStatus[i].boardFound && parameters->usePredictive) <= 0)
                {
                    for (int j = 0; j < 6; j++)
                    {
                        //push new values into previous values list end and remove the one on beggining
                        if (trackerStatus[i].prevLocValues[j].size() > 0)
                            trackerStatus[i].prevLocValues[j].erase(trackerStatus[i].prevLocValues[j].begin());
                    }
                    trackerStatus[i].boardFound = false;

                    continue;
                }
                else
                {
                    if ((tracker_pose_valid[i] != 0) || ((current_timestamp.count() - trackerStatus[i].last_update_timestamp.count()) <= 100.0))
                    {
                        trackerStatus[i].last_update_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
                    }

                }
            }
            catch (std::exception&)
            {
                wxString e = parameters->language.TRACKER_DETECTION_SOMETHINGWRONG;
                gui->CallAfter([e] ()
                               {
                               wxMessageDialog dial(NULL, e, wxT("Error"), wxOK | wxICON_ERROR);
                                dial.ShowModal();
                               cv::destroyWindow("out2");
                               });
                //apriltag_detector_destroy(td);
                mainThreadRunning = false;
                return;
            }
            trackerStatus[i].boardFound = true;

            trackerStatus[i].boardTvec *= calibScale;

            double posValues[6] = {
                trackerStatus[i].boardTvec[0],
                trackerStatus[i].boardTvec[1],
                trackerStatus[i].boardTvec[2],
                trackerStatus[i].boardRvec[0],
                trackerStatus[i].boardRvec[1],
                trackerStatus[i].boardRvec[2] };

            for (int j = 0; j < 6; j++)
            {
                //push new values into previous values list end and remove the one on beggining
                trackerStatus[i].prevLocValues[j].push_back(posValues[j]);
                if (trackerStatus[i].prevLocValues[j].size() > numOfPrevValues)
                {
                    trackerStatus[i].prevLocValues[j].erase(trackerStatus[i].prevLocValues[j].begin());
                }

                std::vector<double> valArray(trackerStatus[i].prevLocValues[j]);
                //sort(valArray.begin(), valArray.end());

                //posValues[j] = valArray[valArray.size() / 2];
                posValues[j] = valArray[valArray.size() - 1];

            }
            //save fitted values back to our variables
            trackerStatus[i].boardTvec[0] = posValues[0];
            trackerStatus[i].boardTvec[1] = posValues[1];
            trackerStatus[i].boardTvec[2] = posValues[2];
            trackerStatus[i].boardRvec[0] = posValues[3];
            trackerStatus[i].boardRvec[1] = posValues[4];
            trackerStatus[i].boardRvec[2] = posValues[5];

            cv::Mat rpos = cv::Mat_<double>(4, 1);

            //transform boards position based on our calibration data

            for (int x = 0; x < 3; x++)
            {
                rpos.at<double>(x, 0) = trackerStatus[i].boardTvec[x];
            }
            rpos.at<double>(3, 0) = 1;
            rpos = wtranslation2 * rpos;

            //convert rodriguez rotation to quaternion
            Quaternion<double> q = rodr2quat(trackerStatus[i].boardRvec[0], trackerStatus[i].boardRvec[1], trackerStatus[i].boardRvec[2]);

            cv::aruco::drawAxis(drawImg, parameters->camMat2, parameters->distCoeffs2, trackerStatus[i].boardRvec, trackerStatus[i].boardTvec, 0.05);
            cv::aruco::drawAxis(drawImgMasked, parameters->camMat2, parameters->distCoeffs2, trackerStatus[i].boardRvec, trackerStatus[i].boardTvec, 0.05);

            q = Quaternion<double>(0, 0, 1, 0) * (wrotation2 * q) * Quaternion<double>(0, 0, 1, 0);

            double a = -rpos.at<double>(0, 0);
            double b = rpos.at<double>(1, 0);
            double c = -rpos.at<double>(2, 0);

            double factor;
            factor = parameters->smoothingFactor;

            if (factor < 0)
                factor = 0;
            else if (factor >= 1)
                factor = 0.99;

            end = clock();
            double frameTime = double(end - last_frame_time) / double(CLOCKS_PER_SEC);

#if 0
            if (a < -1.5 || a > 1.5 || b < -0.5 || b > 2.5 || c < -1.5 || c > 1.5)
            {
                trackerStatus[i].boardFound = false;
            }
#endif
#if 1
            if ((current_timestamp.count() - trackerStatus[i].last_update_timestamp.count()) > 100.0)
                continue;
#endif

            //send all the values
            //frame time is how much time passed since frame was acquired.
            //connection->SendTracker(connection->connectedTrackers[i].DriverId, a, b, c, q.w, q.x, q.y, q.z, -frameTime - parameters->camLatency, factor);

            trackerStatus[i].pose_valid = 0;
            trackerStatus[i].a = a;
            trackerStatus[i].b = b;
            trackerStatus[i].c = c;
            trackerStatus[i].qw = q.w;
            trackerStatus[i].qx = q.x;
            trackerStatus[i].qy = q.y;
            trackerStatus[i].qz = q.z;
        }

        {
            cv::Mat mask = cv::Mat::zeros(image.size(), image.type());

            for (int i = 0; i < corners.size(); i++)
            {
                std::vector<cv::Point> points;
                for(int j = 0; j < corners[i].size(); j++){
                    points.push_back(corners[i].at(j));
                }
                cv::fillConvexPoly(mask, points, cv::Scalar(255, 255, 255), 8, 0);
            }

            drawImg.copyTo(drawImgMasked, mask);
        }


        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(drawImg, corners, ids);
            cv::aruco::drawDetectedMarkers(drawImgMasked, corners, ids);
        }

        end = clock();
        double frameTime = double(end - start) / double(CLOCKS_PER_SEC);

        if (!disableOut)
        {
            int cols, rows;
            if (image.cols < image.rows)
            {
                cols = image.cols * drawImgSize / image.rows;
                rows = drawImgSize;
            }
            else
            {
                cols = drawImgSize;
                rows = image.rows * drawImgSize / image.cols;
            }
            cv::Mat *outImg = new cv::Mat();
            if (privacyMode)
                cv::resize(drawImgMasked, *outImg, cv::Size(cols, rows));
            else
                cv::resize(drawImg, *outImg, cv::Size(cols, rows));
            cv::putText(*outImg, std::to_string(frameTime).substr(0, 5), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
            if (showTimeProfile)
            {
                april.drawTimeProfile(*outImg, cv::Point(10, 60));
            }            
            
            gui->CallAfter([outImg] ()
                           {
                           cv::imshow("out2", *outImg);
                           cv::waitKey(1);
                           delete(outImg);
                           });
        }
        //time of marker detection
    }
    gui->CallAfter([] ()
                   {
                   cv::destroyWindow("out2");
                   });
}
