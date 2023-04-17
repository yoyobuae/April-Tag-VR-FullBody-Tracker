#include <iostream>
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
    parameters->camMat.copyTo(cameraMatrix);
    parameters->distCoeffs.copyTo(distCoeffs);
    parameters->stdDeviationsIntrinsics.copyTo(stdDeviationsIntrinsics);
    std::vector<double> perViewErrors = parameters->perViewErrors;
    std::vector<std::vector<cv::Point2f>> allCharucoCorners = parameters->allCharucoCorners;
    std::vector<std::vector<int>> allCharucoIds = parameters->allCharucoIds;

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
    if (!parameters->wtranslation.empty())
    {
        wtranslation = parameters->wtranslation;
        wrotation = parameters->wrotation;
    }
    calibScale = parameters->calibScale;
}

void Tracker::StartCamera(std::string id, int apiPreference)
{
    if (cameraRunning)
    {
        cameraRunning = false;
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
            if(parameters->camWidth != 0)
                ss << ",width=" << parameters->camWidth;
            if (parameters->camHeight != 0)
                ss << ",height=" << parameters->camHeight;
            ss << ",framerate=" << parameters->camFps << "/1";
            ss << " ! jpegdec ! video/x-raw,format=I420 ! videoconvert ! appsink";
            cap = cv::VideoCapture(ss.str(), apiPreference);
        }
        else
#endif
        {
            cap = cv::VideoCapture(i, apiPreference);
        }

    }
    else
    {			//if address is longer, we try to open it as an ip address
        cap = cv::VideoCapture(id, apiPreference);
    }

    if (!cap.isOpened())
    {
        wxMessageDialog dial(NULL,
            parameters->language.TRACKER_CAMERA_START_ERROR, wxT("Error"), wxOK | wxICON_ERROR);
        dial.ShowModal();
        return;
    }
    //Sleep(1000);

    double codec = 0x47504A4D; //code by FPaul. Should use MJPEG codec to enable fast framerates.
    cap.set(cv::CAP_PROP_FOURCC, codec);

    // On Linux and when GStreamer backend is used we already setup the camera pixel format,
    // width, height and FPS above when the GStreamer pipeline was created.
#if defined(__LINUX__)
    if ((apiPreference != cv::CAP_ANY) && (apiPreference != cv::CAP_GSTREAMER))
#endif
    {
        //cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('m', 'j', 'p', 'g'));
        //cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        if(parameters->camWidth != 0)
            cap.set(cv::CAP_PROP_FRAME_WIDTH, parameters->camWidth);
        if (parameters->camHeight != 0)
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, parameters->camHeight);
        cap.set(cv::CAP_PROP_FPS, parameters->camFps);
    }
    if(parameters->cameraSettings)
        cap.set(cv::CAP_PROP_SETTINGS, 1);
    if (parameters->settingsParameters)
    {
        cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
        cap.set(cv::CAP_PROP_AUTO_EXPOSURE, parameters->cameraAutoexposure);
        cap.set(cv::CAP_PROP_EXPOSURE, parameters->cameraExposure);
        cap.set(cv::CAP_PROP_GAIN, parameters->cameraGain);
    }


    cameraRunning = true;
    cameraThread = std::thread(&Tracker::CameraLoop, this);
    cameraThread.detach();
}

void Tracker::CameraLoop()
{
    bool previewShown = false;
    if (parameters->rotateCl && parameters->rotateCounterCl)
    {
        rotate = true;
        rotateFlag = cv::ROTATE_180;
    }
    else if (parameters->rotateCl)
    {
        rotate = true;
        rotateFlag = cv::ROTATE_90_CLOCKWISE;
    }
    else if (parameters->rotateCounterCl)
    {
        rotate = true;
        rotateFlag = cv::ROTATE_90_COUNTERCLOCKWISE;
    }
    cv::Mat img;
    cv::Mat drawImg;
    double fps = 0;
    clock_t last_frame_time = clock();
    bool frame_visible = false;
    int cols, rows;

    while (cameraRunning)
    {
        if (!cap.read(img))
        {
            gui->CallAfter([parameters=parameters] ()
                           {
                           wxMessageDialog dial(NULL,
                               parameters->language.TRACKER_CAMERA_ERROR, wxT("Error"), wxOK | wxICON_ERROR);
                           dial.ShowModal();
                           });
            cameraRunning = false;
            break;
        }
        clock_t curtime = clock();
        fps = 0.95*fps + 0.05/(double(curtime - last_frame_time) / double(CLOCKS_PER_SEC));
        last_frame_time = curtime;        
        std::string resolution = std::to_string(img.cols) + "x" + std::to_string(img.rows);
        if (previewCamera || previewCameraCalibration)
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
            cv::line(drawImg, cv::Point(img.cols/2, 0), cv::Point(img.cols/2, img.rows), cv::Scalar(0, 0, 255));
            cv::line(drawImg, cv::Point(0, img.rows/2), cv::Point(img.cols, img.rows/2), cv::Scalar(0, 0, 255));
            if (previewCameraCalibration)
            {
                cv::Mat *outImg = new cv::Mat();
                previewCalibration(drawImg, parameters);
                cv::resize(drawImg, *outImg, cv::Size(cols, rows));
                gui->CallAfter([outImg] ()
                               {
                               cv::imshow("Preview", *outImg);
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
                               cv::imshow("Preview", *outImg);
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
                               cv::destroyWindow("Preview");
                               });
                previewShown = false;
        }
        {
            std::unique_lock<std::mutex> lock(cameraFrameMutex);
            // Swap avoids copying the pixel buffer. It only swaps pointers and metadata.
            // The pixel buffer from cameraImage can be reused if the size and format matches.
            cv::swap(img, cameraFrame.image);
            if (img.size() != cameraFrame.image.size() || img.flags != cameraFrame.image.flags)
            {
                img.release();
            }
            cameraFrame.ready = true;
            cameraFrame.captureTime = last_frame_time;
            cameraFrame.swapTime = clock();
        }
        cameraFrameCondVar.notify_one();

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
    cap.release();
}

void Tracker::CopyFreshCameraImageTo(FrameData& frame)
{
    {
        std::unique_lock<std::mutex> lock(cameraFrameMutex);
        if (!cameraFrame.ready)
        {
            cameraFrameCondVar.wait(lock, [&]{ return cameraFrame.ready; });
        }
        {
            cameraFrame.ready = false;
            frame.ready = true;
            // Swap metadata and pointers to pixel buffers.
            cv::swap(frame.image, cameraFrame.image);
            // We don't want to overwrite shared data so release the image unless we are the only user of it.
            if (!(cameraFrame.image.u && cameraFrame.image.u->refcount == 1))
            {
                cameraFrame.image.release();
            }
            frame.captureTime = cameraFrame.captureTime;
            frame.swapTime = cameraFrame.swapTime;
            frame.copyFreshTime = clock();
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
    if (!cameraRunning)
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
        mainThread = std::thread(&Tracker::CalibrateCameraCharuco, this);
    }
    else
    {
        mainThread = std::thread(&Tracker::CalibrateCamera, this);
    }
    mainThread.detach();
}

void Tracker::CalibrateCameraCharuco()
{
    //function to calibrate our camera

    FrameData frame;
    cv::Mat &image = frame.image;
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
    while(mainThreadRunning && cameraRunning)
    {
        CopyFreshCameraImageTo(frame);
        if (rotate)
        {
            cv::rotate(image, image, rotateFlag);
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
            parameters->camMat = cameraMatrix;
            parameters->distCoeffs = distCoeffs;
            parameters->stdDeviationsIntrinsics = stdDeviationsIntrinsics;
            parameters->perViewErrors = perViewErrors;
            parameters->allCharucoCorners = allCharucoCorners;
            parameters->allCharucoIds = allCharucoIds;
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

    FrameData frame;
    cv::Mat &image = frame.image;

    int i = 0;
    int framesSinceLast = -100;

    int picNum = parameters->cameraCalibSamples;

    while (i < picNum)
    {
        if (!mainThreadRunning || !cameraRunning)
        {
            gui->CallAfter([] ()
                           {
                           cv::destroyAllWindows();
                           });
            return;
        }
        CopyFreshCameraImageTo(frame);
        if (rotate)
        {
            cv::rotate(image, image, rotateFlag);
        }
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

    parameters->camMat = cameraMatrix;
    parameters->distCoeffs = distCoeffs;
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
    if (!cameraRunning)
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
    if (parameters->camMat.empty())
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
    mainThread = std::thread(&Tracker::CalibrateTracker, this);
    mainThread.detach();


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
    if (!cameraRunning)
    {
        wxMessageDialog dial(NULL,
            parameters->language.TRACKER_CAMERA_NOTRUNNING, wxT("Error"), wxOK | wxICON_ERROR);
        dial.ShowModal();
        mainThreadRunning = false;
        return;
    }
    if (parameters->camMat.empty())
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
    if (connection->status != connection->CONNECTED)
    {
        wxMessageDialog dial(NULL,
            parameters->language.TRACKER_STEAMVR_NOTCONNECTED, wxT("Error"), wxOK | wxICON_ERROR);
        dial.ShowModal();
        mainThreadRunning = false;
        return;
    }
    mainThreadRunning = true;
    mainThread = std::thread(&Tracker::MainLoop, this);
    mainThread.detach();
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
    FrameData frame;
    cv::Mat &image = frame.image;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    std::vector<int> idsList;
    std::vector<std::vector < std::vector<cv::Point3f >>> cornersList;

    trackers.clear();

    while (cameraRunning && mainThreadRunning)
    {
        CopyFreshCameraImageTo(frame);
        if (rotate)
        {
            cv::rotate(image, image, rotateFlag);
        }

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
        cv::aruco::estimatePoseSingleMarkers(corners, markerSize, parameters->camMat, parameters->distCoeffs, rvecs, tvecs);
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
                if (cv::aruco::estimatePoseBoard(corners, ids, arBoard, parameters->camMat, parameters->distCoeffs, boardRvec[i], boardTvec[i], false) > 0)         //try to estimate current trackers pose
                {
                    cv::aruco::drawAxis(image, parameters->camMat, parameters->distCoeffs, boardRvec[i], boardTvec[i], 0.1f);       //if found, draw axis and mark it found
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

const char *cvTypeToStr(int type)
{
    switch(type) {
    case CV_8UC1: return "CV_8UC1";
    case CV_8UC2: return "CV_8UC2";
    case CV_8UC3: return "CV_8UC3";
    case CV_8UC4: return "CV_8UC4";
    case CV_8SC1: return "CV_8SC1";
    case CV_8SC2: return "CV_8SC2";
    case CV_8SC3: return "CV_8SC3";
    case CV_8SC4: return "CV_8SC4";
    case CV_16UC1: return "CV_16UC1";
    case CV_16UC2: return "CV_16UC2";
    case CV_16UC3: return "CV_16UC3";
    case CV_16UC4: return "CV_16UC4";
    case CV_16SC1: return "CV_16SC1";
    case CV_16SC2: return "CV_16SC2";
    case CV_16SC3: return "CV_16SC3";
    case CV_16SC4: return "CV_16SC4";
    case CV_32SC1: return "CV_32SC1";
    case CV_32SC2: return "CV_32SC2";
    case CV_32SC3: return "CV_32SC3";
    case CV_32SC4: return "CV_32SC4";
    case CV_32FC1: return "CV_32FC1";
    case CV_32FC2: return "CV_32FC2";
    case CV_32FC3: return "CV_32FC3";
    case CV_32FC4: return "CV_32FC4";
    case CV_64FC1: return "CV_64FC1";
    case CV_64FC2: return "CV_64FC2";
    case CV_64FC3: return "CV_64FC3";
    case CV_64FC4: return "CV_64FC4";
    case CV_16FC1: return "CV_16FC1";
    case CV_16FC2: return "CV_16FC2";
    case CV_16FC3: return "CV_16FC3";
    case CV_16FC4: return "CV_16FC4";
    default: return "(unknown)";
    }
}


void Tracker::MainLoop()
{

    int trackerNum = connection->connectedTrackers.size();
    int markersPerTracker = parameters->markersPerTracker;
    int numOfPrevValues = parameters->numOfPrevValues;

    //these variables are used to save detections of apriltags, so we dont define them every frame

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<cv::Point2f> centers;


    FrameData frame;
    cv::Mat &image = frame.image;
    cv::Mat drawImg, ycc, gray, cr;

    cv::Mat  prevImg;

    cv::Mat matchTemplateResult;
    bool didMatchTemplate = false;


    //setup all variables that need to be stored for each tracker and initialize them
    std::vector<TrackerStatus> trackerStatus = std::vector<TrackerStatus>(trackerNum, TrackerStatus());
    for (int i = 0; i < trackerStatus.size(); i++)
    {
        trackerStatus[i].boardFound = false;
        trackerStatus[i].doImageMatching = false;
        trackerStatus[i].boardRvec = cv::Vec3d(0, 0, 0);
        trackerStatus[i].boardTvec = cv::Vec3d(0, 0, 0);
        trackerStatus[i].prevLocValues = std::vector<std::vector<double>>(7, std::vector<double>());
        trackerStatus[i].last_update_timestamp = std::chrono::milliseconds(0);
        trackerStatus[i].searchSize = (int)(parameters->searchWindow*parameters->camMat.at<double>(0,0));
        trackerStatus[i].pose_delta_index = 0;
        for (int j = 0; j < 10; j++)
        {
            trackerStatus[i].pose_delta_history[j].a = 0.0;
            trackerStatus[i].pose_delta_history[j].b = 0.0;
            trackerStatus[i].pose_delta_history[j].c = 0.0;
            trackerStatus[i].pose_delta_history[j].qw = 0.0;
            trackerStatus[i].pose_delta_history[j].qx = 0.0;
            trackerStatus[i].pose_delta_history[j].qy = 0.0;
            trackerStatus[i].pose_delta_history[j].qz = 0.0;
        }
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
    stationPos = wtranslation * stationPos;

    Quaternion<double> stationQ = Quaternion<double>(0, 0, 1, 0) * (wrotation * Quaternion<double>(1, 0, 0, 0));

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
                   cv::namedWindow("out");
                   });

    while (mainThreadRunning && cameraRunning)
    {

        CopyFreshCameraImageTo(frame);

        int rotated_cols = image.cols;
        int rotated_rows = image.rows;
        switch (rotateFlag) {
        case cv::ROTATE_180:
            // Do nothing
            break;
        case cv::ROTATE_90_CLOCKWISE:
        case cv::ROTATE_90_COUNTERCLOCKWISE:
            rotated_cols = image.rows;
            rotated_rows = image.cols;
            break;
        }

        //last is if pose is valid: 0 is valid, 1 is late (hasnt been updated for more than 0.2 secs), -1 means invalid and is only zeros
        for (int i = 0; i < trackerNum; i++)
        {
            trackerStatus[i].pose_valid = -1;
        }

        {
            double frameTime = double(clock() - frame.captureTime) / double(CLOCKS_PER_SEC);

            std::stringstream ss;

            for (int i = 0; i < trackerNum; i++)
                ss << "gettrackerpose " + std::to_string(i) + " " + std::to_string(frameTime + parameters->camLatency) << std::endl;

            std::istringstream ret = connection->Send(ss.str());

            std::string word;

            for (int i = 0; i < trackerNum; i++)
            {
                ret >> word;
                if (word != "trackerpose")
                {
                    continue;
                }

                TrackerPose pose_from_driver;

                //read to our variables
                ret >> trackerStatus[i].idx;
                ret >> pose_from_driver.a;
                ret >> pose_from_driver.b;
                ret >> pose_from_driver.c;
                ret >> pose_from_driver.qw;
                ret >> pose_from_driver.qx;
                ret >> pose_from_driver.qy;
                ret >> pose_from_driver.qz;
                ret >> trackerStatus[i].pose_valid;


                TrackerPose pose_to_store = pose_from_driver - trackerStatus[i].pose_delta_average;

                trackerStatus[i].a = pose_to_store.a;
                trackerStatus[i].b = pose_to_store.b;
                trackerStatus[i].c = pose_to_store.c;
                trackerStatus[i].qw = pose_to_store.qw;
                trackerStatus[i].qx = pose_to_store.qx;
                trackerStatus[i].qy = pose_to_store.qy;
                trackerStatus[i].qz = pose_to_store.qz;
            }
        }

        frame.getPoseTime = clock();

        drawImg = image;
        cv::Mat drawImgMasked = cv::Mat::zeros(drawImg.size(), drawImg.type());
        april.convertToSingleChannel(image, gray);

        frame.toGrayTime = clock();

        clock_t start, end;
        //for timing our detection
        start = clock();

        bool circularWindow = parameters->circularWindow;

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
            trackerStatus[i].maskCenters.clear();
        }

        for (int i = 0; i < trackerNum; i++)
        {
            cv::Mat rpos = (cv::Mat_<double>(4, 1) << -trackerStatus[i].a, trackerStatus[i].b, -trackerStatus[i].c, 1);

            //transform boards position based on our calibration data

            rpos.at<double>(3, 0) = 1;
            rpos = wtranslation.inv() * rpos;

            cv::Vec3d rvec;
            cv::Vec3d tvec;

            if (trackerStatus[i].pose_valid == 0)
            {
                Quaternion<double> q = Quaternion<double>(trackerStatus[i].qw, trackerStatus[i].qx, trackerStatus[i].qy, trackerStatus[i].qz);
                q = q.UnitQuaternion();

                //q = Quaternion<double>(0, 0, 1, 0) * (wrotation * q) * Quaternion<double>(0, 0, 1, 0);
                q = wrotation.inverse() * Quaternion<double>(0, 0, 1, 0).inverse() * q * Quaternion<double>(0, 0, 1, 0).inverse();

                rvec = quat2rodr(q.w, q.x, q.y, q.z);
                tvec[0] = rpos.at<double>(0, 0);
                tvec[1] = rpos.at<double>(1, 0);
                tvec[2] = rpos.at<double>(2, 0);
            }
            else {
                tvec = trackerStatus[i].boardTvec;
                rvec = trackerStatus[i].boardRvec;
            }

            std::vector<cv::Point3f> offsetin, offsetout;
            offsetin.push_back(boardCenters[i]);
            //offsetin.push_back(cv::Point3f(0, 0, 0));
            offsetFromBoardToCameraSpace(offsetin, rvec, tvec, &offsetout);

            std::vector<cv::Point3d> point;
            point.push_back(cv::Point3d(rpos.at<double>(0, 0), rpos.at<double>(1, 0), rpos.at<double>(2, 0)));
            point.push_back(cv::Point3d(trackerStatus[i].boardTvec));
            point.push_back(cv::Point3d(rpos.at<double>(0, 0), rpos.at<double>(1, 0), rpos.at<double>(2, 0)) + cv::Point3d(offsetout[0]));
            point.push_back(cv::Point3d(trackerStatus[i].boardTvec) + cv::Point3d(offsetout[0]));

            std::vector<cv::Point2d> projected;
            cv::Vec3d identity_rvec, identity_tvec;

            cv::projectPoints(point, identity_rvec, identity_tvec, parameters->camMat, parameters->distCoeffs, projected);


            circleOnPostRotatedImg(drawImg, projected[0], 5, cv::Scalar(0, 0, 255), rotateFlag, 2, 8, 0);
            circleOnPostRotatedImg(drawImgMasked, projected[0], 5, cv::Scalar(0, 0, 255), rotateFlag, 2, 8, 0);
            circleOnPostRotatedImg(drawImg, projected[2], 5, cv::Scalar(0, 255, 255), rotateFlag, 2, 8, 0);
            circleOnPostRotatedImg(drawImgMasked, projected[2], 5, cv::Scalar(0, 255, 255), rotateFlag, 2, 8, 0);
            circleOnPostRotatedImg(drawImg, projected[3], 5, cv::Scalar(255, 0, 255), rotateFlag, 2, 8, 0);
            circleOnPostRotatedImg(drawImgMasked, projected[3], 5, cv::Scalar(255, 0, 255), rotateFlag, 2, 8, 0);

            if (trackerStatus[i].pose_valid == 0)
            {

                if (!trackerStatus[i].boardFound)
                {
                    trackerStatus[i].maskCenters.push_back(projected[2]);
                    trackerStatus[i].searchSize = (int)(parameters->searchWindow*parameters->camMat.at<double>(0,0)/point[2].z);
                }
                else
                {
                    trackerStatus[i].maskCenters.push_back(projected[3]);
                    trackerStatus[i].searchSize = (int)(parameters->searchWindow*parameters->camMat.at<double>(0,0)/point[3].z);
                }

                trackerStatus[i].boardFound = true;
                trackerStatus[i].boardFoundDriver = true;
                trackerStatus[i].boardTvec = tvec;
                trackerStatus[i].boardTvecDriver = tvec;
                trackerStatus[i].boardRvec = rvec;

            }
            else
            {
                if (trackerStatus[i].boardFound)
                {
                    trackerStatus[i].maskCenters.push_back(projected[3]);
                    trackerStatus[i].searchSize = (int)(parameters->searchWindow*parameters->camMat.at<double>(0,0)/point[3].z);
                }

                trackerStatus[i].boardFoundDriver = false;        //do we really need to do this? test later
            }

        }

        frame.processPoseTime = clock();

        didMatchTemplate = false;

        for (int i = 0; i < trackerNum; i++)
        {
            if (!trackerStatus[i].doImageMatching)
                continue;

            int searchLeft   = static_cast<int>(trackerStatus[i].oldCenter.x - 3*trackerStatus[i].searchSize);
            int searchRight  = static_cast<int>(trackerStatus[i].oldCenter.x + 3*trackerStatus[i].searchSize);
            int searchTop    = static_cast<int>(trackerStatus[i].oldCenter.y - 3*trackerStatus[i].searchSize);
            int searchBottom = static_cast<int>(trackerStatus[i].oldCenter.y + 3*trackerStatus[i].searchSize);

            searchLeft   = (searchLeft   >= gray.cols) ? (gray.cols - 1) : ((searchLeft   < 0) ? 0 : searchLeft);
            searchRight  = (searchRight  >= gray.cols) ? (gray.cols - 1) : ((searchRight  < 0) ? 0 : searchRight);
            searchTop    = (searchTop    >= gray.rows) ? (gray.rows - 1) : ((searchTop    < 0) ? 0 : searchTop);
            searchBottom = (searchBottom >= gray.rows) ? (gray.rows - 1) : ((searchBottom < 0) ? 0 : searchBottom);

            const int x = searchLeft, y = searchTop;
            const int w = searchRight - searchLeft, h = searchBottom - searchTop;

            if ((w <= 8*trackerStatus[i].snapshot.cols) || (h <= 8*trackerStatus[i].snapshot.rows))
                continue;

            cv::Mat searchGray;
            cv::resize(cv::Mat(gray, cv::Rect(x, y, w, h)), searchGray, cv::Size(), 1.0/8.0, 1.0/8.0, cv::INTER_NEAREST);

            cv::matchTemplate(searchGray, trackerStatus[i].snapshot, matchTemplateResult, cv::TM_CCOEFF);
            didMatchTemplate = true;

            float min = 0.0f, max = 0.0f;
            auto it = matchTemplateResult.begin<float>(), it_end = matchTemplateResult.end<float>();
            for (; it != it_end; ++it)
            {
                if (*it < min)
                    min = *it;
                if (*it > max)
                    max = *it;
            }
            it = matchTemplateResult.begin<float>();
            for (; it != it_end; ++it)
            {
                *it = (*it - min)/(max - min);
            }

            cv::Mat thresholded;
            cv::threshold(matchTemplateResult, thresholded, 0.85, 1.0, cv::THRESH_BINARY);

            thresholded.convertTo(thresholded, CV_8UC1, 255.0);

#if 0
            static bool foo = true;
            if (foo)
            {
                std::cout << "thresholded.type(): " << cvTypeToStr(thresholded.type()) << std::endl;
                foo = false;
            }
#endif
            cv::Mat labels, stats, centroids;
            int cc = cv::connectedComponentsWithStats(thresholded, labels, stats, centroids);

#if 0
            static bool foo = true;
            if (foo)
            {
                std::cout << "cc: " << cc << std::endl;
                std::cout << "stats: " << stats << std::endl;
                std::cout << "centroids: " << centroids << std::endl;
                foo = false;
            }
#endif
            for (int j = 1; j < centroids.rows; ++j)
            {
#if 0
                static bool foo = true;
                if (foo)
                {
                    std::cout << "centroids.type(): " << cvTypeToStr(centroids.type()) << std::endl;
                    std::cout << "centroid: " << centroids.row(j) << std::endl;
                    foo = false;
                }
#endif
                int left   = 8*static_cast<int>(centroids.at<double>(j,0)) + searchLeft;
                int top    = 8*static_cast<int>(centroids.at<double>(j,1)) + searchTop;
                int right  = left + 8*trackerStatus[i].snapshot.cols;
                int bottom = top  + 8*trackerStatus[i].snapshot.rows;

#if 0
                static bool foo = true;
                if (foo)
                {
                    std::cout << "centroids.at<int>(j,0): " << centroids.at<int>(j,0) << std::endl;
                    std::cout << "centroids.at<int>(j,1): " << centroids.at<int>(j,1) << std::endl;
                    std::cout << "centroids.at<double>(j,0): " << centroids.at<double>(j,0) << std::endl;
                    std::cout << "centroids.at<double>(j,1): " << centroids.at<double>(j,1) << std::endl;

                    std::cout << "searchLeft: " << searchLeft << std::endl;
                    std::cout << "searchTop:  " << searchTop << std::endl;

                    std::cout << "left:   " << left << std::endl;
                    std::cout << "top:    " << top << std::endl;
                    std::cout << "right:  " << right << std::endl;
                    std::cout << "bottom: " << bottom << std::endl;
                    foo = false;
                }
#endif

                cv::rectangle(drawImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255), 3);
                cv::rectangle(drawImgMasked, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255), 3);

                cv::Point2f center = cv::Point2f((left + right)/2, (top + bottom)/2);

                auto tmp = center.x;
                switch (rotateFlag) {
                case cv::ROTATE_180:
                    center.x = rotated_cols - center.x;
                    center.y = rotated_rows - center.y;
                    break;
                case cv::ROTATE_90_CLOCKWISE:
                    center.x = rotated_cols - center.y;
                    center.y = tmp;
                    break;
                case cv::ROTATE_90_COUNTERCLOCKWISE:
                    center.x = center.y;
                    center.y = rotated_rows - tmp;
                    break;
                }

                trackerStatus[i].maskCenters.push_back(center);
            }
        }

        //Then define your mask image
        cv::Mat mask = cv::Mat::zeros(gray.size(), gray.type());

        cv::Mat dstImage = cv::Mat::zeros(gray.size(), gray.type());

        bool doMasking = false;

        //I assume you want to draw the circle at the center of your image, with a radius of 50
        for (int i = 0; i < trackerNum; i++)
        {
            for (int j = 0; j < trackerStatus[i].maskCenters.size(); j++)
            {
                if (trackerStatus[i].maskCenters[j].x <= 0 || trackerStatus[i].maskCenters[j].y <= 0 || trackerStatus[i].maskCenters[j].x >= rotated_cols || trackerStatus[i].maskCenters[j].y >= rotated_rows)
                {
                    trackerStatus[i].boardFound = false;    //if detected tracker is out of view of the camera, we mark it as not found, as either the prediction is wrong or we wont see it anyway
                    continue;
                }
                doMasking = true;
                if (circularWindow)
                {
                    if (trackerStatus[i].searchSize > 0)
                    {
                        circleOnPostRotatedImg(mask, trackerStatus[i].maskCenters[j], trackerStatus[i].searchSize, cv::Scalar(255, 0, 0), rotateFlag, -1, 8, 0);
                        circleOnPostRotatedImg(drawImg, trackerStatus[i].maskCenters[j], trackerStatus[i].searchSize, cv::Scalar(255, 0, 0), rotateFlag, 2, 8, 0);
                        circleOnPostRotatedImg(drawImgMasked, trackerStatus[i].maskCenters[j], trackerStatus[i].searchSize, cv::Scalar(255, 0, 0), rotateFlag, 2, 8, 0);
                    }
                }
                else
                {
                    if (trackerStatus[i].searchSize > 0)
                    {
                        cv::Point topLeft = cv::Point(trackerStatus[i].maskCenters[j].x - trackerStatus[i].searchSize, 0);
                        cv::Point bottomRight = cv::Point(trackerStatus[i].maskCenters[j].x + trackerStatus[i].searchSize, rotated_rows);

                        rectangleOnPostRotatedImg(mask, topLeft, bottomRight, cv::Scalar(255, 0, 0), rotateFlag, -1);
                        rectangleOnPostRotatedImg(drawImg, topLeft, bottomRight, cv::Scalar(255, 0, 0), rotateFlag, 3);
                        rectangleOnPostRotatedImg(drawImgMasked, topLeft, bottomRight, cv::Scalar(255, 0, 0), rotateFlag, 3);
                    }
                }
            }
        }

        //Now you can copy your source image to destination image with masking
        if (doMasking)
        {
            gray.copyTo(dstImage, mask);
            gray = dstImage;
        }

        frame.doMaskTime = clock();

        //cv::imshow("test", image);

        if (manualRecalibrate)
        {
            int inputButton = 0;
            inputButton = connection->GetButtonStates();

            double timeSincePress = double(start - calibControllerLastPress) / double(CLOCKS_PER_SEC);
            if (timeSincePress > 60)                                                                        //we exit playspace calibration after 30 seconds of no input detected
            {
                gui->cb3->SetValue(false);
                wxCommandEvent event(wxEVT_COMMAND_CHECKBOX_CLICKED, gui->MANUAL_CALIB_CHECKBOX);
                parentApp->ButtonPressedSpaceCalib(event);
            }

            if (inputButton == 1)       //logic for position button first
            {
                double timeSincePress = double(start - calibControllerLastPress) / double(CLOCKS_PER_SEC);
                if (timeSincePress >= 0.2)
                {
                    if (!calibControllerPosActive)          //if position calibration is inactive, set it to active and calculate offsets 
                    {
                        calibControllerPosActive = true;
                        double pose[7];
                        connection->GetControllerPose(pose);

                        calibControllerPosOffset[0] = 100 * pose[0] - gui->manualCalibX->value;
                        calibControllerPosOffset[1] = 100 * pose[1] - gui->manualCalibY->value;
                        calibControllerPosOffset[2] = 100 * pose[2] - gui->manualCalibZ->value;

                        Quaternion<double> quat(pose[3], pose[4], pose[5], pose[6]);
                        quat.x = -quat.x;
                        quat.z = -quat.z;

                        quat.inverse().QuatRotation(calibControllerPosOffset);

                        calibControllerLastPress = clock();

                    }
                    else       //else, check if button was unpressed for half a second, then set it to inactive
                    {
                        calibControllerPosActive = false;
                    }
                }
                calibControllerLastPress = clock();
            }
            if (inputButton == 2)       //logic for position button first
            {
                double timeSincePress = double(start - calibControllerLastPress) / double(CLOCKS_PER_SEC);
                if (timeSincePress >= 0.2)
                {
                    if (!calibControllerAngleActive)          //if position calibration is inactive, set it to active and calculate offsets 
                    {
                        calibControllerAngleActive = true;
                        double pose[7];
                        connection->GetControllerPose(pose);

                        double xzLen = sqrt(pow(100 * pose[0] - gui->manualCalibX->value, 2) + pow(100 * pose[2] - gui->manualCalibZ->value, 2));
                        double angleA = atan2(100 * pose[1] - gui->manualCalibY->value, xzLen) * 57.3;
                        double angleB = atan2(100 * pose[0] - gui->manualCalibX->value, 100 * pose[2] - gui->manualCalibZ->value) * 57.3;
                        double xyzLen = sqrt(pow(100 * pose[0] - gui->manualCalibX->value, 2) + pow(100 * pose[1] - gui->manualCalibY->value, 2) + pow(100 * pose[2] - gui->manualCalibZ->value, 2));

                        calibControllerAngleOffset[0] = angleA - gui->manualCalibA->value;
                        calibControllerAngleOffset[1] = angleB - gui->manualCalibB->value;
                        calibControllerAngleOffset[2] = xyzLen;

                        calibControllerLastPress = clock();

                    }
                    else       //else, check if button was unpressed for half a second, then set it to inactive
                    {
                        calibControllerAngleActive = false;
                    }
                }
                calibControllerLastPress = clock();
            }

            if (calibControllerPosActive)
            {
                double pose[7];
                connection->GetControllerPose(pose);

                Quaternion<double> quat(pose[3], pose[4], pose[5], pose[6]);
                quat.x = -quat.x;
                quat.z = -quat.z;
                quat.QuatRotation(calibControllerPosOffset);

                gui->manualCalibX->SetValue(100 * pose[0] - calibControllerPosOffset[0]);

                if (!lockHeightCalib)
                {
                    gui->manualCalibY->SetValue(100 * pose[1] - calibControllerPosOffset[1]);
                }

                gui->manualCalibZ->SetValue(100 * pose[2] - calibControllerPosOffset[2]);

                quat.inverse().QuatRotation(calibControllerPosOffset);
            }

            if (calibControllerAngleActive)
            {
                double pose[7];
                connection->GetControllerPose(pose);

                double xzLen = sqrt(pow(100 * pose[0] - gui->manualCalibX->value, 2) + pow(100 * pose[2] - gui->manualCalibZ->value, 2));
                double angleA = atan2(100 * pose[1] - gui->manualCalibY->value, xzLen) * 57.3;
                double angleB = atan2(100 * pose[0] - gui->manualCalibX->value, 100 * pose[2] - gui->manualCalibZ->value) * 57.3;
                double xyzLen = sqrt(pow(100 * pose[0] - gui->manualCalibX->value, 2) + pow(100 * pose[1] - gui->manualCalibY->value, 2) + pow(100 * pose[2] - gui->manualCalibZ->value, 2));

                gui->manualCalibB->SetValue(angleB - calibControllerAngleOffset[1]);
                if (!lockHeightCalib)
                {
                    gui->manualCalibA->SetValue(angleA - calibControllerAngleOffset[0]);
                    calibScale = xyzLen / calibControllerAngleOffset[2];
                    if (calibScale > 1.2)
                        calibScale = 1.2;
                    else if (calibScale < 0.8)
                        calibScale = 0.8;
                }
            }

            //check that camera is facing correct direction
#if 0
            if (gui->manualCalibA->value < 90)
                gui->manualCalibA->SetValue(90);
            else if (gui->manualCalibA->value > 270)
                gui->manualCalibA->SetValue(270);
#endif

            cv::Vec3d calibRot(gui->manualCalibA->value * (M_PI/180.0), gui->manualCalibB->value * (M_PI/180.0), gui->manualCalibC->value * (M_PI/180.0));
            cv::Vec3d calibPos(gui->manualCalibX->value / 100, gui->manualCalibY->value / 100, gui->manualCalibZ->value / 100);
            cv::Vec3d calibRodr(cos(calibRot[0]) * cos(calibRot[1]) * 3.14, sin(calibRot[1]) * 3.14, sin(calibRot[0]) * cos(calibRot[1]) * 3.14);

            wtranslation = getSpaceCalibEuler(calibRot, cv::Vec3d(0, 0, 0), calibPos(0), calibPos(1), calibPos(2));

            wrotation = mRot2Quat(eulerAnglesToRotationMatrix(cv::Vec3f(calibRot)));

            cv::Mat stationPos = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
            stationPos = wtranslation * stationPos;

            Quaternion<double> stationQ = Quaternion<double>(0, 0, 1, 0) * (wrotation * Quaternion<double>(1, 0, 0, 0));

            double a = -stationPos.at<double>(0, 0);
            double b = stationPos.at<double>(1, 0);
            double c = -stationPos.at<double>(2, 0);

            connection->SendStation(0, a, b, c, stationQ.w, stationQ.x, stationQ.y, stationQ.z);
        }
        else
        {
            calibControllerLastPress = clock();
        }
        april.detectMarkers(gray, &corners, &ids, &centers, trackers);
        frame.detectTime = clock();

        for (int i = 0; i < trackerNum; ++i)
        {
            trackerStatus[i].doImageMatching = false;

            std::vector<float> trackerXCoords;
            std::vector<float> trackerYCoords;

            for (int j = 0; j < ids.size(); j++)        //check all of the found markers
            {
                if (ids[j] >= i * markersPerTracker && ids[j] < (i + 1) * markersPerTracker)            //if marker is part of current tracker
                {
                    for (int k = 0; k < corners[j].size(); k++)
                    {
                        trackerXCoords.push_back(corners[j][k].x);
                        trackerYCoords.push_back(corners[j][k].y);
                    }
                }
            }

            if (!trackerXCoords.empty() && !trackerYCoords.empty())
            {
                const auto [leftit, rightit] = std::minmax_element(trackerXCoords.begin(), trackerXCoords.end());
                const auto [topit, bottomit] = std::minmax_element(trackerYCoords.begin(), trackerYCoords.end());

                auto left = *leftit;
                auto top = *topit;
                auto right = *rightit;
                auto bottom = *bottomit;

                left   = (left   >= gray.cols) ? (gray.cols - 1) : ((left   < 0) ? 0 : left);
                right  = (right  >= gray.cols) ? (gray.cols - 1) : ((right  < 0) ? 0 : right);
                top    = (top    >= gray.rows) ? (gray.rows - 1) : ((top    < 0) ? 0 : top);
                bottom = (bottom >= gray.rows) ? (gray.rows - 1) : ((bottom < 0) ? 0 : bottom);

                const int x = static_cast<int>(left), y = static_cast<int>(top);
                const int w = static_cast<int>(right - left), h = static_cast<int>(bottom - top);

                if ((w >= 8) && (h >= 8))
                {
                    cv::resize(cv::Mat(gray, cv::Rect(x, y, w, h)), trackerStatus[i].snapshot, cv::Size(), 1.0/8.0, 1.0/8.0, cv::INTER_NEAREST);
                    trackerStatus[i].doImageMatching = true;
                    trackerStatus[i].oldCenter = cv::Point2f((left + right)/2, (top + bottom)/2);
                }
            }
        }

        for (int i = 0; i < corners.size(); i++) {
            for (int j = 0; j < corners[i].size(); j++) {
                auto tmp = corners[i][j].x;
                switch (rotateFlag) {
                case cv::ROTATE_180:
                    corners[i][j].x = rotated_cols - corners[i][j].x;
                    corners[i][j].y = rotated_rows - corners[i][j].y;
                    break;
                case cv::ROTATE_90_CLOCKWISE:
                    corners[i][j].x = rotated_cols - corners[i][j].y;
                    corners[i][j].y = tmp;
                    break;
                case cv::ROTATE_90_COUNTERCLOCKWISE:
                    corners[i][j].x = corners[i][j].y;
                    corners[i][j].y = rotated_rows - tmp;
                    break;
                }
            }
        }

        for (int i = 0; i < centers.size(); i++) {
            auto tmp = centers[i].x;
            switch (rotateFlag) {
            case cv::ROTATE_180:
                centers[i].x = rotated_cols - centers[i].x;
                centers[i].y = rotated_rows - centers[i].y;
                break;
            case cv::ROTATE_90_CLOCKWISE:
                centers[i].x = rotated_cols - centers[i].y;
                centers[i].y = tmp;
                break;
            case cv::ROTATE_90_COUNTERCLOCKWISE:
                centers[i].x = centers[i].y;
                centers[i].y = rotated_rows - tmp;
                break;
            }
        }

        std::chrono::milliseconds current_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        for (int i = 0; i < trackerNum; ++i) {

            //estimate the pose of current board

            try
            {
                trackerStatus[i].boardTvec /= calibScale;
                if (cv::aruco::estimatePoseBoard(corners, ids, trackers[connection->connectedTrackers[i].TrackerId], parameters->camMat, parameters->distCoeffs, trackerStatus[i].boardRvec, trackerStatus[i].boardTvec, trackerStatus[i].boardFound && parameters->usePredictive) <= 1)
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
                    if ((trackerStatus[i].pose_valid != 0) || ((current_timestamp.count() - trackerStatus[i].last_update_timestamp.count()) <= 100.0))
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
                               cv::destroyWindow("out");
                               });
                //apriltag_detector_destroy(td);
                mainThreadRunning = false;
                return;
            }
            trackerStatus[i].boardFound = true;

            trackerStatus[i].boardTvec *= calibScale;

            if (parameters->depthSmoothing > 0 && trackerStatus[i].boardFoundDriver && !manualRecalibrate)
            {
                //depth estimation is noisy, so try to smooth it more, especialy if using multiple cameras
                //if position is close to the position predicted by the driver, take the depth of the driver.
                //if error is big, take the calculated depth
                //error threshold is defined in the params as depth smoothing

                double distDriver = sqrt(pow(trackerStatus[i].boardTvecDriver[0], 2)
                    + pow(trackerStatus[i].boardTvecDriver[1], 2)
                    + pow(trackerStatus[i].boardTvecDriver[2], 2));

                double distPredict = sqrt(pow(trackerStatus[i].boardTvec[0], 2)
                    + pow(trackerStatus[i].boardTvec[1], 2)
                    + pow(trackerStatus[i].boardTvec[2], 2));

                cv::Vec3d normPredict = trackerStatus[i].boardTvec / distPredict;

                double dist = abs(distPredict - distDriver);

                dist = dist / parameters->depthSmoothing + 0.1;
                if (dist > 1)
                    dist = 1;

                double distFinal = (dist)*distPredict + (1 - dist) * distDriver;

                trackerStatus[i].boardTvec = normPredict * distFinal;

                //trackerStatus[i].boardTvec[2] = (dist) * trackerStatus[i].boardTvec[2] + (1-dist) * trackerStatus[i].boardTvecDriver[2];
            }

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
            rpos = wtranslation * rpos;

            //convert rodriguez rotation to quaternion
            Quaternion<double> q = rodr2quat(trackerStatus[i].boardRvec[0], trackerStatus[i].boardRvec[1], trackerStatus[i].boardRvec[2]);

            drawAxisOnPostRotatedImg(drawImg, parameters->camMat, parameters->distCoeffs, trackerStatus[i].boardRvec, trackerStatus[i].boardTvec, 0.05, rotateFlag);
            drawAxisOnPostRotatedImg(drawImgMasked, parameters->camMat, parameters->distCoeffs, trackerStatus[i].boardRvec, trackerStatus[i].boardTvec, 0.05, rotateFlag);

            q = Quaternion<double>(0, 0, 1, 0) * (wrotation * q) * Quaternion<double>(0, 0, 1, 0);

            double a = -rpos.at<double>(0, 0);
            double b = rpos.at<double>(1, 0);
            double c = -rpos.at<double>(2, 0);

            double factor;
            factor = parameters->smoothingFactor;

            if (factor < 0)
                factor = 0;
            else if (factor >= 1)
                factor = 0.99;


#if 0
            if (a < -1.5 || a > 1.5 || b < -0.5 || b > 2.5 || c < -1.5 || c > 1.5)
            {
                trackerStatus[i].boardFound = false;
            }
#endif
#if 1
            // Reject detected positions that are behind or too far from the camera
            if ((trackerStatus[i].boardTvec[2]  < 0) || (trackerStatus[i].boardTvec[2]  > 3.0))
            {
                trackerStatus[i].boardFound = false;
                continue;
            }

            // Figure out the camera aspect ratio, XZ and YZ ratio limits
            double aspectRatio = (double)rotated_cols/(double)rotated_rows;
            double XZratioLimit = 0.5*(double)rotated_cols/parameters->camMat.at<double>(0,0);
            double YZratioLimit = 0.5*(double)rotated_rows/parameters->camMat.at<double>(1,1);

            // Figure out which dimension is most likely to go outside the camera field of view
            if (abs(trackerStatus[i].boardTvec[0]/trackerStatus[i].boardTvec[1]) > aspectRatio)
            {
                // Check that X coordinate doesn't go out of camera FOV
                if (abs(trackerStatus[i].boardTvec[0]/trackerStatus[i].boardTvec[2]) > XZratioLimit)
                {
                    trackerStatus[i].boardFound = false;
                    continue;
                }
            }
            else
            {
                // Check that Y coordinate doesn't go out of camera FOV
                if (abs(trackerStatus[i].boardTvec[1]/trackerStatus[i].boardTvec[2]) > YZratioLimit)
                {
                    trackerStatus[i].boardFound = false;
                    continue;
                }
            }
#endif
#if 0
            if ((current_timestamp.count() - trackerStatus[i].last_update_timestamp.count()) > 100.0)
                continue;
#endif

            end = clock();
            double frameTime = double(end - frame.captureTime) / double(CLOCKS_PER_SEC);

            //send all the values
            //frame time is how much time passed since frame was acquired.
            if (!multicamAutocalib) {

                TrackerPose pose_local;
                pose_local.a = a;
                pose_local.b = b;
                pose_local.c = c;
                pose_local.qw = q.w;
                pose_local.qx = q.x;
                pose_local.qy = q.y;
                pose_local.qz = q.z;

                TrackerPose pose_to_send = pose_local + trackerStatus[i].pose_delta_average;

                std::istringstream ret = connection->SendTracker(connection->connectedTrackers[i].DriverId,
                                                                 pose_to_send.a,
                                                                 pose_to_send.b,
                                                                 pose_to_send.c,
                                                                 pose_to_send.qw,
                                                                 pose_to_send.qx,
                                                                 pose_to_send.qy,
                                                                 pose_to_send.qz,
                                                                 -frameTime - parameters->camLatency,
                                                                 factor);

                // printf("Response from driver: %s\n", ret.str().c_str());

                do {
                    std::string word;

                    ret >> word;

                    if (word != "updated")
                    {
                        break;
                    }

                    int idx, pose_valid;
                    TrackerPose pose_from_driver;

                    //read to our variables
                    ret >> idx;
                    ret >> pose_from_driver.a;
                    ret >> pose_from_driver.b;
                    ret >> pose_from_driver.c;
                    ret >> pose_from_driver.qw;
                    ret >> pose_from_driver.qx;
                    ret >> pose_from_driver.qy;
                    ret >> pose_from_driver.qz;
                    ret >> pose_from_driver.pose_valid;

                    trackerStatus[i].pose_delta_average = trackerStatus[i].pose_delta_average - trackerStatus[i].pose_delta_history[trackerStatus[i].pose_delta_index] / 10.0;
                    trackerStatus[i].pose_delta_history[trackerStatus[i].pose_delta_index] = compress(pose_from_driver - pose_local);
                    trackerStatus[i].pose_delta_average = trackerStatus[i].pose_delta_average + trackerStatus[i].pose_delta_history[trackerStatus[i].pose_delta_index] / 10.0;
                    if(++trackerStatus[i].pose_delta_index == 10)
                        trackerStatus[i].pose_delta_index = 0;

#if 0
                    printf("local: % 3.3f % 3.3f % 3.3f q: % 3.3f % 3.3f % 3.3f % 3.3f ",
                           a,
                           b,
                           c,
                           q.w,
                           q.x,
                           q.y,
                           q.z);
#endif
#if 1
                    printf("sent: % 3.3f % 3.3f % 3.3f q: % 3.3f % 3.3f % 3.3f % 3.3f ",
                           pose_to_send.a,
                           pose_to_send.b,
                           pose_to_send.c,
                           pose_to_send.qw,
                           pose_to_send.qx,
                           pose_to_send.qy,
                           pose_to_send.qz);
#endif
#if 1
                    printf("driver: % 3.3f % 3.3f % 3.3f q: % 3.3f % 3.3f % 3.3f % 3.3f valid: %d ",
                           pose_from_driver.a,
                           pose_from_driver.b,
                           pose_from_driver.c,
                           pose_from_driver.qw,
                           pose_from_driver.qx,
                           pose_from_driver.qy,
                           pose_from_driver.qz,
                           pose_from_driver.pose_valid);
#endif
#if 0
                    printf("Tracker delta: a: %3.3f b: %3.3f c: %3.3f qw: %3.3f qx: %3.3f qy: %3.3f qz: %3.3f\n",
                           a - trackerStatusInDriver.a,
                           b - trackerStatusInDriver.b,
                           c - trackerStatusInDriver.c,
                           q.w - trackerStatusInDriver.qw,
                           q.x - trackerStatusInDriver.qx,
                           q.y - trackerStatusInDriver.qy,
                           q.z - trackerStatusInDriver.qz);
#endif
                    printf("average: % 3.4f % 3.4f % 3.4f q: % 3.4f % 3.4f % 3.4f % 3.4f\n",
                           trackerStatus[i].pose_delta_average.a,
                           trackerStatus[i].pose_delta_average.b,
                           trackerStatus[i].pose_delta_average.c,
                           trackerStatus[i].pose_delta_average.qw,
                           trackerStatus[i].pose_delta_average.qx,
                           trackerStatus[i].pose_delta_average.qy,
                           trackerStatus[i].pose_delta_average.qz);

                } while (false);

            }
            else if (trackerStatus[i].boardFoundDriver)
            {
                //get rotations of tracker from camera

                cv::Vec3d pose;
                pose = trackerStatus[i].boardTvec;
                double xzLen = sqrt(pow(100 * pose[0], 2) + pow(100 * pose[2], 2));
                double angleA = atan2(100 * pose[1], xzLen) * 57.3;
                double angleB = atan2(100 * pose[0], 100 * pose[2]) * 57.3;
                double xyzLen = sqrt(pow(100 * pose[0], 2) + pow(100 * pose[1], 2) + pow(100 * pose[2], 2));

                pose = trackerStatus[i].boardTvecDriver;
                double xzLenDriver = sqrt(pow(100 * pose[0], 2) + pow(100 * pose[2], 2));
                double angleADriver = atan2(100 * pose[1], xzLenDriver) * 57.3;
                double angleBDriver = atan2(100 * pose[0], 100 * pose[2]) * 57.3;
                double xyzLenDriver = sqrt(pow(100 * pose[0], 2) + pow(100 * pose[1], 2) + pow(100 * pose[2], 2));

                cv::Mat rpos1 = cv::Mat_<double>(4, 1);
                cv::Mat rpos2 = cv::Mat_<double>(4, 1);

                for (int x = 0; x < 3; x++)
                {
                    rpos1.at<double>(x, 0) = trackerStatus[i].boardTvec[x];
                }
                rpos1.at<double>(3, 0) = 1;
                rpos1 = wtranslation * rpos1;
                rpos1.at<double>(0, 0) = -rpos1.at<double>(0, 0);
                rpos1.at<double>(2, 0) = -rpos1.at<double>(2, 0);

                for (int x = 0; x < 3; x++)
                {
                    rpos2.at<double>(x, 0) = trackerStatus[i].boardTvecDriver[x];
                }
                rpos2.at<double>(3, 0) = 1;
                rpos2 = wtranslation * rpos2;
                rpos2.at<double>(0, 0) = -rpos2.at<double>(0, 0);
                rpos2.at<double>(2, 0) = -rpos2.at<double>(2, 0);

                double dX = (rpos1.at<double>(0) - rpos2.at<double>(0)) * 0.1;
                double dY = -(rpos1.at<double>(1) - rpos2.at<double>(1)) * 0.1;
                double dZ = (rpos1.at<double>(2) - rpos2.at<double>(2)) * 0.1;

                if (abs(dX) > 0.01)
                    dX = 0.1 * (dX / abs(dX));
                if (abs(dY) > 0.1)
                    dY = 0.1 * (dY / abs(dY));
                if (abs(dZ) > 0.1)
                    dZ = 0.1 * (dZ / abs(dZ));

                gui->CallAfter([this, dX, dY, dZ, angleA, angleADriver, angleB, angleBDriver] ()
                               {
                                   gui->manualCalibX->SetValue(gui->manualCalibX->value + dX);
                                   gui->manualCalibY->SetValue(gui->manualCalibY->value + dY);
                                   gui->manualCalibZ->SetValue(gui->manualCalibZ->value + dZ);
                                   // gui->manualCalibA->SetValue(gui->manualCalibA->value + 0.1 * (angleA - angleADriver));
                                   // gui->manualCalibB->SetValue(gui->manualCalibB->value + 0.1 * (angleB - angleBDriver));
                               });
                calibScale = calibScale - 0.1 * (1 - (xyzLenDriver / xyzLen));

                if (calibScale > 1.2)
                    calibScale = 1.2;
                else if (calibScale < 0.8)
                    calibScale = 0.8;

            }


        }

        frame.sendTrackerTime = clock();

        for (int i = 0; i < corners.size(); i++) {
            for (int j = 0; j < corners[i].size(); j++) {
                auto tmp = corners[i][j].x;
                switch (rotateFlag) {
                case cv::ROTATE_180:
                    corners[i][j].x = image.cols - corners[i][j].x;
                    corners[i][j].y = image.rows - corners[i][j].y;
                    break;
                case cv::ROTATE_90_CLOCKWISE:
                    corners[i][j].x = corners[i][j].y;
                    corners[i][j].y = image.rows - tmp;
                    break;
                case cv::ROTATE_90_COUNTERCLOCKWISE:
                    corners[i][j].x = image.cols - corners[i][j].y;
                    corners[i][j].y = tmp;
                    break;
                }
            }
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

        if (showTimeProfile)
        {
            cv::Mat *outImg = new cv::Mat();

            int frameWriteMsecs = int(20000.0 * double(frame.swapTime - frame.captureTime) / double(CLOCKS_PER_SEC));
            int frameReadMsecs = int(20000.0 * double(frame.copyFreshTime - frame.captureTime) / double(CLOCKS_PER_SEC));
            int getPoseMsecs = int(20000.0 * double(frame.getPoseTime - frame.captureTime) / double(CLOCKS_PER_SEC));
            int toGrayMsecs = int(20000.0 * double(frame.toGrayTime - frame.captureTime) / double(CLOCKS_PER_SEC));
            int processPoseMsecs = int(20000.0 * double(frame.processPoseTime - frame.captureTime) / double(CLOCKS_PER_SEC));
            int doMaskMsecs = int(20000.0 * double(frame.doMaskTime - frame.captureTime) / double(CLOCKS_PER_SEC));
            int detectMsecs = int(20000.0 * double(frame.detectTime - frame.captureTime) / double(CLOCKS_PER_SEC));
            int sendTrackerMsecs = int(20000.0 * double(frame.sendTrackerTime - frame.captureTime) / double(CLOCKS_PER_SEC));

            rectangle(statsImg, cv::Point(statsCurX, 0),                                cv::Point(statsCurX + 2, statsImg.rows), cv::Scalar(0, 0, 0), -1);                        // Clear
            rectangle(statsImg, cv::Point(statsCurX, statsImg.rows - 0),                cv::Point(statsCurX + 2, statsImg.rows - frameWriteMsecs), cv::Scalar(133, 178, 208), -1);// Light Brown
            rectangle(statsImg, cv::Point(statsCurX, statsImg.rows - frameWriteMsecs),  cv::Point(statsCurX + 2, statsImg.rows - frameReadMsecs), cv::Scalar(23, 73, 207), -1);   // Orange
            rectangle(statsImg, cv::Point(statsCurX, statsImg.rows - frameReadMsecs),   cv::Point(statsCurX + 2, statsImg.rows - getPoseMsecs), cv::Scalar(140, 117, 45), -1);    // Blue
            rectangle(statsImg, cv::Point(statsCurX, statsImg.rows - getPoseMsecs),     cv::Point(statsCurX + 2, statsImg.rows - toGrayMsecs), cv::Scalar(51, 140, 117), -1);     // Green
            rectangle(statsImg, cv::Point(statsCurX, statsImg.rows - toGrayMsecs),      cv::Point(statsCurX + 2, statsImg.rows - processPoseMsecs), cv::Scalar(20, 89, 152), -1); // Brown
            rectangle(statsImg, cv::Point(statsCurX, statsImg.rows - processPoseMsecs), cv::Point(statsCurX + 2, statsImg.rows - doMaskMsecs), cv::Scalar(61, 172, 249), -1);     // Yellow
            rectangle(statsImg, cv::Point(statsCurX, statsImg.rows - doMaskMsecs),      cv::Point(statsCurX + 2, statsImg.rows - detectMsecs), cv::Scalar(51, 140, 117), -1);     // Green
            rectangle(statsImg, cv::Point(statsCurX, statsImg.rows - detectMsecs),      cv::Point(statsCurX + 2, statsImg.rows - sendTrackerMsecs), cv::Scalar(20, 89, 152), -1); // Brown

            if (statsCurX % 20 > 15) {
                for (int y = 99 ; y < 1000; y += 100) {
                    rectangle(statsImg, cv::Point(statsCurX, statsImg.rows - y),  cv::Point(statsCurX + 2, statsImg.rows - y + 2), cv::Scalar(0, 0, 0), -1);
                }
            }

            statsCurX += 3;
            statsCurX = (statsCurX >= 2000) ? 0 : statsCurX;

            statsImg.copyTo(*outImg);
            gui->CallAfter([outImg] ()
                           {
                           cv::imshow("stats", *outImg);
                           cv::waitKey(1);
                           delete(outImg);
                           });
        }

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
            
            cv::Mat *outMatchTemplateResult = new cv::Mat();
            matchTemplateResult.convertTo(*outMatchTemplateResult, CV_8UC1, 255.0);

            gui->CallAfter([outImg, outMatchTemplateResult, didMatchTemplate, this] ()
                           {
                           if (this->rotate)
                           {
                           cv::rotate(*outImg, *outImg, this->rotateFlag);
                           }
                           cv::imshow("out", *outImg);
                           //if (didMatchTemplate) cv::imshow("matchTemplateResult", *outMatchTemplateResult);
                           cv::waitKey(1);
                           delete(outImg);
                           delete(outMatchTemplateResult);
                           });
        }
        //time of marker detection
    }
    gui->CallAfter([] ()
                   {
                   cv::destroyWindow("out");
                   });
    if (showTimeProfile)
    {
        gui->CallAfter([] ()
                       {
                       cv::destroyWindow("stats");
                       });
    }
}
