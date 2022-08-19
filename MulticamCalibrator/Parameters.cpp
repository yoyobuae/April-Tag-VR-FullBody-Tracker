#define GET_VARIABLE_NAME(Variable) (#Variable)

#include "Parameters.h"

Parameters::Parameters()
{
    Load();
}

void Parameters::Load()
{
    cv::FileStorage fs("params.yml", cv::FileStorage::READ);		//open test.yml file to read parameters

    if (!fs["cameraAddr1"].empty())			//if file exists, load all parameters from file into variables
    {
        fs["markersPerTracker"] >> markersPerTracker;
        if (markersPerTracker <= 0)
            markersPerTracker = 45;
        aruco_params = cv::aruco::DetectorParameters::create();
        aruco_params->detectInvertedMarker = true;
        aruco_params->cornerRefinementMethod = 2;
        cv::FileNode fn = fs["arucoParams"];
        if (!fn.empty())		//load all saved aruco params
        {
            fn["cornerRefinementMethod"] >> aruco_params->cornerRefinementMethod;
            fn["markerBorderBits"] >> aruco_params->markerBorderBits;

            fn["adaptiveThreshConstant"] >> aruco_params->adaptiveThreshConstant;
            fn["adaptiveThreshWinSizeMax"] >> aruco_params->adaptiveThreshWinSizeMax;
            fn["adaptiveThreshWinSizeMin"] >> aruco_params->adaptiveThreshWinSizeMin;
            fn["adaptiveThreshWinSizeStep"] >> aruco_params->adaptiveThreshWinSizeStep;
            fn["aprilTagCriticalRad"] >> aruco_params->aprilTagCriticalRad;
            fn["aprilTagDeglitch"] >> aruco_params->aprilTagDeglitch;
            fn["aprilTagMaxLineFitMse"] >> aruco_params->aprilTagMaxLineFitMse;
            fn["aprilTagMaxNmaxima"] >> aruco_params->aprilTagMaxNmaxima;
            fn["aprilTagMinClusterPixels"] >> aruco_params->aprilTagMinClusterPixels;
            fn["aprilTagMinWhiteBlackDiff"] >> aruco_params->aprilTagMinWhiteBlackDiff;
            fn["aprilTagQuadDecimate"] >> aruco_params->aprilTagQuadDecimate;
            fn["aprilTagQuadSigma"] >> aruco_params->aprilTagQuadSigma;
            fn["cornerRefinementMaxIterations"] >> aruco_params->cornerRefinementMaxIterations;
            fn["cornerRefinementMinAccuracy"] >> aruco_params->cornerRefinementMinAccuracy;
            fn["cornerRefinementWinSize"] >> aruco_params->cornerRefinementWinSize;
            fn["detectInvertedMarker"] >> aruco_params->detectInvertedMarker;
            fn["errorCorrectionRate"] >> aruco_params->errorCorrectionRate;
            fn["maxErroneousBitsInBorderRate"] >> aruco_params->maxErroneousBitsInBorderRate;
            fn["maxMarkerPerimeterRate"] >> aruco_params->maxMarkerPerimeterRate;
            fn["minCornerDistanceRate"] >> aruco_params->minCornerDistanceRate;
            fn["minDistanceToBorder"] >> aruco_params->minDistanceToBorder;
            fn["minMarkerDistanceRate"] >> aruco_params->minMarkerDistanceRate;
            fn["minMarkerPerimeterRate"] >> aruco_params->minMarkerPerimeterRate;
            fn["minOtsuStdDev"] >> aruco_params->minOtsuStdDev;
            fn["perspectiveRemoveIgnoredMarginPerCell"] >> aruco_params->perspectiveRemoveIgnoredMarginPerCell;
            fn["perspectiveRemovePixelPerCell"] >> aruco_params->perspectiveRemovePixelPerCell;
            fn["polygonalApproxAccuracyRate"] >> aruco_params->polygonalApproxAccuracyRate;
        }
        fs["cameraAddr1"] >> cameraAddr1;
        fs["cameraAddr2"] >> cameraAddr2;
        fs["cameraApiPreference1"] >> cameraApiPreference1;
        fs["cameraApiPreference2"] >> cameraApiPreference2;
        fs["camFps1"] >> camFps1;
        fs["camFps2"] >> camFps2;
        fs["camHeight1"] >> camHeight1;
        fs["camHeight2"] >> camHeight2;
        fs["camWidth1"] >> camWidth1;
        fs["camWidth2"] >> camWidth2;
        fs["cameraMatrix1"] >> camMat1;
        fs["cameraMatrix2"] >> camMat2;
        fs["distortionCoeffs1"] >> distCoeffs1;
        fs["distortionCoeffs2"] >> distCoeffs2;
        fs["stdDeviationsIntrinsics1"] >> stdDeviationsIntrinsics1;
        fs["stdDeviationsIntrinsics2"] >> stdDeviationsIntrinsics2;
        fs["perViewErrors1"] >> perViewErrors1;
        fs["perViewErrors2"] >> perViewErrors2;
        fs["allCharucoCorners1"] >> allCharucoCorners1;
        fs["allCharucoCorners2"] >> allCharucoCorners2;
        fs["allCharucoIds1"] >> allCharucoIds1;
        fs["allCharucoIds2"] >> allCharucoIds2;
        fs["trackerNum"] >> trackerNum;
        fs["markerSize"] >> markerSize;
        fs["numOfPrevValues"] >> numOfPrevValues;
        fs["quadDecimate"] >> quadDecimate;
        fs["searchWindow1"] >> searchWindow1;
        fs["searchWindow2"] >> searchWindow2;
        fs["usePredictive"] >> usePredictive;
        fs["calibrationTracker"] >> calibrationTracker;
        fs["rotateCl1"] >> rotateCl1;
        fs["rotateCl2"] >> rotateCl2;
        fs["rotateCounterCl1"] >> rotateCounterCl1;
        fs["rotateCounterCl2"] >> rotateCounterCl2;
        fs["coloredMarkers"] >> coloredMarkers;
        fs["calibOffsetX1"] >> calibOffsetX1;
        fs["calibOffsetX2"] >> calibOffsetX2;
        fs["calibOffsetY1"] >> calibOffsetY1;
        fs["calibOffsetY2"] >> calibOffsetY2;
        fs["calibOffsetZ1"] >> calibOffsetZ1;
        fs["calibOffsetZ2"] >> calibOffsetZ2;
        fs["calibOffsetA1"] >> calibOffsetA1;
        fs["calibOffsetA2"] >> calibOffsetA2;
        fs["calibOffsetB1"] >> calibOffsetB1;
        fs["calibOffsetB2"] >> calibOffsetB2;
        fs["calibOffsetC1"] >> calibOffsetC1;
        fs["calibOffsetC2"] >> calibOffsetC2;
        fs["circularWindow"] >> circularWindow;
        fs["smoothingFactor"] >> smoothingFactor;
        fs["ignoreTracker0"] >> ignoreTracker0;
        fs["wtranslation1"] >> wtranslation1;
        fs["wtranslation2"] >> wtranslation2;
        cv::Mat wrotmat1, wrotmat2;
        fs["wrotation1"] >> wrotmat1;
        fs["wrotation2"] >> wrotmat2;
        fs["cameraSettings1"] >> cameraSettings1;
        fs["cameraSettings2"] >> cameraSettings2;
        fs["chessboardCalib"] >> chessboardCalib;
        fs["camLatency"] >> camLatency;
        fs["circularMarkers"] >> circularMarkers;
        fs["trackerCalibDistance"] >> trackerCalibDistance;
        if (trackerCalibDistance < 0.5)
            trackerCalibDistance = 0.5;
        fs["cameraCalibSamples"] >> cameraCalibSamples;
        if (cameraCalibSamples < 15)
            cameraCalibSamples = 15;
        fs["settingsParameters1"] >> settingsParameters1;
        fs["settingsParameters2"] >> settingsParameters2;
        fs["cameraAutoexposure1"] >> cameraAutoexposure1;
        fs["cameraAutoexposure2"] >> cameraAutoexposure2;
        fs["cameraExposure1"] >> cameraExposure1;
        fs["cameraExposure2"] >> cameraExposure2;
        fs["cameraGain1"] >> cameraGain1;
        fs["cameraGain2"] >> cameraGain2;
        fs["trackerCalibCenters"] >> trackerCalibCenters;
        fs["depthSmoothing"] >> depthSmoothing;
        fs["additionalSmoothing"] >> additionalSmoothing;
        fs["markerLibrary"] >> markerLibrary;
        fs["languageSelection"] >> languageSelection;
        fs["calibScale"] >> calibScale;
        if (calibScale < 0.5)
            calibScale = 1;
        fs["ipcAddr"] >> ipcAddr;
        if(!wrotmat1.empty())
            wrotation1 = Quaternion<double>(wrotmat1.at<double>(0), wrotmat1.at<double>(1), wrotmat1.at<double>(2), wrotmat1.at<double>(3));
        if(!wrotmat2.empty())
            wrotation2 = Quaternion<double>(wrotmat2.at<double>(0), wrotmat2.at<double>(1), wrotmat2.at<double>(2), wrotmat2.at<double>(3));
        fn = fs["trackers"];
        if (!fn.empty())		//load all saved markers
        {
            cv::FileNodeIterator curMarker = fn.begin(), it_end = fn.end();
            for (; curMarker != it_end; ++curMarker)
            {
                std::vector<std::vector<cv::Point3f>> boardCorners;
                std::vector<int> boardIds;
                cv::FileNode item = *curMarker;
                item["trackerIds"] >> boardIds;
                //data.push_back(tmp);
                cv::FileNode fnCorners = item["trackerCorners"];
                if (!fnCorners.empty())
                {
                    cv::FileNodeIterator curCorners = fnCorners.begin(), itCorners_end = fnCorners.end();
                    for (; curCorners != itCorners_end; ++curCorners)
                    {
                        std::vector<cv::Point3f> corners;
                        cv::FileNode item2 = *curCorners;
                        item2 >> corners;
                        boardCorners.push_back(corners);
                    }
                }
                cv::Ptr<cv::aruco::Board> arBoard = cv::aruco::Board::create(boardCorners, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), boardIds);
                trackers.push_back(arBoard);
            }
        }
    }
    fs.release();
    if(languageSelection == 1)
        language = get_lang_chinese();
}

void Parameters::Save()
{
    cv::FileStorage fs("params.yml", cv::FileStorage::WRITE);
    fs << "markersPerTracker" << markersPerTracker;
    fs << "arucoParams";
    fs << "{";
    fs << "cornerRefinementMethod" << aruco_params->cornerRefinementMethod;
    fs << "markerBorderBits" << aruco_params->markerBorderBits;
    fs << "adaptiveThreshConstant" << aruco_params->adaptiveThreshConstant;
    fs << "adaptiveThreshWinSizeMax" << aruco_params->adaptiveThreshWinSizeMax;
    fs << "adaptiveThreshWinSizeMin" << aruco_params->adaptiveThreshWinSizeMin;
    fs << "adaptiveThreshWinSizeStep" << aruco_params->adaptiveThreshWinSizeStep;
    fs << "aprilTagCriticalRad" << aruco_params->aprilTagCriticalRad;
    fs << "aprilTagDeglitch" << aruco_params->aprilTagDeglitch;
    fs << "aprilTagMaxLineFitMse" << aruco_params->aprilTagMaxLineFitMse;
    fs << "aprilTagMaxNmaxima" << aruco_params->aprilTagMaxNmaxima;
    fs << "aprilTagMinClusterPixels" << aruco_params->aprilTagMinClusterPixels;
    fs << "aprilTagMinWhiteBlackDiff" << aruco_params->aprilTagMinWhiteBlackDiff;
    fs << "aprilTagQuadDecimate" << aruco_params->aprilTagQuadDecimate;
    fs << "aprilTagQuadSigma" << aruco_params->aprilTagQuadSigma;
    fs << "cornerRefinementMaxIterations" << aruco_params->cornerRefinementMaxIterations;
    fs << "cornerRefinementMinAccuracy" << aruco_params->cornerRefinementMinAccuracy;
    fs << "cornerRefinementWinSize" << aruco_params->cornerRefinementWinSize;
    fs << "detectInvertedMarker" << aruco_params->detectInvertedMarker;
    fs << "errorCorrectionRate" << aruco_params->errorCorrectionRate;
    fs << "maxErroneousBitsInBorderRate" << aruco_params->maxErroneousBitsInBorderRate;
    fs << "maxMarkerPerimeterRate" << aruco_params->maxMarkerPerimeterRate;
    fs << "minCornerDistanceRate" << aruco_params->minCornerDistanceRate;
    fs << "minDistanceToBorder" << aruco_params->minDistanceToBorder;
    fs << "minMarkerDistanceRate" << aruco_params->minMarkerDistanceRate;
    fs << "minMarkerPerimeterRate" << aruco_params->minMarkerPerimeterRate;
    fs << "minOtsuStdDev" << aruco_params->minOtsuStdDev;
    fs << "perspectiveRemoveIgnoredMarginPerCell" << aruco_params->perspectiveRemoveIgnoredMarginPerCell;
    fs << "perspectiveRemovePixelPerCell" << aruco_params->perspectiveRemovePixelPerCell;
    fs << "polygonalApproxAccuracyRate" << aruco_params->polygonalApproxAccuracyRate;
    fs << "}";
    fs << "cameraAddr1" << cameraAddr1;
    fs << "cameraAddr2" << cameraAddr2;
    fs << "cameraApiPreference1" << cameraApiPreference1;
    fs << "cameraApiPreference2" << cameraApiPreference2;
    fs << "camFps1" << camFps1;
    fs << "camFps2" << camFps2;
    fs << "camHeight1" << camHeight1;
    fs << "camHeight2" << camHeight2;
    fs << "camWidth1" << camWidth1;
    fs << "camWidth2" << camWidth2;
    fs << "cameraMatrix1" << camMat1;
    fs << "cameraMatrix2" << camMat2;
    fs << "distortionCoeffs1" << distCoeffs1;
    fs << "distortionCoeffs2" << distCoeffs2;
    fs << "stdDeviationsIntrinsics1" << stdDeviationsIntrinsics1;
    fs << "stdDeviationsIntrinsics2" << stdDeviationsIntrinsics2;
    fs << "perViewErrors1" << perViewErrors1;
    fs << "perViewErrors2" << perViewErrors2;
    fs << "allCharucoCorners1" << allCharucoCorners1;
    fs << "allCharucoCorners2" << allCharucoCorners2;
    fs << "allCharucoIds1" << allCharucoIds1;
    fs << "allCharucoIds2" << allCharucoIds2;
    fs << "trackerNum" << trackerNum;
    fs << "markerSize" << markerSize;
    fs << "numOfPrevValues" << numOfPrevValues;
    fs << "quadDecimate" << quadDecimate;
    fs << "searchWindow1" << searchWindow1;
    fs << "searchWindow2" << searchWindow2;
    fs << "usePredictive" << usePredictive;
    fs << "calibrationTracker" << calibrationTracker;
    fs << "rotateCl1" << rotateCl1;
    fs << "rotateCl2" << rotateCl2;
    fs << "rotateCounterCl1" << rotateCounterCl1;
    fs << "rotateCounterCl2" << rotateCounterCl2;
    fs << "coloredMarkers" << coloredMarkers;
    fs << "calibOffsetX1" << calibOffsetX1;
    fs << "calibOffsetX2" << calibOffsetX2;
    fs << "calibOffsetY1" << calibOffsetY1;
    fs << "calibOffsetY2" << calibOffsetY2;
    fs << "calibOffsetZ1" << calibOffsetZ1;
    fs << "calibOffsetZ2" << calibOffsetZ2;
    fs << "calibOffsetA1" << calibOffsetA1;
    fs << "calibOffsetA2" << calibOffsetA2;
    fs << "calibOffsetB1" << calibOffsetB1;
    fs << "calibOffsetB2" << calibOffsetB2;
    fs << "calibOffsetC1" << calibOffsetC1;
    fs << "calibOffsetC2" << calibOffsetC2;
    fs << "circularWindow" << circularWindow;
    fs << "smoothingFactor" << smoothingFactor;
    fs << "ignoreTracker0" << ignoreTracker0;
    fs << "wtranslation1" << wtranslation1;
    fs << "wtranslation2" << wtranslation2;
    fs << "wrotation1" << (cv::Mat_<double>(4,1) << wrotation1.w,wrotation1.x,wrotation1.y,wrotation1.z);
    fs << "wrotation2" << (cv::Mat_<double>(4,1) << wrotation2.w,wrotation2.x,wrotation2.y,wrotation2.z);
    fs << "cameraSettings1" << cameraSettings1;
    fs << "cameraSettings2" << cameraSettings2;
    fs << "chessboardCalib" << chessboardCalib;
    fs << "camLatency" << camLatency;
    fs << "circularMarkers" << circularMarkers;
    fs << "trackerCalibDistance" << trackerCalibDistance;
    fs << "cameraCalibSamples" << cameraCalibSamples;
    fs << "settingsParameters1" << settingsParameters1;
    fs << "settingsParameters2" << settingsParameters2;
    fs << "cameraAutoexposure1" << cameraAutoexposure1;
    fs << "cameraAutoexposure2" << cameraAutoexposure2;
    fs << "cameraExposure1" << cameraExposure1;
    fs << "cameraExposure2" << cameraExposure2;
    fs << "cameraGain1" << cameraGain1;
    fs << "cameraGain2" << cameraGain2;
    fs << "trackerCalibCenters" << trackerCalibCenters;
    fs << "depthSmoothing" << depthSmoothing;
    fs << "additionalSmoothing" << additionalSmoothing;
    fs << "markerLibrary" << markerLibrary;
    fs << "languageSelection" << languageSelection;
    fs << "calibScale" << calibScale;
    fs << "ipcAddr" << ipcAddr;

    fs << "trackers";
    fs << "{";
    for (int i = 0; i < trackers.size(); i++)
    {
        fs << "tracker_" + std::to_string(i);
        fs << "{";
        fs << "trackerIds";
        fs << trackers[i]->ids;
        fs << "trackerCorners";
        fs << "{";
        for (int j = 0; j < trackers[i]->objPoints.size(); j++)
        {
            fs << "trackerCorners_" + std::to_string(j);
            fs << trackers[i]->objPoints[j];
        }
        fs << "}";
        fs << "}";
    }
    fs << "}";
    language = Lang();

    if (languageSelection == 1)
        language = get_lang_chinese();
}
