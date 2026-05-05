// *****************************************************************************
//
//      Copyright (c) 2026, Teledyne e2v., All rights reserved.
//
//
// TELEDYNE MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. TELEDYNE SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//
// *****************************************************************************

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>


#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "colormap.hpp"

#include <fstream>
#include <string>
#include <list>
#include <vector>
#include <thread>
#include <atomic>
#include <filesystem>

#include "NodeMapInfo.h"
#include "FeatureControl.h"

// application global parameters:
constexpr int roi_scale = 2; // display scale factor: increase depth image size | adapt demosaic image to have the same size
constexpr bool dense_depth = FALSE; 
constexpr int roi_size=30; // size of depth ROI in pixel

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace cv;
using namespace std;

// JET color map with a black at the very end to represent invalid values
static const cv::Mat COLORMAP = cv::Mat(256, 1, CV_8UC3, (void*)&JET_MAP_8U);

bool WriteRawImage(const std::string& filePath, const void* data, size_t dataSize);
std::vector<std::string> GetInput(const std::string& strText);


enum class PointOfCloudFormat : uint8_t {
    Ascii = 0,
    Binary = 1
};

// TODO: validate this function
template <typename T>
void WritePointCloudPLY(const std::string& filename, const T* data, size_t dataSize, float fInvalidValue, PointOfCloudFormat ePointOfCloudFormat) {
    std::ofstream fstream(filename,
        (ePointOfCloudFormat == PointOfCloudFormat::Binary) ? std::ios::binary : std::ios::trunc);
    if (!fstream.is_open())
    {
        std::cout << "Error: Could not open file " << filename << " for writing" << std::endl;
        return;
    }

    if (dataSize % (3 * sizeof(T))) {
        std::cout << "Error: Invalid DataSize" << std::endl;
        return;
    }

    const int fractionalBits = 4;
    const double fixedPointScale = static_cast<double>(1 << fractionalBits); // 2^4 = 16

    // Count valid vertices (z != fInvalidValue)
    size_t validCount = 0;

    if (sizeof(T) == sizeof(int16_t)) {
        const int16_t* ps16Data = (const int16_t*)data;
        for (size_t i = 0; i < dataSize / (3 * sizeof(T)); i++) {
            float z = static_cast<float>(ps16Data[2]) / static_cast<float>(fixedPointScale);
            if (z != fInvalidValue) {
                validCount++;
            }

            ps16Data += 3;
        }
    }
    else if (sizeof(T) == sizeof(float)) {
        const float* pf32Data = (const float*)data;
        for (size_t i = 0; i < dataSize / (3 * sizeof(T)); i++) {
            if (pf32Data[2] != fInvalidValue) {
                validCount++;
            }

            pf32Data += 3;
        }
    }
    else {
        std::cout << "Error: Unsupported type" << std::endl;
        return;
    }

    // Header
    fstream << "ply\n";

    if (ePointOfCloudFormat == PointOfCloudFormat::Binary) {
        fstream << "format binary_little_endian 1.0\n";
    }
    else {
        fstream << "format ascii 1.0\n";
    }
    fstream << "comment PLY file\n";
    fstream << "element vertex " << validCount << "\n";
    fstream << "property float x\n";
    fstream << "property float y\n";
    fstream << "property float z\n";
    fstream << "end_header\n";

    // Write vertex data
    if (sizeof(T) == sizeof(int16_t)) {
        const int16_t* ps16Data = (const int16_t*)data;
        for (size_t i = 0; i < dataSize / (3 * sizeof(T)); i++) {

            // Convert from Q12.4 fixed-point back to float
            float x = static_cast<float>(ps16Data[0]) / static_cast<float>(fixedPointScale);
            float y = static_cast<float>(ps16Data[1]) / static_cast<float>(fixedPointScale);
            float z = static_cast<float>(ps16Data[2]) / static_cast<float>(fixedPointScale);

            if (z != fInvalidValue) {
                if (ePointOfCloudFormat == PointOfCloudFormat::Binary) {
                    fstream.write(reinterpret_cast<const char*>(&x), sizeof(float));
                    fstream.write(reinterpret_cast<const char*>(&y), sizeof(float));
                    fstream.write(reinterpret_cast<const char*>(&z), sizeof(float));
                }
                else {
                    fstream << x << " " << y << " " << z << std::endl;
                }
            }

            ps16Data += 3;
        }
    }
    else if (sizeof(T) == sizeof(float)) {
        const float* pf32Data = (const float*)data;
        for (size_t i = 0; i < dataSize / (3 * sizeof(T)); i++) {

            if (pf32Data[2] != fInvalidValue) {
                if (ePointOfCloudFormat == PointOfCloudFormat::Binary) {
                    fstream.write(reinterpret_cast<const char*>(&pf32Data[0]), sizeof(float) * 3);
                }
                else {
                    fstream << pf32Data[0] << " " << pf32Data[1] << " " << pf32Data[2] << std::endl;
                }
            }

            pf32Data += 3;
        }
    }

    fstream.close();
    std::cout << "Saved: " << filename << " (" << validCount << " points)" << std::endl;
}


// Use the following enum to select the stream mode
enum StreamMode
{
    STREAM_MODE_TELEDYNE_GIGE_VISION, // Teledyne Gige Vision is the default stream mode for spinview which is supported
    // on Windows
    STREAM_MODE_PGRLWF,               // Light Weight Filter driver is our legacy driver which is supported on Windows
    STREAM_MODE_SOCKET, // Socket is supported for MacOS and Linux, and uses native OS network sockets instead of a
    // filter driver
};

#if defined(WIN32) || defined(WIN64)
const StreamMode chosenStreamMode = STREAM_MODE_TELEDYNE_GIGE_VISION;
#else
const StreamMode chosenStreamMode = STREAM_MODE_SOCKET;
#endif

// Disables or enables heartbeat on GEV cameras so debugging does not incur timeout errors
int ConfigureGVCPHeartbeat(CameraPtr pCam, bool enableHeartbeat)
{
    //
    // Write to boolean node controlling the camera's heartbeat
    //
    // *** NOTES ***
    // This applies only to GEV cameras.
    //
    // GEV cameras have a heartbeat built in, but when debugging applications the
    // camera may time out due to its heartbeat. Disabling the heartbeat prevents
    // this timeout from occurring, enabling us to continue with any necessary
    // debugging.
    //
    // *** LATER ***
    // Make sure that the heartbeat is reset upon completion of the debugging.
    // If the application is terminated unexpectedly, the camera may not locked
    // to Spinnaker indefinitely due to the the timeout being disabled.  When that
    // happens, a camera power cycle will reset the heartbeat to its default setting.

    // Retrieve TL device nodemap
    INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

    // Retrieve GenICam nodemap
    INodeMap& nodeMap = pCam->GetNodeMap();

    CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
    if (!IsReadable(ptrDeviceType))
    {
        return -1;
    }

    if (ptrDeviceType->GetIntValue() != DeviceType_GigEVision)
    {
        return 0;
    }

    if (enableHeartbeat)
    {
        std::cout << std::endl << "Resetting heartbeat..." << std::endl << std::endl;
    }
    else
    {
        std::cout << std::endl << "Disabling heartbeat..." << std::endl << std::endl;
    }

    CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
    if (!IsWritable(ptrDeviceHeartbeat))
    {
        std::cout << "Unable to configure heartbeat. Continuing with execution as this may be non-fatal..." << std::endl << std::endl;
    }
    else
    {
        ptrDeviceHeartbeat->SetValue(!enableHeartbeat);

        if (!enableHeartbeat)
        {
            std::cout << "WARNING: Heartbeat has been disabled for the rest of this example run." << std::endl;
            std::cout << "         Heartbeat will be reset upon the completion of this run.  If the " << std::endl;
            std::cout << "         example is aborted unexpectedly before the heartbeat is reset, the" << std::endl;
            std::cout << "         camera may need to be power cycled to reset the heartbeat." << std::endl << std::endl;
        }
        else
        {
            std::cout << "Heartbeat has been reset." << std::endl;
        }
    }

    return 0;
}

int ResetGVCPHeartbeat(CameraPtr pCam)
{
    return ConfigureGVCPHeartbeat(pCam, true);
}

int DisableGVCPHeartbeat(CameraPtr pCam)
{
    return ConfigureGVCPHeartbeat(pCam, false);
}

// This function demonstrates how we can change stream modes.
int SetStreamMode(CameraPtr pCam)
{
    int result = 0;

    // Retrieve Stream nodemap
    const INodeMap& sNodeMap = pCam->GetTLStreamNodeMap();

    // The node "StreamMode" is only available for GEV cameras.
    // Skip setting stream mode if the node is inaccessible.
    const CEnumerationPtr ptrStreamMode = sNodeMap.GetNode("StreamMode");
    if (!IsReadable(ptrStreamMode) || !IsWritable(ptrStreamMode))
    {
        return 0;
    }

    gcstring streamMode;
    switch (chosenStreamMode)
    {
    case STREAM_MODE_PGRLWF:
        streamMode = "LWF";
        break;
    case STREAM_MODE_SOCKET:
        streamMode = "Socket";
        break;
    case STREAM_MODE_TELEDYNE_GIGE_VISION:
    default:
        streamMode = "TeledyneGigeVision";
    }

    // Retrieve the desired entry node from the enumeration node
    const CEnumEntryPtr ptrStreamModeCustom = ptrStreamMode->GetEntryByName(streamMode);
    if (!IsReadable(ptrStreamModeCustom))
    {
        // Failed to get custom node
        std::cout << "Stream mode " + streamMode + " not available.  Aborting..." << std::endl;
        return -1;
    }
    // Retrieve the integer value from the entry node
    const int64_t streamModeCustom = ptrStreamModeCustom->GetValue();

    // Set integer as new value for enumeration node
    ptrStreamMode->SetIntValue(streamModeCustom);

    // Print out the current stream mode
    std::cout << std::endl << "Stream Mode set to " + ptrStreamMode->GetCurrentEntry()->GetSymbolic() << "..." << std::endl;

    return 0;
}

// Camera control menu
bool bControl = true;
bool bParam = true;

std::vector<std::string> AnalgGain = { "x1", "x1___2", "x1___5", "x1___7", "x2", "x2___4", "x3", "x4", "x4___8", "x6", "x6___9", "x8", "x9___6", "x12", "x16" };
void change_control(int value, void*){bControl = true;}
void change_param(int value, void*){bParam = true;}



// ROI selection management
cv::Rect roi(480/2-roi_size/2, 270/2-roi_size/2, roi_size, roi_size);
int bRoiMove=false;
cv::Scalar roi_color = cv::Scalar(0, 255, 0);  //green

void mouseCallback3D(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN) {
        cout << "Left button clicked at (" << x << ", " << y << ")" << endl;
        if(bRoiMove==false){

            if (x >= roi.x*roi_scale && x <= (roi.x + roi.width)*roi_scale && y >= roi.y*roi_scale && y <= (roi.y + roi.height)*roi_scale){
                bRoiMove = true;
                // selRoi = 1;
                roi_color = cv::Scalar(0, 0, 255);
            }        
        }
        else{
            cout << "ROI 1 - new center: (" << x << ", " << y << ")" << endl;
            //change  ROI 1
            roi.x = (x/roi_scale - roi.width/2);
            roi.y = (y/roi_scale - roi.height/2);
            roi_color = cv::Scalar(0, 255, 0);
            cout << "ROI 1 : (" << roi.x << ", " << roi.y << ", " << roi.width << ", " << roi.height << ")" << endl;

            bRoiMove = false;
        }
    }

    // if (event == EVENT_RBUTTONDOWN) {
    //     cout << "Right button clicked at (" << x << ", " << y << ")" << endl;
    // }
    // if (event == EVENT_MOUSEMOVE) {
    //     // cout << "Mouse moved to (" << x << ", " << y << ")" << endl;

}


int AcquireImages(CameraPtr pCam, INodeMap& nodeMap, INodeMap& nodeMapTLDevice, std::atomic<bool>& bGrab, std::atomic<bool>& bQuit, const std::string& strSaveFolder, std::atomic<float>& depth) {
    int result = 0;
    int k;
    //cout << endl << endl << "*** IMAGE ACQUISITION ***" << endl << endl;

    // init parameters
    int exposure_value=5000;
    int again_value=0;
    int fps_value=15;
    int robotSeq_value = 0;
    int measMode_value = 0;
    int backDist_value = 1000;
    int depth_th_value = 0;  // threshold for object detection
    int maxDepth_value = 2550;
    float maxDepth, depth_th; 

    try {
        // Retrieve enumeration node from nodemap
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsReadable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode)) {
            //cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
            bGrab = false;
            return -1;
        }

        // Retrieve entry node from enumeration node
        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsReadable(ptrAcquisitionModeContinuous))
        {
            //cout << "Unable to get or set acquisition mode to continuous (entry retrieval). Aborting..." << endl
            //    << endl;
            bGrab = false;
            return -1;
        }

        // Retrieve integer value from entry node
        const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

        // Set integer value from entry node as new value of enumeration node
        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

        //cout << "Acquisition mode set to continuous..." << endl;

        //config before streaming
        WriteFeatureValue(pCam, "FrameRate", std::to_string(15));
        if(dense_depth) WriteFeatureValue(pCam, "RangeMode", "DenseDepth");
        else WriteFeatureValue(pCam, "RangeMode", "SparseDepth");

        // Begin acquiring images
        pCam->BeginAcquisition();

        //cout << "Acquiring images..." << endl;

        uint64_t timeoutInMilliSecs = 2000;

        const float fPointOfCloudInvalidValue = -1.0f; 

        // create 3D image window out of the loop to add controls
        cv::namedWindow("3D image");
        cv::createTrackbar("Max Depth", "3D image", &maxDepth_value, 2550, change_param);
        cv::createTrackbar("Mode", "3D image", &measMode_value, 1, change_param);
        cv::createTrackbar("Depth th", "3D image", &depth_th_value, 2500, change_param);
        cv::createTrackbar("Back dist", "3D image", &backDist_value, 2500, change_param);       
        // cv::createTrackbar("Confidence threshold", "3D image", &confidence_value, 250, change_control);
        setMouseCallback("3D image", mouseCallback3D, nullptr);

        // create 2D image window out of the loop to add controls
        cv::namedWindow("2D image");
        cv::createTrackbar("Exposure", "2D image", &exposure_value, 100000, change_control);
        cv::createTrackbar("Again", "2D image", &again_value, 14, change_control);
        cv::createTrackbar("Fps", "2D image", &fps_value, 15, change_control);

        // Acquire images until the user instructs us to stop.
        while (bGrab) {

            // Adjust camera parameter
            if(bControl)
            {
                WriteFeatureValue(pCam, "Exposure", std::to_string(exposure_value));
                WriteFeatureValue(pCam, "AnalogGain", AnalgGain[again_value]);
                // confidence_th=confidence_value/100000.0;
                // WriteFeatureValue(pCam, "DisparityH_ConfidenceThreshold", std::to_string(confidence_th));
                // WriteFeatureValue(pCam, "DisparityV_ConfidenceThreshold", std::to_string(confidence_th));
                WriteFeatureValue(pCam, "FrameRate", std::to_string(fps_value));
                bControl = false;
            }

            if(bParam)
            {
                depth_th = depth_th_value / 10.0;
                maxDepth = maxDepth_value / 10.0;

                bParam = false;
            }

            try {
                ImageList imageList;

                imageList = pCam->GetNextImageSync(timeoutInMilliSecs);

                size_t unValue;
                PixelFormatEnums ePixelFormat;

                for (size_t unIndex = 0; unIndex < imageList.GetSize(); unIndex++) {
                    ImagePtr img = imageList.GetByIndex(unIndex);

                    if (img->IsIncomplete()) {
                        // Retrieve and print the image status description
                        std::cout << "Image incomplete: " << Image::GetImageStatusDescription(img->GetImageStatus()) << "..." << std::endl;
                    }
                    else {
                        size_t unWidth = img->GetWidth();
                        size_t unHeight = img->GetHeight();
                        size_t unSize = img->GetImageSize();

                        switch (img->GetPixelFormat()) {
                        case PixelFormat_Mono8: {

                            // std::cout << "u32Component="  << u32Component << " u32Part="  << u32Part << " PixelType=PvPixelMono8" << std::endl;
                            if (!strSaveFolder.empty())
                                WriteRawImage(strSaveFolder + "2DImage.raw", img->GetData(), unSize);

                            cv::Mat mat((int)unHeight, (int)unWidth, CV_8UC1, img->GetData());
                            cv::Mat color;
                            cv::cvtColor(mat, color, cv::COLOR_GRAY2BGR);

                            const uint32_t MAX_IMAGE_WIDTH = 1920;
                            const uint32_t MAX_IMAGE_HEIGHT = 1080;

                            if ((unWidth == MAX_IMAGE_WIDTH) && (unHeight == MAX_IMAGE_HEIGHT)) {
                                cv::Mat draw;
                                cv::resize(color, draw, cv::Size(color.size().width * roi_scale/ 4, color.size().height * roi_scale/ 4));
                                cv::imshow("2D image", draw);
                            }
                            else { // w=480  h=270  (binning x4)
                                cv::imshow("2D image", color);
                            }
                            // Add some controls
                            k = cv::waitKey(1);
                            if(k == 'q' || k == 27)
                            {
                                std::cout << "Live preview stop" << std::endl;
                                bGrab = false;
                                bQuit = true;
                            }
                            break;
                        }
                        case PixelFormat_Coord3D_C16: {
                            if (!strSaveFolder.empty())
                                WriteRawImage(strSaveFolder + "3Ddepth16.raw", img->GetData(), unSize);

                            cv::Mat mat((int)unHeight, (int)unWidth, CV_16UC1, img->GetData());
                            cv::Mat rescaledGray;
                            cv::Mat colorMat;

                            cv::Mat matFloat;
                            const int fractionalBits = 4;
                            const double fixedPointScale = static_cast<double>(1 << fractionalBits); // 2^4 = 16
                            mat.convertTo(matFloat, CV_32FC1, 1.0 / fixedPointScale);

                            // rescaled [0:250] cm to [0:255] for color map
                            matFloat.convertTo(rescaledGray, CV_8UC1, 255.f / 250.f);
                            cv::applyColorMap(rescaledGray, colorMat, COLORMAP);
                            cv::cvtColor(colorMat, colorMat, cv::COLOR_RGB2BGR);

                            // display image
                            cv::imshow("3D sparse depth 16bit", colorMat);
                            cv::waitKey(1);

                            break;
                        }
                        case PixelFormat_Coord3D_C32f: { // this mode by default for depth image
                            if (!strSaveFolder.empty())
                                WriteRawImage(strSaveFolder + "3dDepth32f.raw", img->GetData(), unSize);

                            cv::Mat mat((int)unHeight, (int)unWidth, CV_32FC1, img->GetData());

                            // distance calculation into ROI
                            cv::Mat depth_cropped = mat(roi);
                            float m = cv::mean(depth_cropped, depth_cropped <maxDepth)[0];
                            // std::cout << "Mean: " << m << "\n";

                            if(measMode_value == 1) {
                                depth = (backDist_value/10.0) - m;
                            }
                            else {
                                depth = m;
                            } 
                            
                            cv::Mat rescaledGray;
                            cv::Mat colorMat;
                            // rescaled [0:250] cm to [0:255] for color map
                            mat.convertTo(rescaledGray, CV_8UC1, 255.f / maxDepth);
                            cv::applyColorMap(rescaledGray, colorMat, COLORMAP);
                            cv::cvtColor(colorMat, colorMat, cv::COLOR_RGB2BGR);

                            // Increase depth image size
                            cv::resize(colorMat, colorMat, cv::Size(mat.size().width * roi_scale, mat.size().height * roi_scale));

                            // define ROI rectangle color
                            if(bRoiMove==false)
                            {
                                
                                if(measMode_value == 0) 
                                {
                                    if((depth > 0) && (depth < depth_th)) roi_color = cv::Scalar(0, 165, 255); // object detected: orange
                                    else roi_color = cv::Scalar(0, 255, 0); // default: green
                                }
                                else
                                {
                                    if((depth > 0) && (depth > depth_th)) roi_color = cv::Scalar(0, 165, 255); // object detected: orange
                                    else roi_color = cv::Scalar(0, 255, 0); // default: green
                                }
                            }

                            // display window overlay
                            cv::Point topLeft(roi.x * roi_scale, roi.y * roi_scale);
                            cv::Point bottomRight((roi.x + roi.width)* roi_scale, (roi.y + roi.height)* roi_scale);
                            cv::rectangle(colorMat, topLeft, bottomRight, roi_color, 2);
                                                
                            // Add text above the rectangle
                            std::ostringstream oss;
                            if(measMode_value == 1) {oss << "Height: " << std::fixed << std::setprecision(1) << depth << "cm";}
                            else{oss << "Depth: " << std::fixed << std::setprecision(1) << depth << "cm";}
                            std::string label = oss.str();
                            cv::Point textPos((topLeft.x)-30 , (topLeft.y) - 10);  // slightly above the rectangle
                            cv::putText(
                                colorMat,
                                label,
                                textPos,
                                cv::FONT_HERSHEY_SIMPLEX,
                                0.7,        // font scale
                                roi_color,  // green text
                                2           // thickness
                            );
                          
                            cv::imshow("3D image", colorMat);
                            cv::waitKey(1);
                            break;
                        }
                        case PixelFormat_Coord3D_ABC16: {
                            if (!strSaveFolder.empty()) {
                                WriteRawImage(strSaveFolder + "PointOfCloud16bit.raw", img->GetData(), unSize);

                                WritePointCloudPLY<int16_t>(strSaveFolder + "PointOfCloud16bit.ply",
                                    reinterpret_cast<const int16_t*>(img->GetData()),
                                    unSize,
                                    fPointOfCloudInvalidValue,
                                    PointOfCloudFormat::Ascii);
                            }

                            break;
                        }
                        case PixelFormat_Coord3D_ABC32f: {
                            if (!strSaveFolder.empty()) {
                                WriteRawImage(strSaveFolder + "PointOfCloud32bit.raw", img->GetData(), unSize);

                                WritePointCloudPLY<float>(strSaveFolder + "PointOfCloud32bit.ply",
                                    reinterpret_cast<const float*>(img->GetData()),
                                    unSize,
                                    fPointOfCloudInvalidValue,
                                    PointOfCloudFormat::Ascii);
                            }

                            break;
                        }
                        default:
                            // Unknown format;
                            break;
                        }
                        // Release image
                        img->Release();
                    }
                }                      
            }
            catch (Spinnaker::Exception& e)
            {
                std::cout << "Spinnaker exception1 AcquireImages: " << e.what() << std::endl;
                result = -1;
                bGrab = false;
            }
        }
        // End acquisition
        pCam->EndAcquisition();
    }
    catch (Spinnaker::Exception& e) {
        std::cout << "Spinnaker exception2 AcquireImages: " << e.what() << std::endl;
        result = -1;
        bGrab = false;
    }

    return result;
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap& nodeMap) {
    int result = 0;
    std::cout << std::endl << "*** DEVICE INFORMATION ***" << std::endl << std::endl;

    try {
        FeatureList_t features;
        CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsReadable(category)) {
            category->GetFeatures(features);

            FeatureList_t::const_iterator it;
            for (it = features.begin(); it != features.end(); ++it) {
                try {
                    CNodePtr pfeatureNode = *it;
                    std::cout << pfeatureNode->GetName() << " : ";
                    CValuePtr pValue = (CValuePtr)pfeatureNode;
                    std::cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                    std::cout << std::endl;
                }
                catch (Spinnaker::Exception) {
                    std::cout << "Node not readable" << std::endl;
                }
            }
        }
        else {
            std::cout << "Device control information not readable." << std::endl;
        }
    }
    catch (Spinnaker::Exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        result = -1;
    }

    return result;
}

// This function acts as the body of the example; please see NodeMapInfo example
// for more in-depth comments on setting up cameras.
int RunSingleCamera(CameraPtr pCam, const std::string& strSaveFolder)
{
    int result;

    try
    {
        // Retrieve TL device nodemap and print device information
        INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        result = PrintDeviceInfo(nodeMapTLDevice);
        if (result != 0)
            return result;

        // Initialize camera
        pCam->Init();

        // Retrieve GenICam nodemap
        INodeMap& nodeMap = pCam->GetNodeMap();

        // Configure heartbeat for GEV camera
#ifdef _DEBUG
        result = DisableGVCPHeartbeat(pCam);
#else
        result = ResetGVCPHeartbeat(pCam);
#endif

        if (result != 0)
            return result;

        // Set stream mode
        result = SetStreamMode(pCam);
        if (result != 0)
            return result;

        std::atomic<bool> bGrab = false;
        std::atomic<bool> bQuit = false;
        std::atomic<float> depth = 0;

        // Start video stream
        bGrab = true;


        while ((!bQuit) && (result == 0)) {
            if (bGrab) {
                // Acquire images
                result = AcquireImages(pCam, nodeMap, nodeMapTLDevice, std::ref(bGrab), std::ref(bQuit), strSaveFolder, std::ref(depth));
                if (result != 0) {
                    std::cout << "Error AcquireImages" << std::endl;
                }
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }

#ifdef _DEBUG
        // Reset heartbeat for GEV camera
        result = result | ResetGVCPHeartbeat(pCam);
#endif

        // Deinitialize camera
        pCam->DeInit();
    }
    catch (Spinnaker::Exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        result = -1;
    }

    return result;
}

// Main entry point: camera detection
// starts video stream and video loop: RunSingleCamera
// graphic operation and image processing in: AcquireImages
int main(int argc, char** argv) {
    std::string strSaveFolder;
    if (argc >= 2) {
        strSaveFolder = std::string(argv[1]);

        namespace fs = std::filesystem;
        if (fs::exists(strSaveFolder) && fs::is_directory(strSaveFolder)) {
            if (!strSaveFolder.empty() &&
                (strSaveFolder.back() != '/') &&
                (strSaveFolder.back() != '\\')) {
                strSaveFolder += "/";
            }
        }
        else {
            strSaveFolder.clear();
        }
    }

    // Print application build information
    std::cout << "Application build date: " << __DATE__ << " " << __TIME__ << std::endl << std::endl;

    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();

    // Print out current library version
    const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    std::cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
        << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << std::endl
        << std::endl;

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();

    const unsigned int numCameras = camList.GetSize();

    std::cout << "Number of cameras detected: " << numCameras << std::endl << std::endl;

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        std::cout << "No cameras detected!" << std::endl;
        std::cout << "Done! Press Enter to exit..." << std::endl;
        // std::getchar();

        return -1;
    }

    // Create shared pointer to camera
    CameraPtr pCam = nullptr;

   

   // Select camera
   pCam = camList.GetByIndex(0);

   std::cout << std::endl << "Running example for camera" << std::endl;

   // Run example
   int result = 0;
   result = RunSingleCamera(pCam, strSaveFolder);

   std::cout << "Camera example complete result=" << result << std::endl << std::endl;

    // Release reference to the camera
    pCam = nullptr;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();

    std::cout << std::endl << "Done! Press Enter to exit..." << std::endl;
    // std::getchar();

    return result;
}

bool WriteRawImage(const std::string& filePath, const void* data, size_t dataSize) {
    if (!data) {
        std::cerr << "Error null data pointer." << std::endl;
        return false;
    }

    // Open file in binary mode
    std::ofstream outFile(filePath, std::ios::binary);
    if (!outFile) {
        std::cerr << "Error: opening file:" << filePath << std::endl;
        return false;
    }

    // Write raw data
    outFile.write(reinterpret_cast<const char*>(data), dataSize);

    outFile.close();
    return true;
}

std::vector<std::string> GetInput(const std::string &strText) {
    std::vector<std::string> VectResult;
    std::cout << strText;
    std::string strInput;
    std::getline(std::cin, strInput);

    std::istringstream iss(strInput);
    std::string strCmd;

    while (iss >> strCmd) {
        VectResult.push_back(strCmd);
    }

    return VectResult;
}