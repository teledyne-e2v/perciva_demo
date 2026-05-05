# perciva_demo
Demo software to run Perciva 5D camera connected to a Windows PC

# Requirements

Software tested on PC with Windows 11 and following installation using Visual Studio Community 2026 (Desktop development with C++) :
* Windows 11 SDK
* MSVC v18.x build tools
* C++ CMake tools for Windows

Note that it works also with MSVC v17.x (Visual Studio Community 2022)

Install openCV for windows in `C:` drive and check if *.lib binaries and *.cmake files are available here: `C:/opencv/build/x64/vc16/lib`. Modify the CMakeLists.txt if needed.

**IMPORTANT:** Add the link to openCV in the environment variable PATH: `C:/opencv/build/x64/vc16/bin`

Install Teledyne Spinnaker SDK on the PC. Modify the `CMakeLists.txt` with the correct path if needed.

To build this project and compile it, please use CMake into VSCode for example

# User Guide

This application open two display windows showing depth map and demosaic image from Perciva camera.
In depth image display, a ROI caculates an display the mean distance value.
This ROI can be moved by click inside then click in the window to define a new center.
The ROI size can be adapted changing in the code the constant:  `roi_size`

`ESC`: Quit the application

## 3D image display:

This window displays the depth image from the camera. Some paraters are avaible to adjust the application settings.
By default the **Sparse depth** image is displayed but the **Dense depth** image can also be used changing in the code the constant:  `dense_depth = FALSE`

The maximum depth of the application can be modified with the parameter `Max Depth` (integer value in mm). The color displayed values and the color map are adjusted with this parameter.

For the ROI calculation two modes are available:

`Mode=0`: the calculated distance is the **depth** from the camera to the objetc. A paratemer can be adjusted to detect the presence of an object approaching the camera: the ROI rectangle turns orange when the depth is smaller than `Depth th` (integer value in mm)

`Mode=1`: the calculated distance is the **height** of the objetc: a subratction of the averaged value inside the ROI and a background distance defined by the parameter `Back dist`(integer value in mm). The `Depth th` parameter is still valid and defines the max height of the object to detect.

## 2D image display:

This window displays the demosaic image from the camera. Some camera controls are avalable.

`Exposure`: Exposure time of the sensor (integer value in us)

`Again`: Analog gain of the sensor from x1 to x16

`Fps`: frame rate in fps. Useful in case of flickering in the image.