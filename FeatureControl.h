#pragma once

#include "Spinnaker.h"

bool WriteFeatureValue(Spinnaker::CameraPtr pCam, const std::string& strName, const std::string& strValue);
bool ReadFeatureValue(Spinnaker::CameraPtr pCam, const std::string& strName, std::string& strValue);

