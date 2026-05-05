
#include <iostream>
#include <sstream>
#include <string>

#include "FeatureControl.h"

using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;


bool WriteFeatureValue(Spinnaker::CameraPtr pCam, const std::string& strName, const std::string& strValue) {
    try {
        INodeMap& nodeMap = pCam->GetNodeMap();

        INode* pNode = nodeMap.GetNode(strName.c_str());
        if (!pNode) {
            std::cout << "WriteFeatureValue Unknown feature name: " << strName << std::endl;
            return false;
        }

        CNodePtr pFeatureNode = pNode;
        CValuePtr pValue = (CValuePtr)pFeatureNode;
        if (IsWritable(pValue)) {
            pValue->FromString(strValue.c_str());
            return true;
        }
        else {
            std::cout << "WriteFeatureValue feature name: " << strName << " is not writable" << std::endl;
            return false;
        }
    }
    catch (Spinnaker::Exception& e) {
        std::cout << "Spinnaker exception WriteFeatureValue: " << e.what() << std::endl;
    }
    catch (...) {
        std::cout << "Unknown exception WriteFeatureValue" << std::endl;
    }

    return false;
}

bool ReadFeatureValue(Spinnaker::CameraPtr pCam, const std::string& strName, std::string& strValue) {
    try {
        INodeMap& nodeMap = pCam->GetNodeMap();

        INode* pNode = nodeMap.GetNode(strName.c_str());
        if (!pNode) {
            std::cout << "ReadFeatureValue Unknown feature name: " << strName << std::endl;
            return false;
        }

        CNodePtr pFeatureNode = pNode;
        CValuePtr pValue = (CValuePtr)pFeatureNode;
        if (IsReadable(pValue)) {
            gcstring strVal;
            strVal = pValue->ToString();
            strValue = strVal.c_str();
            return true;
        }
        else {
            std::cout << "ReadFeatureValue feature name: " << strName << " is not readable" << std::endl;
            return false;
        }
    }
    catch (Spinnaker::Exception& e) {
        std::cout << "Spinnaker exception ReadFeatureValue: " << e.what() << std::endl;
    }
    catch (...) {
        std::cout << "Unknown exception ReadFeatureValue" << std::endl;
    }

    return false;
}
