/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Kelleher Guerin, Simon Leonard
  Created on: 2011

  (C) Copyright 2011 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include <sawOpenNI/osaOpenNI.h>
#include <cisstVector/vctDynamicMatrix.h>
#include <cisstVector/vctDynamicNArray.h>

#include "osaOpenNIData.h"


osaOpenNI::osaOpenNI(int numUsers) :
    ProjectivePointsBuffer(0),
    WorldPointsBuffer(0),
    PointsBufferSize(0)
{
    this->Data = new osaOpenNIData;
    this->Data->SetStates();
    this->users = numUsers;
    this->usingPrecapturedCalibration = false;
}


osaOpenNI::osaOpenNI(int numUsers, char usrPath) :
    ProjectivePointsBuffer(0),
    WorldPointsBuffer(0),
    PointsBufferSize(0)
{
    this->Data = new osaOpenNIData;
    this->Data->SetStates();
    this->users = numUsers;

    this->usingPrecapturedCalibration = true;
    this->Data->usingPrecapCalib = true;
    XnChar* cpath = new XnChar(usrPath);
    this->Data->userCalibPath = cpath;
}


osaOpenNI::~osaOpenNI()
{
    if (this->Data){
        CleanupExit();
        delete this->Data;
        this->Data = 0;    
    }
    if (ProjectivePointsBuffer) delete ProjectivePointsBuffer;
    if (WorldPointsBuffer)      delete WorldPointsBuffer;
}


void osaOpenNI::CleanupExit(void)
{
    if(this->Data) this->Data->context.Shutdown();
}

void osaOpenNI::Configure (const std::string & fname)
{
    // Store XML Config File
    this->name = fname;

    // Errors
    xn::EnumerationErrors errors;


    // Init
    XnStatus status = Data->context.InitFromXmlFile (this->name.data(), &errors); 

    if (status == XN_STATUS_NO_NODE_PRESENT) {
        XnChar strError[1024];
        errors.ToString (strError, 1024);
        CMN_LOG_RUN_ERROR << strError << std::endl;
        return;
    }

    else if (status != XN_STATUS_OK) {
        CMN_LOG_RUN_ERROR << "Failed to open " << this->name << " "
                          << xnGetStatusString(status) << std::endl;
        return;
    }

    status = Data->context.FindExistingNode(XN_NODE_TYPE_DEPTH, Data->depthgenerator);
    if (status != XN_STATUS_OK) {
        CMN_LOG_RUN_ERROR << "Failed to find depth node "
                          << xnGetStatusString(status) << std::endl;
        return;
    }

    status = Data->context.FindExistingNode(XN_NODE_TYPE_IMAGE, Data->rgbgenerator);
    if (status != XN_STATUS_OK) {
        CMN_LOG_RUN_ERROR << "Failed to find RGB node "
                          << xnGetStatusString(status) << std::endl;
        return;
    }

    // register the depth image with the rgb image
    status=Data->depthgenerator.GetAlternativeViewPointCap().SetViewPoint(Data->rgbgenerator);
    if (status != XN_STATUS_OK) {
        CMN_LOG_RUN_ERROR << "Failed to set viewpoint: "
                          << xnGetStatusString(status) << std::endl;
        return;
    }

    status = Data->context.FindExistingNode(XN_NODE_TYPE_USER, Data->usergenerator);
    if (status != XN_STATUS_OK) {
        status = Data->usergenerator.Create(Data->context);
    }

    XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;
    
    if(this->usingPrecapturedCalibration){
        // Dont register callbacks...
        Data->usergenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
    }else{


        if (!Data->usergenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
            CMN_LOG_RUN_ERROR << "Supplied user generator doesn't support skeleton"
                              << std::endl;
        }

        else{

            Data->usergenerator.RegisterUserCallbacks(User_NewUser,
                                                  User_LostUser,
                                                  this->Data,
                                                  hUserCallbacks);

            Data->usergenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart,
                                                                          UserCalibration_CalibrationEnd,
                                                                          this->Data,
                                                                          hCalibrationCallbacks);

            if (Data->usergenerator.GetSkeletonCap().NeedPoseForCalibration()) {
                Data->needPose = TRUE;

                if (!Data->usergenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
                    CMN_LOG_RUN_ERROR << "Pose required, but not supported" << std::endl;
                }

                Data->usergenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected,
                                                                              NULL,
                                                                              this->Data,
                                                                              hPoseCallbacks);
                Data->usergenerator.GetSkeletonCap().GetCalibrationPose(Data->strPose);
            }

            Data->usergenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

        }

    }

    // Start Generating
    Data->context.StartGeneratingAll();

}

void osaOpenNI::Update(int type) {
    // Query the context based on type
    if(type == WAIT_NONE_UPDATE_ALL)
        Data->context.WaitNoneUpdateAll();
    else if(type == WAIT_AND_UPDATE_ALL)
        Data->context.WaitAndUpdateAll();
    else if(type == WAIT_ANY_UPDATE_ONE)
        Data->context.WaitAnyUpdateAll();
}

void osaOpenNI::InitSkeletons() {

    osaOpenNISkeleton* skeleton;
    for(int i = 0; i<this->users; i++) {
        skeleton = new osaOpenNISkeleton(this);
        skeletons.push_back(skeleton);
        std::cout<<skeletons.size()<<std::endl;
    }

}

osaOpenNI::Errno osaOpenNI::GetRGBPlanarImage(vctDynamicNArray<unsigned char,3>& RGBimage) {

    xn::ImageMetaData rgbMD;
    Data->rgbgenerator.GetMetaData(rgbMD);

    RGBimage.SetSize(vctDynamicNArray<unsigned char,3>::nsize_type(rgbMD.YRes(), rgbMD.XRes(), 3));
    memcpy(RGBimage.Pointer(), rgbMD.Data(), rgbMD.YRes()*rgbMD.XRes()*3*sizeof(unsigned char));

    return osaOpenNI::ESUCCESS;

}

osaOpenNI::Errno osaOpenNI::GetRGBImage(vctDynamicMatrix<unsigned char>& RGBimage) {

    xn::ImageMetaData rgbMD;
    Data->rgbgenerator.GetMetaData(rgbMD);

    // create image
    RGBimage.SetSize(rgbMD.YRes(), rgbMD.XRes()*3);
    memcpy(RGBimage.Pointer(), rgbMD.Data(), rgbMD.YRes()*rgbMD.XRes()*3*sizeof(unsigned char));

    return osaOpenNI::ESUCCESS;

}

osaOpenNI::Errno osaOpenNI::GetRGBImage(vctDynamicMatrixRef<unsigned char> RGBimage)
{
    xn::ImageMetaData rgbMD;
    Data->rgbgenerator.GetMetaData(rgbMD);

    // check image size
    if (RGBimage.cols() != (rgbMD.XRes() * 3) ||
        RGBimage.rows() != rgbMD.YRes()) {
        CMN_LOG_RUN_ERROR << "osaOpenNI::GetRGBImage - image size mismatch" << std::endl;
        return osaOpenNI::EFAILURE;
    }

    memcpy(RGBimage.Pointer(), rgbMD.Data(), rgbMD.YRes() * rgbMD.XRes() * 3);

    return osaOpenNI::ESUCCESS;
}

osaOpenNI::Errno osaOpenNI::GetDepthImageRaw(vctDynamicMatrix<double> & depthimage) {

    // Get data
    xn::DepthMetaData depthMD;
    Data->depthgenerator.GetMetaData(depthMD);
    const XnDepthPixel* src = depthMD.Data();

    depthimage.SetSize(depthMD.YRes(), depthMD.XRes());
    double* dest = depthimage.Pointer();

    const size_t N = depthMD.YRes()*depthMD.XRes();
    for (size_t i = 0; i<N; i++) {
        (*dest) = (*src);
        src++;
        dest++;
    }

    return osaOpenNI::ESUCCESS;

}

osaOpenNI::Errno osaOpenNI::GetDepthImageRaw(vctDynamicMatrixRef<unsigned short> depthimage)
{
    xn::DepthMetaData depthMD;
    Data->depthgenerator.GetMetaData(depthMD);

    // check image size
    if (depthimage.cols() != depthMD.XRes() ||
        depthimage.rows() != depthMD.YRes()) {
        CMN_LOG_RUN_ERROR << "osaOpenNI::GetDepthImageRaw - image size mismatch" << std::endl;
        return osaOpenNI::EFAILURE;
    }

    memcpy(depthimage.Pointer(), depthMD.Data(), depthMD.YRes() * depthMD.XRes() * 2);

    return osaOpenNI::ESUCCESS;
}

void osaOpenNI::GetDepthImage(vctDynamicMatrix<double>& placeHolder) {

    // Get depth data
    xn::DepthMetaData depthMD;
    Data->depthgenerator.GetMetaData(depthMD);
    const XnDepthPixel* pDepth = depthMD.Data();

    placeHolder.SetSize(depthMD.YRes(), depthMD.XRes());
    double* ptr = placeHolder.Pointer();

    const size_t end = depthMD.YRes()*depthMD.XRes();
    for (size_t i = 0; i < end; i++)
        {
            (*ptr) =  255.0 * (*pDepth) / 2048.0;
            ptr++; pDepth++;
        }
}


osaOpenNI::Errno
osaOpenNI::GetRangeData(vctDynamicMatrix<double>& rangedata,
                        const std::vector< vctFixedSizeVector<unsigned short,2> >& pixels) {

    // Get data
    xn::DepthMetaData depthMD;
    Data->depthgenerator.GetMetaData(depthMD);

    // create arrays
    XnUInt32 cnt;
    XnPoint3D* proj = NULL;
    XnPoint3D* wrld = NULL;

    if (pixels.empty()) {

        // create arrays
        cnt = depthMD.XRes()*depthMD.YRes();
        proj = new XnPoint3D[ cnt ];
        wrld = new XnPoint3D[ cnt ];

        // Create projective coordinates
        for (size_t i=0, x=0; x<depthMD.XRes(); x++) {
            for (size_t y=0; y<depthMD.YRes(); i++, y++) {
                proj[i].X = (XnFloat)x;
                proj[i].Y = (XnFloat)y;
                proj[i].Z = depthMD(x,y);
            }
        }

    }
    else{

        // create arrays
        cnt = pixels.size();
        proj = new XnPoint3D[ cnt ];
        wrld = new XnPoint3D[ cnt ];

        for (size_t i=0; i<pixels.size(); i++) {
            unsigned int x = pixels[i][0];
            unsigned int y = pixels[i][1];
            proj[i].X = (XnFloat)x;
            proj[i].Y = (XnFloat)y;
            proj[i].Z = depthMD(x,y);
        }

    }

    // Convert projective to 3D
    XnStatus status = Data->depthgenerator.ConvertProjectiveToRealWorld(cnt, proj, wrld);
    if (status != XN_STATUS_OK) {
        CMN_LOG_RUN_ERROR << "Failed to convert projective to world: "
                          << xnGetStatusString(status) << std::endl;
    }

    // create matrix
    rangedata.SetSize(3, cnt);
    for (size_t i=0; i<cnt; i++) {
        rangedata[0][i] = -wrld[i].X/1000.0;
        rangedata[1][i] =  wrld[i].Y/1000.0;
        rangedata[2][i] =  wrld[i].Z/1000.0;
    }

    delete[] proj;
    delete[] wrld;

    return osaOpenNI::ESUCCESS;
}

osaOpenNI::Errno osaOpenNI::GetRangeData(vctDynamicMatrixRef<vctFloat3> rangedata)
{
    // Get data
    xn::DepthMetaData depthMD;
    Data->depthgenerator.GetMetaData(depthMD);

    const unsigned int width  = depthMD.XRes();
    const unsigned int height = depthMD.YRes();

    // check image size
    if (rangedata.cols() != width ||
        rangedata.rows() != height) {
        CMN_LOG_RUN_ERROR << "cisstOpenNI::GetRangeData - data size mismatch" << std::endl;
        return osaOpenNI::EFAILURE;
    }

    // update buffer size if needed
    const unsigned int size = width * height;
    if (PointsBufferSize < size) {
        if (ProjectivePointsBuffer) delete ProjectivePointsBuffer;
        if (WorldPointsBuffer)      delete WorldPointsBuffer;
        ProjectivePointsBuffer = new char[size * sizeof(XnPoint3D)];
        WorldPointsBuffer      = new char[size * sizeof(XnPoint3D)];
        if (!ProjectivePointsBuffer || !WorldPointsBuffer) {
            CMN_LOG_RUN_ERROR << "cisstOpenNI::GetRangeData - failed to allocate data buffer" << std::endl;
            return osaOpenNI::EFAILURE;
        }
    }

    XnPoint3D* proj = reinterpret_cast<XnPoint3D*>(ProjectivePointsBuffer);
    XnPoint3D* wrld = reinterpret_cast<XnPoint3D*>(WorldPointsBuffer);

    // create projective coordinates
    unsigned int i, x, y;
    for(i = 0, x = 0; x < width; x ++) {
        for(y = 0; y < height; i ++, y ++) {
            proj[i].X = static_cast<XnFloat>(x);
            proj[i].Y = static_cast<XnFloat>(y);
            proj[i].Z = static_cast<XnFloat>(depthMD(x, y));
        }
    }

    // convert projective to 3D
    XnStatus status = Data->depthgenerator.ConvertProjectiveToRealWorld(size, proj, wrld);
    if (status != XN_STATUS_OK) {
        CMN_LOG_RUN_ERROR << "cisstOpenNI::GetRangeData - failed to convert projective to world: "
                          << xnGetStatusString( status ) << std::endl;
    }

    for (i = 0, x = 0; x < width; x ++) {
        for (y = 0; y < height; i ++, y ++) {
            rangedata.Element(y, x).Assign(-wrld[i].X / 1000.0,
                                            wrld[i].Y / 1000.0,
                                            wrld[i].Z / 1000.0);
        }
    }

    return osaOpenNI::ESUCCESS;
}

std::vector<osaOpenNISkeleton*> &osaOpenNI::UpdateAndGetUserSkeletons() {

    // Initialize Users
    XnUserID aUsers[6];
    XnUInt16 nUsers = 6;
    Data->usergenerator.GetUsers(aUsers, nUsers);

    if(nUsers > this->users) printf("More users than max allowance\n");

    for (int i = 0; i < this->users; ++i)
        {
            if (Data->usergenerator.GetSkeletonCap().IsTracking(aUsers[i]))
                {
                    this->skeletons[i]->Update(aUsers[i]);
                }else{
                this->skeletons[i]->SetExists(false);
            }
        }

    return skeletons;
}

void osaOpenNI::UpdateUserSkeletons() {

    // Initialize Users
    XnUserID aUsers[6];
    XnUInt16 nUsers = 6;
    Data->usergenerator.GetUsers(aUsers, nUsers);

    if(nUsers > this->users) printf("More users than max allowance\n");

    for (int i = 0; i < this->users; ++i)
        {
            if (Data->usergenerator.GetSkeletonCap().IsTracking(aUsers[i]))
                {
                    this->skeletons[i]->Update(aUsers[i]);
                    this->skeletons[i]->usrState = Data->usrState;
                    this->skeletons[i]->calState = Data->usrCalState;
                }else{
                this->skeletons[i]->SetExists(false);
            }
        }
}

std::vector<osaOpenNISkeleton*> &osaOpenNI::GetUserSkeletons() {
    return skeletons;
}

osaOpenNI::Errno
osaOpenNI::Convert3DToProjectiveMask(const vctDynamicMatrix<double>& rangedata,
                                     vctDynamicMatrix<bool>& mask) {

    // allocate the arrays
    XnUInt32 cnt = rangedata.cols();
    XnPoint3D*  wrld = new XnPoint3D[ cnt ];
    XnPoint3D*  proj = new XnPoint3D[ cnt ];

    // copy the 3d points to the 3d array
    for (XnUInt32 i=0; i<cnt; i++) {
        wrld[i].X = -rangedata[0][i]*1000.0;
        wrld[i].Y =  rangedata[1][i]*1000.0;
        wrld[i].Z =  rangedata[2][i]*1000.0;
    }

    // convert to projective
    XnStatus status;
    status = Data->depthgenerator.ConvertRealWorldToProjective(cnt, wrld, proj);
    if (status != XN_STATUS_OK) {
        CMN_LOG_RUN_ERROR << "Failed to convert world to projective: "
                          << xnGetStatusString(status) << std::endl;
        return osaOpenNI::EFAILURE;
    }

    // use this to find the size of the mask
    xn::DepthMetaData depthMD;
    Data->depthgenerator.GetMetaData(depthMD);
    mask.SetSize(depthMD.YRes(), depthMD.XRes());
    mask.SetAll(false);

    for (XnUInt32 i=0; i<cnt; i++) {
        mask[ proj[i].Y ][ proj[i].X ] = true;
    }

    delete[] wrld;
    delete[] proj;

    return osaOpenNI::ESUCCESS;
}

void osaOpenNI::Convert3DToProjective(const vctFloat3& point3d, vctFloat3& point2d)
{
    Data->depthgenerator.ConvertRealWorldToProjective(1, (XnPoint3D*)(point3d.Pointer()), (XnPoint3D*)(point2d.Pointer()));
}








