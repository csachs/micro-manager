///////////////////////////////////////////////////////////////////////////////
// FILE:          SimulatingCamera.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   A work in progress camera device adapter pulling image data
///               via HTTP from specific feeder servers.
//
// AUTHOR:        Christian C. Sachs
//
// COPYRIGHT:     Forschungszentrum Jülich
// LICENSE:       BSD (2-clause/FreeBSD license)

#include "SimulatingCamera.h"
#include <cstdio>
#include <string>
#include <math.h>
#include "../../MMDevice/ModuleInterface.h"
#include <sstream>
#include <algorithm>
#include <iostream>
#include "picohttpclient.hpp"


using namespace std;
const double CSimulatingCamera::nominalPixelSizeUm_ = 1.0;
double g_IntensityFactor_ = 1.0;

// External names used used by the rest of the system
// to load particular device from the "DemoCamera.dll" library
const char* g_CameraDeviceName = "SimulatingCamera";


// constants for naming pixel types (allowed values of the "PixelType" property)
const char* g_PixelType_8bit = "8bit";
const char* g_PixelType_16bit = "16bit";
const char* g_PixelType_32bitRGB = "32bitRGB";
const char* g_PixelType_64bitRGB = "64bitRGB";
const char* g_PixelType_32bit = "32bit";  // floating point greyscale


///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////

MODULE_API void InitializeModuleData()
{
   RegisterDevice(g_CameraDeviceName, MM::CameraDevice, "SimulatingCamera");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
   if (deviceName == 0)
      return 0;

   // decide which device class to create based on the deviceName parameter
   if (strcmp(deviceName, g_CameraDeviceName) == 0)
   {
      // create camera
      return new CSimulatingCamera();
   }

   return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
   delete pDevice;
}


CSimulatingCamera::CSimulatingCamera() :
   CCameraBase<CSimulatingCamera> (),
   initialized_(false),
   readoutUs_(0.0),
   bitDepth_(8),
   roiX_(0),
   roiY_(0),
   sequenceStartTime_(0),
   binSize_(1),
   cameraCCDXSize_(512),
   cameraCCDYSize_(512),
   triggerDevice_(""),
   stopOnOverflow_(false),
   nComponents_(1)
{

   // call the base class method to set-up default error codes/messages
   InitializeDefaultErrorMessages();
   readoutStartTime_ = GetCurrentMMTime();
   thd_ = new MySequenceThread(this);

}

CSimulatingCamera::~CSimulatingCamera()
{
   StopSequenceAcquisition();
   delete thd_;
}


void CSimulatingCamera::GetName(char* name) const
{
   // Return the name used to referr to this device adapte
   CDeviceUtils::CopyLimitedString(name, g_CameraDeviceName);
}


int CSimulatingCamera::Initialize()
{
   if (initialized_)
      return DEVICE_OK;

   // set property list
   // -----------------

   // Name
   int nRet = CreateStringProperty(MM::g_Keyword_Name, g_CameraDeviceName, true);
   if (DEVICE_OK != nRet)
      return nRet;

   // Description
   nRet = CreateStringProperty(MM::g_Keyword_Description, "SimulatingCamera Adapter", true);
   if (DEVICE_OK != nRet)
      return nRet;

   // CameraName
   nRet = CreateStringProperty(MM::g_Keyword_CameraName, "SimulatingCamera", true);
   assert(nRet == DEVICE_OK);

   // CameraID
   nRet = CreateStringProperty(MM::g_Keyword_CameraID, "V1.0", true);
   assert(nRet == DEVICE_OK);

   nRet = CreateIntegerProperty("Binning", 1, true);
   assert(nRet == DEVICE_OK);

   CPropertyAction *pAct;


   pAct = new CPropertyAction (this, &CSimulatingCamera::OnURL);
   nRet = CreateProperty("URL", "http://localhost:8555/", MM::String, false, pAct, false);
   assert(nRet == DEVICE_OK);

   pAct = new CPropertyAction (this, &CSimulatingCamera::OnChannelDevice);
   nRet = CreateProperty("ChannelDevice", "DWheel", MM::String, false, pAct, false);
   assert(nRet == DEVICE_OK);

   // pixel type
   pAct = new CPropertyAction (this, &CSimulatingCamera::OnPixelType);
   nRet = CreateStringProperty(MM::g_Keyword_PixelType, g_PixelType_8bit, false, pAct);
   assert(nRet == DEVICE_OK);

   vector<string> pixelTypeValues;
   pixelTypeValues.push_back(g_PixelType_8bit);
   pixelTypeValues.push_back(g_PixelType_16bit);
   pixelTypeValues.push_back(g_PixelType_32bitRGB);
   pixelTypeValues.push_back(g_PixelType_64bitRGB);
   //pixelTypeValues.push_back(::g_PixelType_32bit);

   nRet = SetAllowedValues(MM::g_Keyword_PixelType, pixelTypeValues);
   if (nRet != DEVICE_OK)
      return nRet;

   // Bit depth
   pAct = new CPropertyAction (this, &CSimulatingCamera::OnBitDepth);
   nRet = CreateIntegerProperty("BitDepth", 8, false, pAct);
   assert(nRet == DEVICE_OK);

   vector<string> bitDepths;
   bitDepths.push_back("8");
   bitDepths.push_back("10");
   bitDepths.push_back("12");
   bitDepths.push_back("14");
   bitDepths.push_back("16");
   bitDepths.push_back("32");
   nRet = SetAllowedValues("BitDepth", bitDepths);
   if (nRet != DEVICE_OK)
      return nRet;


   // CCD size of the camera we are modeling
   pAct = new CPropertyAction (this, &CSimulatingCamera::OnCameraCCDXSize);
   CreateIntegerProperty("OnCameraCCDXSize", 512, false, pAct);
   pAct = new CPropertyAction (this, &CSimulatingCamera::OnCameraCCDYSize);
   CreateIntegerProperty("OnCameraCCDYSize", 512, false, pAct);


   // synchronize all properties
   // --------------------------
   nRet = UpdateStatus();
   if (nRet != DEVICE_OK)
      return nRet;


   // setup the buffer
   // ----------------
   nRet = ResizeImageBuffer();
   if (nRet != DEVICE_OK)
      return nRet;


   initialized_ = true;

   return DEVICE_OK;


}

/**
* Shuts down (unloads) the device.
* Required by the MM::Device API.
* Ideally this method will completely unload the device and release all resources.
* Shutdown() may be called multiple times in a row.
* After Shutdown() we should be allowed to call Initialize() again to load the device
* without causing problems.
*/
int CSimulatingCamera::Shutdown()
{
   initialized_ = false;
   return DEVICE_OK;
}

/**
* Performs exposure and grabs a single image.
* This function should block during the actual exposure and return immediately afterwards
* (i.e., before readout).  This behavior is needed for proper synchronization with the shutter.
* Required by the MM::Camera API.
*/
int CSimulatingCamera::SnapImage()
{
   static int callCounter = 0;
   ++callCounter;

   MM::MMTime startTime = GetCurrentMMTime();
   double exp = GetExposure();

   FetchImageFromUrl(img_);

   MM::MMTime s0(0,0);
   if( s0 < startTime )
   {
      while (exp > (GetCurrentMMTime() - startTime).getMsec())
      {
         CDeviceUtils::SleepMs(1);
      }
   }
   else
   {
      std::cerr << "You are operating this device adapter without setting the core callback, timing functions aren't yet available" << std::endl;
      // called without the core callback probably in off line test program
      // need way to build the core in the test program

   }
   readoutStartTime_ = GetCurrentMMTime();

   return DEVICE_OK;
}


/**
* Returns pixel data.
* Required by the MM::Camera API.
* The calling program will assume the size of the buffer based on the values
* obtained from GetImageBufferSize(), which in turn should be consistent with
* values returned by GetImageWidth(), GetImageHight() and GetImageBytesPerPixel().
* The calling program allso assumes that camera never changes the size of
* the pixel buffer on its own. In other words, the buffer can change only if
* appropriate properties are set (such as binning, pixel type, etc.)
*/
const unsigned char* CSimulatingCamera::GetImageBuffer()
{
   MMThreadGuard g(imgPixelsLock_);
   MM::MMTime readoutTime(readoutUs_);
   while (readoutTime > (GetCurrentMMTime() - readoutStartTime_)) {}
   unsigned char *pB = (unsigned char*)(img_.GetPixels());
   return pB;
}

/**
* Returns image buffer X-size in pixels.
* Required by the MM::Camera API.
*/
unsigned CSimulatingCamera::GetImageWidth() const
{
   return img_.Width();
}

/**
* Returns image buffer Y-size in pixels.
* Required by the MM::Camera API.
*/
unsigned CSimulatingCamera::GetImageHeight() const
{
   return img_.Height();
}

/**
* Returns image buffer pixel depth in bytes.
* Required by the MM::Camera API.
*/
unsigned CSimulatingCamera::GetImageBytesPerPixel() const
{
   return img_.Depth();
} 

/**
* Returns the bit depth (dynamic range) of the pixel.
* This does not affect the buffer size, it just gives the client application
* a guideline on how to interpret pixel values.
* Required by the MM::Camera API.
*/
unsigned CSimulatingCamera::GetBitDepth() const
{
   return bitDepth_;
}

/**
* Returns the size in bytes of the image buffer.
* Required by the MM::Camera API.
*/
long CSimulatingCamera::GetImageBufferSize() const
{
   return img_.Width() * img_.Height() * GetImageBytesPerPixel();
}

/**
* Sets the camera Region Of Interest.
* Required by the MM::Camera API.
* This command will change the dimensions of the image.
* Depending on the hardware capabilities the camera may not be able to configure the
* exact dimensions requested - but should try do as close as possible.
* If the hardware does not have this capability the software should simulate the ROI by
* appropriately cropping each frame.
* This demo implementation ignores the position coordinates and just crops the buffer.
* @param x - top-left corner coordinate
* @param y - top-left corner coordinate
* @param xSize - width
* @param ySize - height
*/
int CSimulatingCamera::SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize)
{
   if (xSize == 0 && ySize == 0)
   {
      // effectively clear ROI
      ResizeImageBuffer();
      roiX_ = 0;
      roiY_ = 0;
   }
   else
   {
      // apply ROI
      img_.Resize(xSize, ySize);
      roiX_ = x;
      roiY_ = y;
   }
   return DEVICE_OK;
}

/**
* Returns the actual dimensions of the current ROI.
* Required by the MM::Camera API.
*/
int CSimulatingCamera::GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize)
{
   x = roiX_;
   y = roiY_;

   xSize = img_.Width();
   ySize = img_.Height();

   return DEVICE_OK;
}

/**
* Resets the Region of Interest to full frame.
* Required by the MM::Camera API.
*/
int CSimulatingCamera::ClearROI()
{
   ResizeImageBuffer();
   roiX_ = 0;
   roiY_ = 0;

   return DEVICE_OK;
}

/**
* Returns the current exposure setting in milliseconds.
* Required by the MM::Camera API.
*/
double CSimulatingCamera::GetExposure() const
{
   char buf[MM::MaxStrLength];
   int ret = GetProperty(MM::g_Keyword_Exposure, buf);
   if (ret != DEVICE_OK)
      return 0.0;
   return atof(buf);
}

/**
* Sets exposure in milliseconds.
* Required by the MM::Camera API.
*/
void CSimulatingCamera::SetExposure(double exp)
{
   SetProperty(MM::g_Keyword_Exposure, CDeviceUtils::ConvertToString(exp));
   GetCoreCallback()->OnExposureChanged(this, exp);;
}

/**
* Returns the current binning factor.
* Required by the MM::Camera API.
*/
int CSimulatingCamera::GetBinning() const
{
   char buf[MM::MaxStrLength];
   int ret = GetProperty(MM::g_Keyword_Binning, buf);
   if (ret != DEVICE_OK)
      return 1;
   return atoi(buf);
}

/**
* Sets binning factor.
* Required by the MM::Camera API.
*/
int CSimulatingCamera::SetBinning(int binF)
{
   return SetProperty(MM::g_Keyword_Binning, CDeviceUtils::ConvertToString(binF));
}

int CSimulatingCamera::IsExposureSequenceable(bool& isSequenceable) const
{
   isSequenceable = false;
   return DEVICE_OK;
}

/**
 * Required by the MM::Camera API
 * Please implement this yourself and do not rely on the base class implementation
 * The Base class implementation is deprecated and will be removed shortly
 */
int CSimulatingCamera::StartSequenceAcquisition(double interval)
{
   return StartSequenceAcquisition(LONG_MAX, interval, false);
}

/**
* Stop and wait for the Sequence thread finished
*/
int CSimulatingCamera::StopSequenceAcquisition()
{
   if (!thd_->IsStopped()) {
      thd_->Stop();
      thd_->wait();
   }

   return DEVICE_OK;
}

/**
* Simple implementation of Sequence Acquisition
* A sequence acquisition should run on its own thread and transport new images
* coming of the camera into the MMCore circular buffer.
*/
int CSimulatingCamera::StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow)
{
   if (IsCapturing())
      return DEVICE_CAMERA_BUSY_ACQUIRING;

   int ret = GetCoreCallback()->PrepareForAcq(this);
   if (ret != DEVICE_OK)
      return ret;
   sequenceStartTime_ = GetCurrentMMTime();
   imageCounter_ = 0;
   thd_->Start(numImages,interval_ms);
   stopOnOverflow_ = stopOnOverflow;

   return DEVICE_OK;
}

/*
 * Inserts Image and MetaData into MMCore circular Buffer
 */
int CSimulatingCamera::InsertImage()
{
   MM::MMTime timeStamp = this->GetCurrentMMTime();
   char label[MM::MaxStrLength];
   this->GetLabel(label);

   // Important:  metadata about the image are generated here:
   Metadata md;
   md.put("Camera", label);
   md.put(MM::g_Keyword_Metadata_StartTime, CDeviceUtils::ConvertToString(sequenceStartTime_.getMsec()));
   md.put(MM::g_Keyword_Elapsed_Time_ms, CDeviceUtils::ConvertToString((timeStamp - sequenceStartTime_).getMsec()));
   md.put(MM::g_Keyword_Metadata_ROI_X, CDeviceUtils::ConvertToString( (long) roiX_));
   md.put(MM::g_Keyword_Metadata_ROI_Y, CDeviceUtils::ConvertToString( (long) roiY_));

   imageCounter_++;

   char buf[MM::MaxStrLength];
   GetProperty(MM::g_Keyword_Binning, buf);
   md.put(MM::g_Keyword_Binning, buf);

   MMThreadGuard g(imgPixelsLock_);

   const unsigned char* pI;
   pI = GetImageBuffer();

   unsigned int w = GetImageWidth();
   unsigned int h = GetImageHeight();
   unsigned int b = GetImageBytesPerPixel();

   int ret = GetCoreCallback()->InsertImage(this, pI, w, h, b, md.Serialize().c_str());
   if (!stopOnOverflow_ && ret == DEVICE_BUFFER_OVERFLOW)
   {
      // do not stop on overflow - just reset the buffer
      GetCoreCallback()->ClearImageBuffer(this);
      // don't process this same image again...
      return GetCoreCallback()->InsertImage(this, pI, w, h, b, md.Serialize().c_str(), false);
   }
   else
   {
      return ret;
   }
}

/*
 * Do actual capturing
 * Called from inside the thread
 */
int CSimulatingCamera::RunSequenceOnThread(MM::MMTime startTime)
{
   int ret=DEVICE_ERR;

   // Trigger
   if (triggerDevice_.length() > 0) {
      MM::Device* triggerDev = GetDevice(triggerDevice_.c_str());
      if (triggerDev != 0) {
         LogMessage("trigger requested");
         triggerDev->SetProperty("Trigger","+");
      }
   }

   double exposure = GetExposure();

   FetchImageFromUrl(img_);

   // Simulate exposure duration
   double finishTime = exposure * (imageCounter_ + 1);
   while ((GetCurrentMMTime() - startTime).getMsec() < finishTime)
   {
      CDeviceUtils::SleepMs(1);
   }

   ret = InsertImage();

   if (ret != DEVICE_OK)
   {
      return ret;
   }
   return ret;
};

bool CSimulatingCamera::IsCapturing() {
  return !thd_->IsStopped();
}

/*
 * called from the thread function before exit
 */
void CSimulatingCamera::OnThreadExiting() throw()
{
   try
   {
      LogMessage(g_Msg_SEQUENCE_ACQUISITION_THREAD_EXITING);
      GetCoreCallback()?GetCoreCallback()->AcqFinished(this,0):DEVICE_OK;
   }
   catch(...)
   {
      LogMessage(g_Msg_EXCEPTION_IN_ON_THREAD_EXITING, false);
   }
}


MySequenceThread::MySequenceThread(CSimulatingCamera* pCam)
   :intervalMs_(default_intervalMS)
   ,numImages_(default_numImages)
   ,imageCounter_(0)
   ,stop_(true)
   ,suspend_(false)
   ,camera_(pCam)
   ,startTime_(0)
   ,actualDuration_(0)
   ,lastFrameTime_(0)
{};

MySequenceThread::~MySequenceThread() {};

void MySequenceThread::Stop() {
   MMThreadGuard g(this->stopLock_);
   stop_=true;
}

void MySequenceThread::Start(long numImages, double intervalMs)
{
   MMThreadGuard g1(this->stopLock_);
   MMThreadGuard g2(this->suspendLock_);
   numImages_=numImages;
   intervalMs_=intervalMs;
   imageCounter_=0;
   stop_ = false;
   suspend_=false;
   activate();
   actualDuration_ = 0;
   startTime_= camera_->GetCurrentMMTime();
   lastFrameTime_ = 0;
}

bool MySequenceThread::IsStopped(){
   MMThreadGuard g(this->stopLock_);
   return stop_;
}

void MySequenceThread::Suspend() {
   MMThreadGuard g(this->suspendLock_);
   suspend_ = true;
}

bool MySequenceThread::IsSuspended() {
   MMThreadGuard g(this->suspendLock_);
   return suspend_;
}

void MySequenceThread::Resume() {
   MMThreadGuard g(this->suspendLock_);
   suspend_ = false;
}

int MySequenceThread::svc(void) throw()
{
   int ret=DEVICE_ERR;
   try
   {
      do
      {
         ret = camera_->RunSequenceOnThread(startTime_);
      } while (DEVICE_OK == ret && !IsStopped() && imageCounter_++ < numImages_-1);
      if (IsStopped())
         camera_->LogMessage("SeqAcquisition interrupted by the user\n");
   }catch(...){
      camera_->LogMessage(g_Msg_EXCEPTION_IN_THREAD, false);
   }
   stop_=true;
   actualDuration_ = camera_->GetCurrentMMTime() - startTime_;
   camera_->OnThreadExiting();
   return ret;
}



///////////////////////////////////////////////////////////////////////////////
// CSimulatingCamera Action handlers
///////////////////////////////////////////////////////////////////////////////



int CSimulatingCamera::OnURL(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(url_.c_str());
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(url_);
   }
   return DEVICE_OK;
}

int CSimulatingCamera::OnChannelDevice(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(channelDevice_.c_str());
   }
   else if (eAct == MM::AfterSet)
   {
      pProp->Get(channelDevice_);
   }
   return DEVICE_OK;
}


/**
* Handles "PixelType" property.
*/
int CSimulatingCamera::OnPixelType(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   int ret = DEVICE_ERR;
   switch(eAct)
   {
   case MM::AfterSet:
      {
         if(IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

         string pixelType;
         pProp->Get(pixelType);

         if (pixelType.compare(g_PixelType_8bit) == 0)
         {
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 1);
            bitDepth_ = 8;
            ret=DEVICE_OK;
         }
         else if (pixelType.compare(g_PixelType_16bit) == 0)
         {
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 2);
            bitDepth_ = 16;
            ret=DEVICE_OK;
         }
         else if ( pixelType.compare(g_PixelType_32bitRGB) == 0)
         {
            nComponents_ = 4;
            img_.Resize(img_.Width(), img_.Height(), 4);
            bitDepth_ = 8;
            ret=DEVICE_OK;
         }
         else if ( pixelType.compare(g_PixelType_64bitRGB) == 0)
         {
            nComponents_ = 4;
            img_.Resize(img_.Width(), img_.Height(), 8);
            bitDepth_ = 16;
            ret=DEVICE_OK;
         }
         else if ( pixelType.compare(g_PixelType_32bit) == 0)
         {
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 4);
            bitDepth_ = 32;
            ret=DEVICE_OK;
         }
         else
         {
            // on error switch to default pixel type
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 1);
            pProp->Set(g_PixelType_8bit);
            bitDepth_ = 8;
            ret = ERR_UNKNOWN_MODE;
         }
      }
      break;
   case MM::BeforeGet:
      {
         long bytesPerPixel = GetImageBytesPerPixel();
         if (bytesPerPixel == 1)
         {
            pProp->Set(g_PixelType_8bit);
         }
         else if (bytesPerPixel == 2)
         {
            pProp->Set(g_PixelType_16bit);
         }
         else if (bytesPerPixel == 4)
         {
            if (nComponents_ == 4)
            {
               pProp->Set(g_PixelType_32bitRGB);
            }
            else if (nComponents_ == 1)
            {
               pProp->Set(::g_PixelType_32bit);
            }
         }
         else if (bytesPerPixel == 8)
         {
            pProp->Set(g_PixelType_64bitRGB);
         }
       else
         {
            pProp->Set(g_PixelType_8bit);
         }
         ret = DEVICE_OK;
      } break;
   default:
      break;
   }
   return ret;
}

/**
* Handles "BitDepth" property.
*/
int CSimulatingCamera::OnBitDepth(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   int ret = DEVICE_ERR;
   switch(eAct)
   {
   case MM::AfterSet:
      {
         if(IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;

         long bitDepth;
         pProp->Get(bitDepth);

         unsigned int bytesPerComponent;

         switch (bitDepth) {
            case 8:
               bytesPerComponent = 1;
               bitDepth_ = 8;
               ret=DEVICE_OK;
            break;
            case 10:
               bytesPerComponent = 2;
               bitDepth_ = 10;
               ret=DEVICE_OK;
            break;
            case 12:
               bytesPerComponent = 2;
               bitDepth_ = 12;
               ret=DEVICE_OK;
            break;
            case 14:
               bytesPerComponent = 2;
               bitDepth_ = 14;
               ret=DEVICE_OK;
            break;
            case 16:
               bytesPerComponent = 2;
               bitDepth_ = 16;
               ret=DEVICE_OK;
            break;
            case 32:
               bytesPerComponent = 4;
               bitDepth_ = 32;
               ret=DEVICE_OK;
            break;
            default:
               // on error switch to default pixel type
               bytesPerComponent = 1;

               pProp->Set((long)8);
               bitDepth_ = 8;
               ret = ERR_UNKNOWN_MODE;
            break;
         }
         char buf[MM::MaxStrLength];
         GetProperty(MM::g_Keyword_PixelType, buf);
         std::string pixelType(buf);
         unsigned int bytesPerPixel = 1;


         // automagickally change pixel type when bit depth exceeds possible value
         if (pixelType.compare(g_PixelType_8bit) == 0)
         {
            if( 2 == bytesPerComponent)
            {
               SetProperty(MM::g_Keyword_PixelType, g_PixelType_16bit);
               bytesPerPixel = 2;
            }
            else if ( 4 == bytesPerComponent)
            {
               SetProperty(MM::g_Keyword_PixelType, g_PixelType_32bit);
               bytesPerPixel = 4;

            }else
            {
               bytesPerPixel = 1;
            }
         }
         else if (pixelType.compare(g_PixelType_16bit) == 0)
         {
            bytesPerPixel = 2;
         }
         else if ( pixelType.compare(g_PixelType_32bitRGB) == 0)
         {
            bytesPerPixel = 4;
         }
         else if ( pixelType.compare(g_PixelType_32bit) == 0)
         {
            bytesPerPixel = 4;
         }
         else if ( pixelType.compare(g_PixelType_64bitRGB) == 0)
         {
            bytesPerPixel = 8;
         }
         img_.Resize(img_.Width(), img_.Height(), bytesPerPixel);

      } break;
   case MM::BeforeGet:
      {
         pProp->Set((long)bitDepth_);
         ret=DEVICE_OK;
      } break;
   default:
      break;
   }
   return ret;
}


int CSimulatingCamera::OnCameraCCDXSize(MM::PropertyBase* pProp , MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(cameraCCDXSize_);
   }
   else if (eAct == MM::AfterSet)
   {
      long value;
      pProp->Get(value);
      if ( (value < 16) || (33000 < value))
         return DEVICE_ERR;  // invalid image size
      if( value != cameraCCDXSize_)
      {
         cameraCCDXSize_ = value;
         img_.Resize(cameraCCDXSize_, cameraCCDYSize_);
      }
   }
   return DEVICE_OK;

}

int CSimulatingCamera::OnCameraCCDYSize(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(cameraCCDYSize_);
   }
   else if (eAct == MM::AfterSet)
   {
      long value;
      pProp->Get(value);
      if ( (value < 16) || (33000 < value))
         return DEVICE_ERR;  // invalid image size
      if( value != cameraCCDYSize_)
      {
         cameraCCDYSize_ = value;
         img_.Resize(cameraCCDXSize_, cameraCCDYSize_);
      }
   }
   return DEVICE_OK;

}


///////////////////////////////////////////////////////////////////////////////
// Private CSimulatingCamera methods
///////////////////////////////////////////////////////////////////////////////

/**
* Sync internal image buffer size to the chosen property values.
*/
int CSimulatingCamera::ResizeImageBuffer()
{
   char buf[MM::MaxStrLength];
   //int ret = GetProperty(MM::g_Keyword_Binning, buf);
   //if (ret != DEVICE_OK)
   //   return ret;
   //binSize_ = atol(buf);

   int ret = GetProperty(MM::g_Keyword_PixelType, buf);
   if (ret != DEVICE_OK)
      return ret;

   std::string pixelType(buf);
   int byteDepth = 0;

   if (pixelType.compare(g_PixelType_8bit) == 0)
   {
      byteDepth = 1;
   }
   else if (pixelType.compare(g_PixelType_16bit) == 0)
   {
      byteDepth = 2;
   }
   else if ( pixelType.compare(g_PixelType_32bitRGB) == 0)
   {
      byteDepth = 4;
   }
   else if ( pixelType.compare(g_PixelType_32bit) == 0)
   {
      byteDepth = 4;
   }
   else if ( pixelType.compare(g_PixelType_64bitRGB) == 0)
   {
      byteDepth = 8;
   }

   img_.Resize(cameraCCDXSize_, cameraCCDYSize_, byteDepth);
   return DEVICE_OK;
}


inline long min(long a, long b) {
    return (a < b) ? a : b;
}

bool CSimulatingCamera::FetchImageFromUrl(ImgBuffer& img)
{
   MMThreadGuard g(imgPixelsLock_);

   double x, y, z;

   GetCoreCallback()->GetXYPosition(x, y);
   GetCoreCallback()->GetFocusPosition(z);

   long channel = 0;

   if(channelDevice_.length() > 0)
   {
     // I'd love to avoid "GetStateDevice"; but the device-agnostic API does not seem
     // to give access to a "get position of a state device" function
     // (i.e. I need to know the name of the state property)
     MM::State *stateDevice = GetCoreCallback()->GetStateDevice(this, channelDevice_.c_str());
     if(stateDevice)
     {
        stateDevice->GetPosition(channel);
     }
   }

   // This works ... properly fetches <x,y,z> coordinates

   URI theUri(url_);

   theUri.querystring = string("width=") + CDeviceUtils::ConvertToString((int)img_.Width()) + "&"
    "height=" + CDeviceUtils::ConvertToString((int)img_.Height()) + "&"
    "depth=" + CDeviceUtils::ConvertToString((int)img_.Depth()) + "&"
    "x=" + CDeviceUtils::ConvertToString(x) + "&"
    "y=" + CDeviceUtils::ConvertToString(y) + "&"
    "z=" + CDeviceUtils::ConvertToString(z) + "&"
    "channel=" + CDeviceUtils::ConvertToString(channel);

   HTTPResponse response = HTTPClient::request(HTTPClient::GET, theUri);

   if(response.success) {
     unsigned char* pBuf = const_cast<unsigned char*>(img_.GetPixels());
     memcpy(pBuf, response.body.c_str(), min(response.body.size(), img_.Height()*img_.Width()*img_.Depth()));
     return true;
   } else {
     return false;
   }

}

