///////////////////////////////////////////////////////////////////////////////
// FILE:          PyCH.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   ...
//
// AUTHOR:        Christian C. Sachs
//
// COPYRIGHT:
// LICENSE:
#include "PyCH.h"
#include <dlfcn.h>
#include <cstdio>
#include <string>
#include <math.h>
#include "../../MMDevice/ModuleInterface.h"
#include <sstream>
#include <algorithm>
#include <iostream>

using namespace std;
const double CPyCH::nominalPixelSizeUm_ = 1.0;
double g_IntensityFactor_ = 1.0;
// External names used used by the rest of the system
// to load particular device from the "DemoCamera.dll" library
const char *g_CameraDeviceName = "PyCH";
// constants for naming pixel types (allowed values of the "PixelType" property)
const char *g_PixelType_8bit = "8bit";
const char *g_PixelType_16bit = "16bit";
const char *g_PixelType_32bitRGB = "32bitRGB";
const char *g_PixelType_64bitRGB = "64bitRGB";
const char *g_PixelType_32bit = "32bit";  // floating point greyscale
///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////
MODULE_API void InitializeModuleData() {
   RegisterDevice(g_CameraDeviceName, MM::CameraDevice, "PyCH");
   RegisterDevice("EmptyCamera", MM::CameraDevice, "EmptyCamera");
   RegisterDevice("MultiPyCH", MM::CameraDevice, "MultiPyCH");
}

MODULE_API MM::Device *CreateDevice(const char *deviceName) {
   if (deviceName == 0)
      return 0;
   // decide which device class to create based on the deviceName parameter
   if (strcmp(deviceName, g_CameraDeviceName) == 0) {
      // create camera
      return new CPyCH();
   } else if (strcmp(deviceName, "EmptyCamera") == 0) {
      return new CEmptyCam();
   } else if (strcmp(deviceName, "MultiPyCH") == 0) {
      return new CMultiCameraPyCH();
   }
   return 0;
}

MODULE_API void DeleteDevice(MM::Device *pDevice) {
   delete pDevice;
}



int PythonImageCallback::Initialize(MM::Device *host, MM::Core *core) {
   host_ = host;
   callback_ = core;

   /*
    * Initialize Python
    */

   // Python's shared library needs to be loaded, so the linker
   // can find symbols from libraries which in turn will be later loaded by python
   dlopen("libpython3.5m.so", RTLD_LAZY | RTLD_GLOBAL);

   try {

      Py_Initialize();

      main_module = p::import("__main__");
      main_namespace = main_module.attr("__dict__");

      np::initialize();

      p::exec("_parameters = {}", main_namespace);
      p::exec("def _set_parameters(s):\n"
                    "    import json\n"
                    "    global _parameters\n"
                    "    _parameters = json.loads(s)", main_namespace);
      p::exec("def _callback(): pass", main_namespace);
   }  catch(p::error_already_set const &) {
      PyErr_Print();
   }

}

void PythonImageCallback::runScript(std::string name) {
   try {
      p::exec_file(name.c_str(), main_namespace);
   }  catch(p::error_already_set const &) {
      PyErr_Print();
   }
}

void PythonImageCallback::updateValuesXYZ() {
   double x, y, z;

   GetCoreCallback()->GetXYPosition(x, y);
   GetCoreCallback()->GetFocusPosition(z);

   try {
      main_namespace["_x"] = x;
      main_namespace["_y"] = y;
      main_namespace["_z"] = z;
   } catch (p::error_already_set const &) {
      PyErr_Print();
   }
}

void PythonImageCallback::updateValuesChannelDevice() {
   long channel = 0;

   if (channelDevice_.length() > 0) {
      MM::State *stateDevice = GetCoreCallback()->GetStateDevice(host_, channelDevice_.c_str());
      if (stateDevice) {
         stateDevice->GetPosition(channel);
      }
   }

   try {
      main_namespace["_channel"] = channel;
   } catch(p::error_already_set const &) {
      PyErr_Print();
   }
}

   /*
   if (cameraDevice_.length() > 0) {
      // I'd love to avoid "GetStateDevice"; but the device-agnostic API does not seem
      // to give access to a "get position of a state device" function
      // (i.e. I need to know the name of the state property)
      MM::Camera *cameraDevice = static_cast<MM::Camera*>(GetCoreCallback()->GetDevice(this, cameraDevice_.c_str()));

      if (cameraDevice) {
         cameraDevice->SnapImage();
         const unsigned char *otherBuffer = cameraDevice->GetImageBuffer();
         memcpy(img_.GetPixelsRW(), otherBuffer, min(cameraDevice->GetImageBufferSize(), img_.Width()*img_.Height()*img_.Depth()));
      }
   }
   */


void PythonImageCallback::processImage(ImgBuffer& img_) {
   try {
      np::ndarray array = np::from_data(
            img_.GetPixelsRW(), //const_cast<uint8_t *>(),
            dtype_conversion(img_.Depth()),
            p::make_tuple(img_.Height(), img_.Width()),
            p::make_tuple(img_.Width() * img_.Depth(), img_.Depth()),
            p::object()
      );

      main_namespace["_image_buffer"] = array;

      main_namespace["_callback"]();

      //p::exec("_callback()", main_namespace);

   } catch(p::error_already_set const &) {
      PyErr_Print();
   }
}


CPyCH::CPyCH() :
      CCameraBase<CPyCH>(),
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
      nComponents_(1) {
   // call the base class method to set-up default error codes/messages
   InitializeDefaultErrorMessages();
   readoutStartTime_ = GetCurrentMMTime();
   thd_ = new MySequenceThread(this);

}

CPyCH::~CPyCH() {
   StopSequenceAcquisition();
   delete thd_;
}

void CPyCH::GetName(char *name) const {
   // Return the name used to referr to this device adapte
   CDeviceUtils::CopyLimitedString(name, g_CameraDeviceName);
}

int CPyCH::Initialize() {
   if (initialized_)
      return DEVICE_OK;

   // set property list
   // -----------------
   // Name
   int nRet = CreateStringProperty(MM::g_Keyword_Name, g_CameraDeviceName, true);
   if (DEVICE_OK != nRet)
      return nRet;
   // Description
   nRet = CreateStringProperty(MM::g_Keyword_Description, "PyCH Adapter", true);
   if (DEVICE_OK != nRet)
      return nRet;
   // CameraName
   nRet = CreateStringProperty(MM::g_Keyword_CameraName, "PyCH", true);
   assert(nRet == DEVICE_OK);
   // CameraID
   nRet = CreateStringProperty(MM::g_Keyword_CameraID, "V1.0", true);
   assert(nRet == DEVICE_OK);

   nRet = CreateIntegerProperty("Binning", 1, true);
   assert(nRet == DEVICE_OK);
   CPropertyAction *pAct;


   pAct = new CPropertyAction(this, &CPyCH::OnScript);
   nRet = CreateProperty("ScriptPath", "", MM::String, false, pAct, false);
   assert(nRet == DEVICE_OK);

   pAct = new CPropertyAction(this, &CPyCH::OnChannelDevice);
   nRet = CreateProperty("ChannelDevice", "DWheel", MM::String, false, pAct, false);
   assert(nRet == DEVICE_OK);

   pAct = new CPropertyAction(this, &CPyCH::OnCameraDevice);
   nRet = CreateProperty("CameraDevice", "DCam", MM::String, false, pAct, false);
   assert(nRet == DEVICE_OK);

   // pixel type
   pAct = new CPropertyAction(this, &CPyCH::OnPixelType);
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
   pAct = new CPropertyAction(this, &CPyCH::OnBitDepth);
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
   pAct = new CPropertyAction(this, &CPyCH::OnCameraCCDXSize);
   CreateIntegerProperty("OnCameraCCDXSize", 512, false, pAct);
   pAct = new CPropertyAction(this, &CPyCH::OnCameraCCDYSize);
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

   pyc_.Initialize(this, GetCoreCallback());

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
int CPyCH::Shutdown() {
   initialized_ = false;
   return DEVICE_OK;
}

/**
* Performs exposure and grabs a single image.
* This function should block during the actual exposure and return immediately afterwards
* (i.e., before readout).  This behavior is needed for proper synchronization with the shutter.
* Required by the MM::Camera API.
*/
int CPyCH::SnapImage() {
   static int callCounter = 0;
   ++callCounter;

   MM::MMTime startTime = GetCurrentMMTime();
   double exp = GetExposure();
   PerformCallback(img_);
   MM::MMTime s0(0, 0);
   if (s0 < startTime) {
      while (exp > (GetCurrentMMTime() - startTime).getMsec()) {
         CDeviceUtils::SleepMs(1);
      }
   } else {
      std::cerr
            << "You are operating this device adapter without setting the core callback, timing functions aren't yet available"
            << std::endl;
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
const unsigned char *CPyCH::GetImageBuffer() {
   MMThreadGuard g(imgPixelsLock_);
   MM::MMTime readoutTime(readoutUs_);
   while (readoutTime > (GetCurrentMMTime() - readoutStartTime_)) {}
   unsigned char *pB = (unsigned char *) (img_.GetPixels());
   return pB;
}

/**
* Returns image buffer X-size in pixels.
* Required by the MM::Camera API.
*/
unsigned CPyCH::GetImageWidth() const {
   return img_.Width();
}

/**
* Returns image buffer Y-size in pixels.
* Required by the MM::Camera API.
*/
unsigned CPyCH::GetImageHeight() const {
   return img_.Height();
}

/**
* Returns image buffer pixel depth in bytes.
* Required by the MM::Camera API.
*/
unsigned CPyCH::GetImageBytesPerPixel() const {
   return img_.Depth();
}

/**
* Returns the bit depth (dynamic range) of the pixel.
* This does not affect the buffer size, it just gives the client application
* a guideline on how to interpret pixel values.
* Required by the MM::Camera API.
*/
unsigned CPyCH::GetBitDepth() const {
   return bitDepth_;
}

/**
* Returns the size in bytes of the image buffer.
* Required by the MM::Camera API.
*/
long CPyCH::GetImageBufferSize() const {
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
int CPyCH::SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize) {
   if (xSize == 0 && ySize == 0) {
      // effectively clear ROI
      ResizeImageBuffer();
      roiX_ = 0;
      roiY_ = 0;
   } else {
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
int CPyCH::GetROI(unsigned &x, unsigned &y, unsigned &xSize, unsigned &ySize) {
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
int CPyCH::ClearROI() {
   ResizeImageBuffer();
   roiX_ = 0;
   roiY_ = 0;

   return DEVICE_OK;
}

/**
* Returns the current exposure setting in milliseconds.
* Required by the MM::Camera API.
*/
double CPyCH::GetExposure() const {
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
void CPyCH::SetExposure(double exp) {
   SetProperty(MM::g_Keyword_Exposure, CDeviceUtils::ConvertToString(exp));
   GetCoreCallback()->OnExposureChanged(this, exp);;
}

/**
* Returns the current binning factor.
* Required by the MM::Camera API.
*/
int CPyCH::GetBinning() const {
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
int CPyCH::SetBinning(int binF) {
   return SetProperty(MM::g_Keyword_Binning, CDeviceUtils::ConvertToString(binF));
}

int CPyCH::IsExposureSequenceable(bool &isSequenceable) const {
   isSequenceable = false;
   return DEVICE_OK;
}

/**
 * Required by the MM::Camera API
 * Please implement this yourself and do not rely on the base class implementation
 * The Base class implementation is deprecated and will be removed shortly
 */
int CPyCH::StartSequenceAcquisition(double interval) {
   return StartSequenceAcquisition(LONG_MAX, interval, false);
}

/**
* Stop and wait for the Sequence thread finished
*/
int CPyCH::StopSequenceAcquisition() {
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
int CPyCH::StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow) {
   if (IsCapturing())
      return DEVICE_CAMERA_BUSY_ACQUIRING;
   int ret = GetCoreCallback()->PrepareForAcq(this);
   if (ret != DEVICE_OK)
      return ret;
   sequenceStartTime_ = GetCurrentMMTime();
   imageCounter_ = 0;
   thd_->Start(numImages, interval_ms);
   stopOnOverflow_ = stopOnOverflow;

   return DEVICE_OK;
}

/*
 * Inserts Image and MetaData into MMCore circular Buffer
 */
int CPyCH::InsertImage() {
   MM::MMTime timeStamp = this->GetCurrentMMTime();
   char label[MM::MaxStrLength];
   this->GetLabel(label);

   // Important:  metadata about the image are generated here:
   Metadata md;
   md.put("Camera", label);
   md.put(MM::g_Keyword_Metadata_StartTime, CDeviceUtils::ConvertToString(sequenceStartTime_.getMsec()));
   md.put(MM::g_Keyword_Elapsed_Time_ms, CDeviceUtils::ConvertToString((timeStamp - sequenceStartTime_).getMsec()));
   md.put(MM::g_Keyword_Metadata_ROI_X, CDeviceUtils::ConvertToString((long) roiX_));
   md.put(MM::g_Keyword_Metadata_ROI_Y, CDeviceUtils::ConvertToString((long) roiY_));
   imageCounter_++;
   char buf[MM::MaxStrLength];
   GetProperty(MM::g_Keyword_Binning, buf);
   md.put(MM::g_Keyword_Binning, buf);
   MMThreadGuard g(imgPixelsLock_);
   const unsigned char *pI;
   pI = GetImageBuffer();
   unsigned int w = GetImageWidth();
   unsigned int h = GetImageHeight();
   unsigned int b = GetImageBytesPerPixel();
   int ret = GetCoreCallback()->InsertImage(this, pI, w, h, b, md.Serialize().c_str());
   if (!stopOnOverflow_ && ret == DEVICE_BUFFER_OVERFLOW) {
      // do not stop on overflow - just reset the buffer
      GetCoreCallback()->ClearImageBuffer(this);
      // don't process this same image again...
      return GetCoreCallback()->InsertImage(this, pI, w, h, b, md.Serialize().c_str(), false);
   } else {
      return ret;
   }
}

/*
 * Do actual capturing
 * Called from inside the thread
 */
int CPyCH::RunSequenceOnThread(MM::MMTime startTime) {
   int ret = DEVICE_ERR;

   // Trigger
   if (triggerDevice_.length() > 0) {
      MM::Device *triggerDev = GetDevice(triggerDevice_.c_str());
      if (triggerDev != 0) {
         LogMessage("trigger requested");
         triggerDev->SetProperty("Trigger", "+");
      }
   }

   double exposure = GetExposure();
   PerformCallback(img_);
   // Simulate exposure duration
   double finishTime = exposure * (imageCounter_ + 1);
   while ((GetCurrentMMTime() - startTime).getMsec() < finishTime) {
      CDeviceUtils::SleepMs(1);
   }
   ret = InsertImage();
   if (ret != DEVICE_OK) {
      return ret;
   }
   return ret;
};

bool CPyCH::IsCapturing() {
   return !thd_->IsStopped();
}

/*
 * called from the thread function before exit
 */
void CPyCH::OnThreadExiting() throw() {
   try {
      LogMessage(g_Msg_SEQUENCE_ACQUISITION_THREAD_EXITING);
      GetCoreCallback() ? GetCoreCallback()->AcqFinished(this, 0) : DEVICE_OK;
   }
   catch (...) {
      LogMessage(g_Msg_EXCEPTION_IN_ON_THREAD_EXITING, false);
   }
}

MySequenceThread::MySequenceThread(CPyCH *pCam)
      : intervalMs_(default_intervalMS), numImages_(default_numImages), imageCounter_(0), stop_(true), suspend_(false),
        camera_(pCam), startTime_(0), actualDuration_(0), lastFrameTime_(0) {};

MySequenceThread::~MySequenceThread() {};

void MySequenceThread::Stop() {
   MMThreadGuard g(this->stopLock_);
   stop_ = true;
}

void MySequenceThread::Start(long numImages, double intervalMs) {
   MMThreadGuard g1(this->stopLock_);
   MMThreadGuard g2(this->suspendLock_);
   numImages_ = numImages;
   intervalMs_ = intervalMs;
   imageCounter_ = 0;
   stop_ = false;
   suspend_ = false;
   activate();
   actualDuration_ = 0;
   startTime_ = camera_->GetCurrentMMTime();
   lastFrameTime_ = 0;
}

bool MySequenceThread::IsStopped() {
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

int MySequenceThread::svc(void) throw() {
   int ret = DEVICE_ERR;
   try {
      do {
         ret = camera_->RunSequenceOnThread(startTime_);
      } while (DEVICE_OK == ret && !IsStopped() && imageCounter_++ < numImages_ - 1);
      if (IsStopped())
         camera_->LogMessage("SeqAcquisition interrupted by the user\n");
   } catch (...) {
      camera_->LogMessage(g_Msg_EXCEPTION_IN_THREAD, false);
   }
   stop_ = true;
   actualDuration_ = camera_->GetCurrentMMTime() - startTime_;
   camera_->OnThreadExiting();
   return ret;
}


///////////////////////////////////////////////////////////////////////////////
// CPyCH Action handlers
///////////////////////////////////////////////////////////////////////////////
int CPyCH::OnScript(MM::PropertyBase *pProp, MM::ActionType eAct) {
   if (eAct == MM::BeforeGet) {
      pProp->Set(scriptFile_.c_str());
   } else if (eAct == MM::AfterSet) {
      pProp->Get(scriptFile_);
      if(scriptFile_ != "") {
         pyc_.runScript(scriptFile_);
      }
   }
   return DEVICE_OK;
}

int CPyCH::OnChannelDevice(MM::PropertyBase *pProp, MM::ActionType eAct) {
   if (eAct == MM::BeforeGet) {
      pProp->Set(pyc_.getChannelDevice().c_str());
   } else if (eAct == MM::AfterSet) {
      std::string channelDevice;
      pProp->Get(channelDevice);
      pyc_.setChannelDevice(channelDevice);
   }
   return DEVICE_OK;
}

int CPyCH::OnCameraDevice(MM::PropertyBase *pProp, MM::ActionType eAct) {
   if (eAct == MM::BeforeGet) {
      pProp->Set(cameraDevice_.c_str());
   } else if (eAct == MM::AfterSet) {
      pProp->Get(cameraDevice_);
   }
   return DEVICE_OK;
}

/**
* Handles "PixelType" property.
*/
int CPyCH::OnPixelType(MM::PropertyBase *pProp, MM::ActionType eAct) {
   int ret = DEVICE_ERR;
   switch (eAct) {
      case MM::AfterSet: {
         if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;
         string pixelType;
         pProp->Get(pixelType);
         if (pixelType.compare(g_PixelType_8bit) == 0) {
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 1);
            bitDepth_ = 8;
            ret = DEVICE_OK;
         } else if (pixelType.compare(g_PixelType_16bit) == 0) {
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 2);
            bitDepth_ = 16;
            ret = DEVICE_OK;
         } else if (pixelType.compare(g_PixelType_32bitRGB) == 0) {
            nComponents_ = 4;
            img_.Resize(img_.Width(), img_.Height(), 4);
            bitDepth_ = 8;
            ret = DEVICE_OK;
         } else if (pixelType.compare(g_PixelType_64bitRGB) == 0) {
            nComponents_ = 4;
            img_.Resize(img_.Width(), img_.Height(), 8);
            bitDepth_ = 16;
            ret = DEVICE_OK;
         } else if (pixelType.compare(g_PixelType_32bit) == 0) {
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 4);
            bitDepth_ = 32;
            ret = DEVICE_OK;
         } else {
            // on error switch to default pixel type
            nComponents_ = 1;
            img_.Resize(img_.Width(), img_.Height(), 1);
            pProp->Set(g_PixelType_8bit);
            bitDepth_ = 8;
            ret = ERR_UNKNOWN_MODE;
         }
      }
         break;
      case MM::BeforeGet: {
         long bytesPerPixel = GetImageBytesPerPixel();
         if (bytesPerPixel == 1) {
            pProp->Set(g_PixelType_8bit);
         } else if (bytesPerPixel == 2) {
            pProp->Set(g_PixelType_16bit);
         } else if (bytesPerPixel == 4) {
            if (nComponents_ == 4) {
               pProp->Set(g_PixelType_32bitRGB);
            } else if (nComponents_ == 1) {
               pProp->Set(::g_PixelType_32bit);
            }
         } else if (bytesPerPixel == 8) {
            pProp->Set(g_PixelType_64bitRGB);
         } else {
            pProp->Set(g_PixelType_8bit);
         }
         ret = DEVICE_OK;
      }
         break;
      default:
         break;
   }
   return ret;
}

/**
* Handles "BitDepth" property.
*/
int CPyCH::OnBitDepth(MM::PropertyBase *pProp, MM::ActionType eAct) {
   int ret = DEVICE_ERR;
   switch (eAct) {
      case MM::AfterSet: {
         if (IsCapturing())
            return DEVICE_CAMERA_BUSY_ACQUIRING;
         long bitDepth;
         pProp->Get(bitDepth);
         unsigned int bytesPerComponent;
         switch (bitDepth) {
            case 8:
               bytesPerComponent = 1;
               bitDepth_ = 8;
               ret = DEVICE_OK;
               break;
            case 10:
               bytesPerComponent = 2;
               bitDepth_ = 10;
               ret = DEVICE_OK;
               break;
            case 12:
               bytesPerComponent = 2;
               bitDepth_ = 12;
               ret = DEVICE_OK;
               break;
            case 14:
               bytesPerComponent = 2;
               bitDepth_ = 14;
               ret = DEVICE_OK;
               break;
            case 16:
               bytesPerComponent = 2;
               bitDepth_ = 16;
               ret = DEVICE_OK;
               break;
            case 32:
               bytesPerComponent = 4;
               bitDepth_ = 32;
               ret = DEVICE_OK;
               break;
            default:
               // on error switch to default pixel type
               bytesPerComponent = 1;
               pProp->Set((long) 8);
               bitDepth_ = 8;
               ret = ERR_UNKNOWN_MODE;
               break;
         }
         char buf[MM::MaxStrLength];
         GetProperty(MM::g_Keyword_PixelType, buf);
         std::string pixelType(buf);
         unsigned int bytesPerPixel = 1;

         // automagickally change pixel type when bit depth exceeds possible value
         if (pixelType.compare(g_PixelType_8bit) == 0) {
            if (2 == bytesPerComponent) {
               SetProperty(MM::g_Keyword_PixelType, g_PixelType_16bit);
               bytesPerPixel = 2;
            } else if (4 == bytesPerComponent) {
               SetProperty(MM::g_Keyword_PixelType, g_PixelType_32bit);
               bytesPerPixel = 4;
            } else {
               bytesPerPixel = 1;
            }
         } else if (pixelType.compare(g_PixelType_16bit) == 0) {
            bytesPerPixel = 2;
         } else if (pixelType.compare(g_PixelType_32bitRGB) == 0) {
            bytesPerPixel = 4;
         } else if (pixelType.compare(g_PixelType_32bit) == 0) {
            bytesPerPixel = 4;
         } else if (pixelType.compare(g_PixelType_64bitRGB) == 0) {
            bytesPerPixel = 8;
         }
         img_.Resize(img_.Width(), img_.Height(), bytesPerPixel);
      }
         break;
      case MM::BeforeGet: {
         pProp->Set((long) bitDepth_);
         ret = DEVICE_OK;
      }
         break;
      default:
         break;
   }
   return ret;
}

int CPyCH::OnCameraCCDXSize(MM::PropertyBase *pProp, MM::ActionType eAct) {
   if (eAct == MM::BeforeGet) {
      pProp->Set(cameraCCDXSize_);
   } else if (eAct == MM::AfterSet) {
      long value;
      pProp->Get(value);
      if ((value < 16) || (33000 < value))
         return DEVICE_ERR;  // invalid image size
      if (value != cameraCCDXSize_) {
         cameraCCDXSize_ = value;
         img_.Resize(cameraCCDXSize_, cameraCCDYSize_);
      }
   }
   return DEVICE_OK;
}

int CPyCH::OnCameraCCDYSize(MM::PropertyBase *pProp, MM::ActionType eAct) {
   if (eAct == MM::BeforeGet) {
      pProp->Set(cameraCCDYSize_);
   } else if (eAct == MM::AfterSet) {
      long value;
      pProp->Get(value);
      if ((value < 16) || (33000 < value))
         return DEVICE_ERR;  // invalid image size
      if (value != cameraCCDYSize_) {
         cameraCCDYSize_ = value;
         img_.Resize(cameraCCDXSize_, cameraCCDYSize_);
      }
   }
   return DEVICE_OK;
}
///////////////////////////////////////////////////////////////////////////////
// Private CPyCH methods
///////////////////////////////////////////////////////////////////////////////
/**
* Sync internal image buffer size to the chosen property values.
*/
int CPyCH::ResizeImageBuffer() {
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
   if (pixelType.compare(g_PixelType_8bit) == 0) {
      byteDepth = 1;
   } else if (pixelType.compare(g_PixelType_16bit) == 0) {
      byteDepth = 2;
   } else if (pixelType.compare(g_PixelType_32bitRGB) == 0) {
      byteDepth = 4;
   } else if (pixelType.compare(g_PixelType_32bit) == 0) {
      byteDepth = 4;
   } else if (pixelType.compare(g_PixelType_64bitRGB) == 0) {
      byteDepth = 8;
   }
   img_.Resize(cameraCCDXSize_, cameraCCDYSize_, byteDepth);
   return DEVICE_OK;
}

inline long min(long a, long b) {
   return (a < b) ? a : b;
}



bool CPyCH::PerformCallback(ImgBuffer &img) {
   MMThreadGuard g(imgPixelsLock_);

   pyc_.updateValuesXYZ();
   pyc_.updateValuesChannelDevice();
   pyc_.processImage(img_);

}


/*****************************************************************/


const char* g_Undefined = "Undefined";
const char* g_deviceNameMultiCameraPyCH = "MultiCameraPyCH";


CMultiCameraPyCH::CMultiCameraPyCH() :
      imageBuffer_(0),
      nrCamerasInUse_(0),
      initialized_(false)
{
   InitializeDefaultErrorMessages();

   SetErrorText(ERR_INVALID_DEVICE_NAME, "Please select a valid camera");
   SetErrorText(ERR_NO_PHYSICAL_CAMERA, "No physical camera assigned");
   SetErrorText(ERR_NO_EQUAL_SIZE, "Cameras differ in image size");

   // Name
   CreateProperty(MM::g_Keyword_Name, g_deviceNameMultiCameraPyCH, MM::String, true);

   // Description
   CreateProperty(MM::g_Keyword_Description, "Combines multiple cameras into a single camera", MM::String, true);

   for (int i = 0; i < MAX_NUMBER_PHYSICAL_CAMERAS; i++) {
      usedCameras_.push_back(g_Undefined);
      physicalCameras_.push_back(0);
   }
}

CMultiCameraPyCH::~CMultiCameraPyCH()
{
   if (initialized_)
      Shutdown();
}

int CMultiCameraPyCH::Shutdown()
{
   delete imageBuffer_;
   // Rely on the cameras to shut themselves down
   return DEVICE_OK;
}

int CMultiCameraPyCH::Initialize()
{
   // get list with available Cameras.
   std::vector<std::string> availableCameras;
   availableCameras.clear();
   char deviceName[MM::MaxStrLength];
   unsigned int deviceIterator = 0;
   for(;;)
   {
      GetLoadedDeviceOfType(MM::CameraDevice, deviceName, deviceIterator++);
      if( 0 < strlen(deviceName))
      {
         availableCameras.push_back(std::string(deviceName));
      }
      else
         break;
   }

   availableCameras_.push_back(g_Undefined);
   std::vector<std::string>::iterator iter;
   for (iter = availableCameras.begin();
        iter != availableCameras.end();
        iter++ )
   {
      MM::Device* camera = GetDevice((*iter).c_str());
      std::ostringstream os;
      os << this << " " << camera;
      LogMessage(os.str().c_str());
      if (camera &&  (this != camera))
         availableCameras_.push_back(*iter);
   }

   for (long i = 0; i < MAX_NUMBER_PHYSICAL_CAMERAS; i++)
   {
      CPropertyActionEx* pAct = new CPropertyActionEx (this, &CMultiCameraPyCH::OnPhysicalCamera, i);
      std::ostringstream os;
      os << "Physical Camera " << i+1;
      CreateProperty(os.str().c_str(), availableCameras_[0].c_str(), MM::String, false, pAct, false);
      SetAllowedValues(os.str().c_str(), availableCameras_);
   }

   CPropertyAction* pAct = new CPropertyAction(this, &CMultiCameraPyCH::OnBinning);
   CreateProperty(MM::g_Keyword_Binning, "1", MM::Integer, false, pAct, false);

   initialized_ = true;

   return DEVICE_OK;
}

void CMultiCameraPyCH::GetName(char* name) const
{
   CDeviceUtils::CopyLimitedString(name, g_deviceNameMultiCameraPyCH);
}

int CMultiCameraPyCH::SnapImage()
{
   if (nrCamerasInUse_ < 1)
      return ERR_NO_PHYSICAL_CAMERA;

   if (!ImageSizesAreEqual())
      return ERR_NO_EQUAL_SIZE;

   PyCHMultiCameraSnapThread t[MAX_NUMBER_PHYSICAL_CAMERAS];
   for (unsigned int i = 0; i < physicalCameras_.size(); i++)
   {
      if (physicalCameras_[i] != 0)
      {
         t[i].SetCamera(physicalCameras_[i]);
         t[i].Start();
      }
   }
   // I think that the PyCHMultiCameraSnapThread destructor waits until the SnapImage function is done
   // So, we are likely to be waiting here until all cameras are done snapping

   return DEVICE_OK;
}

/**
 * return the ImageBuffer of the first physical camera
 */
const unsigned char* CMultiCameraPyCH::GetImageBuffer()
{
   if (nrCamerasInUse_ < 1)
      return 0;

   return GetImageBuffer(0);
}

const unsigned char* CMultiCameraPyCH::GetImageBuffer(unsigned channelNr)
{
   // We have a vector of physicalCameras, and a vector of Strings listing the cameras
   // we actually use.
   int j = -1;
   unsigned height = GetImageHeight();
   unsigned width = GetImageWidth();
   unsigned pixDepth = GetImageBytesPerPixel();
   for (unsigned int i = 0; i < physicalCameras_.size(); i++)
   {
      if (usedCameras_[i] != g_Undefined)
         j++;
      if (j == (int) channelNr)
      {
         unsigned thisHeight = physicalCameras_[i]->GetImageHeight();
         unsigned thisWidth = physicalCameras_[i]->GetImageWidth();
         if (height == thisHeight && width == thisWidth)
            return physicalCameras_[i]->GetImageBuffer();
         else
         {
            img_.Resize(width, height, pixDepth);
            img_.ResetPixels();
            if (width == thisWidth)
            {
               memcpy(img_.GetPixelsRW(), physicalCameras_[i]->GetImageBuffer(), thisHeight * thisWidth * pixDepth);
            }
            else
            {
               // we need to copy line by line
               const unsigned char* pixels = physicalCameras_[i]->GetImageBuffer();
               for (unsigned i=0; i < thisHeight; i++)
               {
                  memcpy(img_.GetPixelsRW() + i * width, pixels + i * thisWidth, thisWidth);
               }
            }
            return img_.GetPixels();
         }
      }
   }
   return 0;
}

bool CMultiCameraPyCH::IsCapturing()
{
   std::vector<MM::Camera*>::iterator iter;
   for (iter = physicalCameras_.begin(); iter != physicalCameras_.end(); iter++ ) {
      if ( (*iter != 0) && (*iter)->IsCapturing())
         return true;
   }

   return false;
}

/**
 * Returns the largest width of cameras used
 */
unsigned CMultiCameraPyCH::GetImageWidth() const
{
   // TODO: should we use cached width?
   // If so, when do we cache?
   // Since this function is const, we can not cache the width found
   unsigned width = 0;
   unsigned int j = 0;
   while (j < physicalCameras_.size() )
   {
      if (physicalCameras_[j] != 0) {
         unsigned tmp = physicalCameras_[j]->GetImageWidth();
         if (tmp > width)
            width = tmp;
      }
      j++;
   }

   return width;
}

/**
 * Returns the largest height of cameras used
 */
unsigned CMultiCameraPyCH::GetImageHeight() const
{
   unsigned height = 0;
   unsigned int j = 0;
   while (j < physicalCameras_.size() )
   {
      if (physicalCameras_[j] != 0)
      {
         unsigned tmp = physicalCameras_[j]->GetImageHeight();
         if (tmp > height)
            height = tmp;
      }
      j++;
   }

   return height;
}


/**
 * Returns true if image sizes of all available cameras are identical
 * false otherwise
 * edge case: if we have no or one camera, their sizes are equal
 */
bool CMultiCameraPyCH::ImageSizesAreEqual() {
   unsigned height = 0;
   unsigned width = 0;
   for (int i = 0; i < physicalCameras_.size(); i++) {
      if (physicalCameras_[i] != 0)
      {
         height = physicalCameras_[0]->GetImageHeight();
         width = physicalCameras_[0]->GetImageWidth();
      }
   }

   for (int i = 0; i < physicalCameras_.size(); i++) {
      if (physicalCameras_[i] != 0)
      {
         if (height != physicalCameras_[i]->GetImageHeight())
            return false;
         if (width != physicalCameras_[i]->GetImageWidth())
            return false;
      }
   }
   return true;
}

unsigned CMultiCameraPyCH::GetImageBytesPerPixel() const
{
   if (physicalCameras_[0] != 0)
   {
      unsigned bytes = physicalCameras_[0]->GetImageBytesPerPixel();
      for (unsigned int i = 1; i < physicalCameras_.size(); i++)
      {
         if (physicalCameras_[i] != 0)
            if (bytes != physicalCameras_[i]->GetImageBytesPerPixel())
               return 0;
      }
      return bytes;
   }
   return 0;
}

unsigned CMultiCameraPyCH::GetBitDepth() const
{
   // Return the maximum bit depth found in all channels.
   if (physicalCameras_[0] != 0)
   {
      unsigned bitDepth = 0;
      for (unsigned int i = 0; i < physicalCameras_.size(); i++)
      {
         if (physicalCameras_[i] != 0)
         {
            unsigned nextBitDepth = physicalCameras_[i]->GetBitDepth();
            if (nextBitDepth > bitDepth)
            {
               bitDepth = nextBitDepth;
            }
         }
      }
      return bitDepth;
   }
   return 0;
}

long CMultiCameraPyCH::GetImageBufferSize() const
{
   long maxSize = 0;
   int unsigned counter = 0;
   for (unsigned int i = 0; i < physicalCameras_.size(); i++)
   {
      if (physicalCameras_[i] != 0)
      {
         counter++;
         long tmp = physicalCameras_[i]->GetImageBufferSize();
         if (tmp > maxSize)
            maxSize = tmp;
      }
   }

   return counter * maxSize;
}

double CMultiCameraPyCH::GetExposure() const
{
   if (physicalCameras_[0] != 0)
   {
      double exposure = physicalCameras_[0]->GetExposure();
      for (unsigned int i = 1; i < physicalCameras_.size(); i++)
      {
         if (physicalCameras_[i] != 0)
            if (exposure != physicalCameras_[i]->GetExposure())
               return 0;
      }
      return exposure;
   }
   return 0.0;
}

void CMultiCameraPyCH::SetExposure(double exp)
{
   if (exp > 0.0)
   {
      for (unsigned int i = 0; i < physicalCameras_.size(); i++)
      {
         if (physicalCameras_[i] != 0)
            physicalCameras_[i]->SetExposure(exp);
      }
   }
}

int CMultiCameraPyCH::SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize)
{
   for (unsigned int i = 0; i < physicalCameras_.size(); i++)
   {
      // TODO: deal with case when CCD size are not identical
      if (physicalCameras_[i] != 0)
      {
         int ret = physicalCameras_[i]->SetROI(x, y, xSize, ySize);
         if (ret != DEVICE_OK)
            return ret;
      }
   }
   return DEVICE_OK;
}

int CMultiCameraPyCH::GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize)
{
   // TODO: check if ROI is same on all cameras
   if (physicalCameras_[0] != 0)
   {
      int ret = physicalCameras_[0]->GetROI(x, y, xSize, ySize);
      if (ret != DEVICE_OK)
         return ret;
   }

   return DEVICE_OK;
}

int CMultiCameraPyCH::ClearROI()
{
   for (unsigned int i = 0; i < physicalCameras_.size(); i++)
   {
      if (physicalCameras_[i] != 0)
      {
         int ret = physicalCameras_[i]->ClearROI();
         if (ret != DEVICE_OK)
            return ret;
      }
   }

   return DEVICE_OK;
}

int CMultiCameraPyCH::PrepareSequenceAcqusition()
{
   if (nrCamerasInUse_ < 1)
      return ERR_NO_PHYSICAL_CAMERA;

   for (unsigned int i = 0; i < physicalCameras_.size(); i++)
   {
      if (physicalCameras_[i] != 0)
      {
         int ret = physicalCameras_[i]->PrepareSequenceAcqusition();
         if (ret != DEVICE_OK)
            return ret;
      }
   }

   return DEVICE_OK;
}

int CMultiCameraPyCH::StartSequenceAcquisition(double interval)
{
   if (nrCamerasInUse_ < 1)
      return ERR_NO_PHYSICAL_CAMERA;

   if (!ImageSizesAreEqual())
      return ERR_NO_EQUAL_SIZE;

   for (unsigned int i = 0; i < physicalCameras_.size(); i++)
   {
      if (physicalCameras_[i] != 0)
      {
         std::ostringstream os;
         os << i;
         physicalCameras_[i]->AddTag(MM::g_Keyword_CameraChannelName, usedCameras_[i].c_str(),
                                     usedCameras_[i].c_str());
         physicalCameras_[i]->AddTag(MM::g_Keyword_CameraChannelIndex, usedCameras_[i].c_str(),
                                     os.str().c_str());

         int ret = physicalCameras_[i]->StartSequenceAcquisition(interval);
         if (ret != DEVICE_OK)
            return ret;
      }
   }
   return DEVICE_OK;
}

int CMultiCameraPyCH::StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow)
{
   if (nrCamerasInUse_ < 1)
      return ERR_NO_PHYSICAL_CAMERA;

   for (unsigned int i = 0; i < physicalCameras_.size(); i++)
   {
      if (physicalCameras_[i] != 0)
      {
         int ret = physicalCameras_[i]->StartSequenceAcquisition(numImages, interval_ms, stopOnOverflow);
         if (ret != DEVICE_OK)
            return ret;
      }
   }
   return DEVICE_OK;
}

int CMultiCameraPyCH::StopSequenceAcquisition()
{
   for (unsigned int i = 0; i < physicalCameras_.size(); i++)
   {
      if (physicalCameras_[i] != 0)
      {
         int ret = physicalCameras_[i]->StopSequenceAcquisition();

         //
         if (ret != DEVICE_OK)
            return ret;
         std::ostringstream os;
         os << 0;
         physicalCameras_[i]->AddTag(MM::g_Keyword_CameraChannelName, usedCameras_[i].c_str(),
                                     "");
         physicalCameras_[i]->AddTag(MM::g_Keyword_CameraChannelIndex, usedCameras_[i].c_str(),
                                     os.str().c_str());
      }
   }
   return DEVICE_OK;
}

int CMultiCameraPyCH::GetBinning() const
{
   int binning = 0;
   if (physicalCameras_[0] != 0)
      binning = physicalCameras_[0]->GetBinning();
   for (unsigned int i = 0; i < physicalCameras_.size(); i++)
   {
      if (physicalCameras_[i] != 0)
      {
         if (binning != physicalCameras_[i]->GetBinning())
            return 0;
      }
   }
   return binning;
}

int CMultiCameraPyCH::SetBinning(int bS)
{
   for (unsigned int i = 0; i < physicalCameras_.size(); i++)
   {
      if (physicalCameras_[i] != 0)
      {
         int ret = physicalCameras_[i]->SetBinning(bS);
         if (ret != DEVICE_OK)
            return ret;
      }
   }
   return DEVICE_OK;
}

int CMultiCameraPyCH::IsExposureSequenceable(bool& isSequenceable) const
{
   isSequenceable = false;

   return DEVICE_OK;
}

unsigned CMultiCameraPyCH::GetNumberOfComponents() const
{
   return 1;
}

unsigned CMultiCameraPyCH::GetNumberOfChannels() const
{
   return nrCamerasInUse_;
}

int CMultiCameraPyCH::GetChannelName(unsigned channel, char* name)
{
   CDeviceUtils::CopyLimitedString(name, "");
   int ch = Logical2Physical(channel);
   if (ch >= 0 && static_cast<unsigned>(ch) < usedCameras_.size())
   {
      CDeviceUtils::CopyLimitedString(name, usedCameras_[ch].c_str());
   }
   return DEVICE_OK;
}

int CMultiCameraPyCH::Logical2Physical(int logical)
{
   int j = -1;
   for (unsigned int i = 0; i < usedCameras_.size(); i++)
   {
      if (usedCameras_[i] != g_Undefined)
         j++;
      if (j == logical)
         return i;
   }
   return -1;
}


int CMultiCameraPyCH::OnPhysicalCamera(MM::PropertyBase* pProp, MM::ActionType eAct, long i)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(usedCameras_[i].c_str());
   }

   else if (eAct == MM::AfterSet)
   {
      if (physicalCameras_[i] != 0)
      {
         physicalCameras_[i]->RemoveTag(MM::g_Keyword_CameraChannelName);
         physicalCameras_[i]->RemoveTag(MM::g_Keyword_CameraChannelIndex);
      }

      std::string cameraName;
      pProp->Get(cameraName);

      if (cameraName == g_Undefined) {
         usedCameras_[i] = g_Undefined;
         physicalCameras_[i] = 0;
      } else {
         MM::Camera* camera = (MM::Camera*) GetDevice(cameraName.c_str());
         if (camera != 0) {
            usedCameras_[i] = cameraName;
            physicalCameras_[i] = camera;
            std::ostringstream os;
            os << i;
            char myName[MM::MaxStrLength];
            GetLabel(myName);
            camera->AddTag(MM::g_Keyword_CameraChannelName, myName, usedCameras_[i].c_str());
            camera->AddTag(MM::g_Keyword_CameraChannelIndex, myName, os.str().c_str());
         } else
            return ERR_INVALID_DEVICE_NAME;
      }
      nrCamerasInUse_ = 0;
      for (unsigned int i = 0; i < usedCameras_.size(); i++)
      {
         if (usedCameras_[i] != g_Undefined)
            nrCamerasInUse_++;
      }

      // TODO: Set allowed binning values correctly
      if (physicalCameras_[0] != 0)
      {
         ClearAllowedValues(MM::g_Keyword_Binning);
         int nr = physicalCameras_[0]->GetNumberOfPropertyValues(MM::g_Keyword_Binning);
         for (int j = 0; j < nr; j++)
         {
            char value[MM::MaxStrLength];
            physicalCameras_[0]->GetPropertyValueAt(MM::g_Keyword_Binning, j, value);
            AddAllowedValue(MM::g_Keyword_Binning, value);
         }
      }
   }

   return DEVICE_OK;
}

int CMultiCameraPyCH::OnBinning(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set((long)GetBinning());
   }
   else if (eAct == MM::AfterSet)
   {
      long binning;
      pProp->Get(binning);
      int ret = SetBinning(binning);
      if (ret != DEVICE_OK)
         return ret;
   }
   return DEVICE_OK;
}
