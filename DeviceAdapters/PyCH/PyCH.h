///////////////////////////////////////////////////////////////////////////////
#ifndef _PYCH_H
#define _PYCH_H
#include "../../MMDevice/DeviceBase.h"
#include "../../MMDevice/ImgBuffer.h"
#include "../../MMDevice/DeviceThreads.h"
#include "../Utilities/Utilities.h"
#include <string>
#include <map>
#include <algorithm>
#include <stdint.h>

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <iostream>

//////////////////////////////////////////////////////////////////////////////
// Error codes
//
#define ERR_UNKNOWN_MODE         102
#define ERR_UNKNOWN_POSITION     103
#define ERR_IN_SEQUENCE          104
#define ERR_SEQUENCE_INACTIVE    105
#define ERR_STAGE_MOVING         106
#define HUB_NOT_AVAILABLE        107

namespace p = boost::python;

namespace np = boost::python::numpy;

inline np::dtype dtype_conversion(int n) {
   switch(n) {
      case 1:
         return np::dtype::get_builtin<uint8_t>();
      case 2:
         return np::dtype::get_builtin<uint16_t>();
      case 4:
         return np::dtype::get_builtin<float>();
      default:
         return np::dtype::get_builtin<uint8_t>();
   }
}

class PythonImageCallback {
public:
   PythonImageCallback() {};
   ~PythonImageCallback() {};

   MM::Core *GetCoreCallback() { return callback_; }

   int Initialize(MM::Device *host, MM::Core *core);
   void processImage(ImgBuffer& img_);

   void processBuffer(unsigned char *buffer, int channels, int height, int width, int depth);


      void runScript(std::string name);

   void updateValuesXYZ();
   void updateValuesChannelDevice();
   void setChannelDevice(std::string channelDevice) { channelDevice_ = channelDevice; }
   std::string getChannelDevice() { return channelDevice_; }

private:
   std::string channelDevice_;

   MM::Device* host_;
   MM::Core* callback_;

   p::object main_module;
   p::object main_namespace;
};


class PyCHMultiCameraSnapThread : public MMDeviceThreadBase
{
public:
   PyCHMultiCameraSnapThread() :
         camera_(0),
         started_(false)
   {}

   ~PyCHMultiCameraSnapThread() { if (started_) wait(); }

   void SetCamera(MM::Camera* camera) { camera_ = camera; }

   int svc() { camera_->SnapImage(); return 0; }

   void Start() { activate(); started_ = true; }

private:
   MM::Camera* camera_;
   bool started_;
};





class PyCHMultiCameraSnapThread;


class CPyCHCamera : public CCameraBase<CPyCHCamera> {
public:
   CPyCHCamera() : CCameraBase(), bufferSize(0), buffer(NULL), channelCount_(1), width_(0), height_(0), bytes_(0) {
      CreateProperty("Width", "512", MM::Integer, false);
      CreateProperty("Height", "512", MM::Integer, false);

      CreateProperty("Bytes", "1", MM::Integer, false);
      CreateProperty("Binning", "1", MM::Integer, true);

      InitializeDefaultErrorMessages();

      SetErrorText(ERR_INVALID_DEVICE_NAME, "Please select a valid camera");
      SetErrorText(ERR_NO_PHYSICAL_CAMERA, "No physical camera assigned");

      CreateProperty(MM::g_Keyword_Name, "a", MM::String, true);
      CreateProperty(MM::g_Keyword_Description, "Description.", MM::String, true);

      CreateProperty("ChannelCount", "1", MM::Integer, false, new CPropertyAction(this, &CPyCHCamera::OnSetChannelCount), true);


   };
   ~CPyCHCamera() {};

   std::vector<std::string> cameraNames;
   std::vector<MM::Camera*> cameraDevices;
   std::vector<int> cameraSnapState;

   std::vector<size_t> selectedCamera;

   int Initialize() {
      unsigned int deviceIterator = 0;
      char deviceName[MM::MaxStrLength];

      while(true)
      {
         GetLoadedDeviceOfType(MM::CameraDevice, deviceName, deviceIterator++);

         std::string deviceNameStr = deviceName;
         if(deviceNameStr == "") {
            break;
         }

         MM::Camera *cam = (MM::Camera*)GetDevice(deviceNameStr.c_str());

         if (cam == this) {
            continue;
         }

         if(cam) {
            cameraNames.push_back(deviceNameStr);
            cameraDevices.push_back(cam);
            cameraSnapState.push_back(0);
         }
      }

      cameraNames.push_back("Empty Channel");
      cameraDevices.push_back(NULL);
      cameraSnapState.push_back(0);

      /* ******************************************************************** */

      for(long i = 0; i < channelCount_; i++) {
         std::ostringstream os;
         os << "Camera " << (i+1);
         CreateProperty(os.str().c_str(), "Empty Channel", MM::String, false, new CPropertyActionEx(this, &CPyCHCamera::OnSetChannel, i), false);
         SetAllowedValues(os.str().c_str(), cameraNames);
         selectedCamera[i] = cameraDevices.size() - 1;
      }


      CreateProperty("ScriptPath", "", MM::String, false, new CPropertyAction(this, &CPyCHCamera::OnScript), false);

      pyc_.Initialize(this, GetCoreCallback());



      return DEVICE_OK;
   };

   long channelCount_;

   int OnSetChannelCount(MM::PropertyBase *pProp, MM::ActionType eAct) {
      if(eAct == MM::BeforeGet)
      {
         pProp->Set(channelCount_);
      }
      else if(eAct == MM::AfterSet) {
         long count;
         pProp->Get(count);
         if(count != channelCount_) {
            FreeBuffers();
            channelCount_ = count;
            selectedCamera.resize(channelCount_);
            CreateBuffers();
         }
      }
      return DEVICE_OK;
   }

   int OnSetChannel(MM::PropertyBase *pProp, MM::ActionType eAct, long channel) {
      if(eAct == MM::BeforeGet)
      {
         pProp->Set(cameraNames[selectedCamera[channel]].c_str());
      }
      else if(eAct == MM::AfterSet) {
         std::string cameraName;
         pProp->Get(cameraName);

         long index = (std::find(cameraNames.begin(), cameraNames.end(), cameraName) - cameraNames.begin());

         assert(index >= 0);

         MM::Camera *camera = cameraDevices[selectedCamera[channel]];

         if(camera != cameraDevices[index]) {
            /*
            if (camera) {
               camera->RemoveTag(MM::g_Keyword_CameraChannelName);
               camera->RemoveTag(MM::g_Keyword_CameraChannelIndex)
            }
            // add new labels ... (look at multicam logic)
            */

            selectedCamera[(size_t)channel] = index;

         }


         width_ = getLongProperty("Width");
         height_ = getLongProperty("Height");
         bytes_ =  getLongProperty("Bytes");

         for(int i = 0; i < selectedCamera.size(); i++) {
            MM::Camera *camera = cameraDevices[selectedCamera[i]];
            if(!camera)
               continue;

            width_ = std::max(width_, (long)camera->GetImageWidth());
            height_ = std::max(height_, (long)camera->GetImageHeight());
            bytes_ = std::max(bytes_, (long)camera->GetImageBytesPerPixel());

         }

         FreeBuffers();
         CreateBuffers();

      }
      return DEVICE_OK;
   }

   unsigned char *buffer;
   long planeSize;

   void FreeBuffers() {
      if (buffer) {
         delete[] buffer;
         buffer = NULL;
      }
   }

   void Empty() {
      if(buffer) {
         memset(buffer, 0, planeSize * channelCount_);
      }
   }

   void CreateBuffers() {
      planeSize = GetImageWidth() * GetImageHeight() * GetImageBytesPerPixel();
      buffer = new unsigned char[planeSize * channelCount_];
   }

   int Shutdown() { return DEVICE_OK; };
   void GetName(char *name) const { CDeviceUtils::CopyLimitedString(name, "PyCHCamera"); };

   long GetImageBufferSize() const { return planeSize; }

   unsigned GetNumberOfChannels() const { return channelCount_; }

   // TODO override this meaningfully TODO
   int GetChannelName(unsigned channel, char* name) { CDeviceUtils::CopyLimitedString(name, ""); return DEVICE_OK; }

   unsigned GetBitDepth() const { return 8 * GetImageBytesPerPixel(); }
   int GetBinning() const { return 1; }
   int SetBinning(int binSize) { return DEVICE_OK; }
   void SetExposure(double exp_ms) { return ; }
   double GetExposure() const { return 1.0; }
   int SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize) { return DEVICE_OK; }
   int GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize) { x = 0; y = 0; xSize = GetImageWidth(); ySize = GetImageHeight(); return DEVICE_OK; }

   int ClearROI() { return DEVICE_OK; }

   int IsExposureSequenceable(bool& isSequenceable) const { isSequenceable = false; return DEVICE_OK; }

   const unsigned char* GetImageBuffer(unsigned channel) {
      return buffer + planeSize*channel;
   }

   const unsigned char* GetImageBuffer() {
      return GetImageBuffer(0);
   }

   long width_, height_, bytes_;

   unsigned GetImageWidth() const { return getLongProperty("Width"); }

   unsigned GetImageHeight() const { return getLongProperty("Height"); }

   unsigned GetImageBytesPerPixel() const { return getLongProperty("Bytes"); }

   int SnapImage() {
      for(size_t i = 0; i < cameraSnapState.size(); i++) {
         cameraSnapState[i] = 0;
      }

      {
         // TODO inefficient to allocate and deallocate
         std::vector<PyCHMultiCameraSnapThread*> camthreads(channelCount_);

         long c = 0;
         for (size_t i = 0; i < selectedCamera.size(); i++) {
            if(cameraSnapState[selectedCamera[i]])
               continue;

            if(!cameraDevices[selectedCamera[i]])
               continue;

            camthreads[c] = new PyCHMultiCameraSnapThread();

            camthreads[c]->SetCamera(cameraDevices[selectedCamera[i]]);
            cameraSnapState[selectedCamera[i]] = 1;

            camthreads[c]->Start();

            c++;
         }

         for(size_t i = 0; i < c; i++)
            delete camthreads[c];
      }

      for (size_t i = 0; i < selectedCamera.size(); i++) {
         MM::Camera *camera = cameraDevices[selectedCamera[i]];
         if(camera == NULL) {
            // empty camera, create empty buffer
            memset((void*)GetImageBuffer(i), 0, planeSize);
         } else {
            if(camera->GetImageWidth() == width_) {
               if(camera->GetImageHeight() == height_) {
                  assert(planeSize == camera->GetImageBufferSize());
                  memcpy((void*)GetImageBuffer(i), camera->GetImageBuffer(), camera->GetImageBufferSize());
               } else {
                  const unsigned char* otherBuffer = camera->GetImageBuffer();
                  assert(0 && "Currently not implemented");
               }
            } else {
               assert(0 && "Currently not implemented");
            }
         }
      }

      pyc_.processBuffer(buffer, channelCount_, GetImageHeight(), GetImageWidth(), GetImageBytesPerPixel());

      return DEVICE_OK;

   };

   int OnScript(MM::PropertyBase *pProp, MM::ActionType eAct) {
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



private:

   long getLongProperty(const char *name) const {
      char buf[MM::MaxStrLength];
      GetProperty(name, buf);
      return atoi(buf);
   }
   PythonImageCallback pyc_;

   std::string scriptFile_;

   long bufferSize;
};











class CEmptyCam : public CCameraBase<CEmptyCam> {
public:
   CEmptyCam() : CCameraBase(), bufferSize(0), buffer(NULL) {
      CreateProperty("Width", "512", MM::Integer, false);
      CreateProperty("Height", "512", MM::Integer, false);

      CreateProperty("Bytes", "1", MM::Integer, false);
      CreateProperty("Binning", "1", MM::Integer, true);

   };
   ~CEmptyCam() {};

   int Initialize() { return DEVICE_OK; };
   int Shutdown() { return DEVICE_OK; };
   void GetName(char *name) const { CDeviceUtils::CopyLimitedString(name, "EmptyCam"); };

   long GetImageBufferSize() const { return bufferSize; }
   unsigned GetBitDepth() const { return 8 * GetImageBytesPerPixel(); }
   int GetBinning() const { return 1; }
   int SetBinning(int binSize) { return DEVICE_OK; }
   void SetExposure(double exp_ms) { return ; }
   double GetExposure() const { return 1.0; }
   int SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize) { return DEVICE_OK; }
   int GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize) { x = 0; y = 0; xSize = GetImageWidth(); ySize = GetImageHeight(); return DEVICE_OK; }

   int ClearROI() { return DEVICE_OK; }

   int IsExposureSequenceable(bool& isSequenceable) const { isSequenceable = false; return DEVICE_OK; }


   const unsigned char* GetImageBuffer() {
      long currentSize = GetImageWidth() * GetImageHeight() * GetImageBytesPerPixel();
      if (currentSize != bufferSize) {
         if (buffer) {
            // potential double free if malloc fails
            delete buffer;
         }

         buffer = new unsigned char[currentSize];
         bufferSize = currentSize;

         memset(buffer, 0, bufferSize);
      }

      return buffer;
   }

   unsigned GetImageWidth() const { return getLongProperty("Width"); }

   unsigned GetImageHeight() const { return getLongProperty("Height"); }

   unsigned GetImageBytesPerPixel() const { return getLongProperty("Bytes"); }

   int SnapImage() { return DEVICE_OK; };
private:


   long getLongProperty(const char *name) const {
      char buf[MM::MaxStrLength];
      GetProperty(name, buf);
      return atoi(buf);
   }

   unsigned char *buffer;
   long bufferSize;
};

class MySequenceThread;
class CPyCH : public CCameraBase<CPyCH>
{
public:
   CPyCH();
   ~CPyCH();

   // MMDevice API
   // ------------
   int Initialize();
   int Shutdown();

   void GetName(char* name) const;

   // MMCamera API
   // ------------
   int SnapImage();
   const unsigned char* GetImageBuffer();
   unsigned GetImageWidth() const;
   unsigned GetImageHeight() const;
   unsigned GetImageBytesPerPixel() const;
   unsigned GetBitDepth() const;
   long GetImageBufferSize() const;
   double GetExposure() const;
   void SetExposure(double exp);
   int SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize);
   int GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize);
   int ClearROI();
   int PrepareSequenceAcqusition() { return DEVICE_OK; }
   int StartSequenceAcquisition(double interval);
   int StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow);
   int StopSequenceAcquisition();
   int InsertImage();
   int RunSequenceOnThread(MM::MMTime startTime);
   bool IsCapturing();
   void OnThreadExiting() throw();
   double GetNominalPixelSizeUm() const {return nominalPixelSizeUm_;}
   double GetPixelSizeUm() const {return nominalPixelSizeUm_ * GetBinning();}
   int GetBinning() const;
   int SetBinning(int bS);
   int IsExposureSequenceable(bool& isSequenceable) const;
   int GetExposureSequenceMaxLength(long& nrEvents) { nrEvents = 0; return DEVICE_OK; };
   int StartExposureSequence() { return DEVICE_ERR; }
   int StopExposureSequence() { return DEVICE_ERR; }
   int ClearExposureSequence() { return DEVICE_ERR; }
   int AddToExposureSequence(double exposureTime_ms) { return DEVICE_ERR; }
   int SendExposureSequence() { return DEVICE_ERR; }
   unsigned  GetNumberOfComponents() const { return nComponents_;};
   // action interface
   // ----------------
   int OnScript(MM::PropertyBase *pProp, MM::ActionType eAct);
   int OnChannelDevice(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnCameraDevice(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnPixelType(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnBitDepth(MM::PropertyBase* pProp, MM::ActionType eAct);
   int OnCameraCCDXSize(MM::PropertyBase* , MM::ActionType );
   int OnCameraCCDYSize(MM::PropertyBase* , MM::ActionType );
private:
   bool PerformCallback(ImgBuffer &img);
   int ResizeImageBuffer();
   static const double nominalPixelSizeUm_;

   PythonImageCallback pyc_;

   std::string scriptFile_;
   std::string channelDevice_;
   std::string cameraDevice_;

   ImgBuffer img_;
   bool initialized_;
   double readoutUs_;
   MM::MMTime readoutStartTime_;
   int bitDepth_;
   unsigned roiX_;
   unsigned roiY_;
   MM::MMTime sequenceStartTime_;
   long imageCounter_;
   long binSize_;
   long cameraCCDXSize_;
   long cameraCCDYSize_;
   std::string triggerDevice_;
   bool stopOnOverflow_;
   MMThreadLock imgPixelsLock_;
   friend class MySequenceThread;
   int nComponents_;
   MySequenceThread * thd_;
};

class MySequenceThread : public MMDeviceThreadBase
{
   friend class CPyCH;
   enum { default_numImages=1, default_intervalMS = 100 };
public:
   MySequenceThread(CPyCH* pCam);
   ~MySequenceThread();
   void Stop();
   void Start(long numImages, double intervalMs);
   bool IsStopped();
   void Suspend();
   bool IsSuspended();
   void Resume();
   double GetIntervalMs(){return intervalMs_;}
   void SetLength(long images) {numImages_ = images;}
   long GetLength() const {return numImages_;}
   long GetImageCounter(){return imageCounter_;}
   MM::MMTime GetStartTime(){return startTime_;}
   MM::MMTime GetActualDuration(){return actualDuration_;}
private:
   int svc(void) throw();
   double intervalMs_;
   long numImages_;
   long imageCounter_;
   bool stop_;
   bool suspend_;
   CPyCH* camera_;
   MM::MMTime startTime_;
   MM::MMTime actualDuration_;
   MM::MMTime lastFrameTime_;
   MMThreadLock stopLock_;
   MMThreadLock suspendLock_;
};


#define MAX_NUMBER_PHYSICAL_CAMERAS       5


#define ERR_INVALID_DEVICE_NAME            10001

#define ERR_NO_PHYSICAL_CAMERA             10010
#define ERR_NO_EQUAL_SIZE                  10011


/**
 * CameraSnapThread: helper thread for MultiCamera
 */

/*
 * CMultiCameraPyCH: Combines multiple physical cameras into one logical device
 */
class CMultiCameraPyCH : public CCameraBase<CMultiCameraPyCH>
{
public:
   CMultiCameraPyCH();
   ~CMultiCameraPyCH();

   int Initialize();
   int Shutdown();

   void GetName(char* name) const;

   int SnapImage();
   const unsigned char* GetImageBuffer();
   const unsigned char* GetImageBuffer(unsigned channelNr);
   unsigned GetImageWidth() const;
   unsigned GetImageHeight() const;
   unsigned GetImageBytesPerPixel() const;
   unsigned GetBitDepth() const;
   long GetImageBufferSize() const;
   double GetExposure() const;
   void SetExposure(double exp);
   int SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize);
   int GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize);
   int ClearROI();
   int PrepareSequenceAcqusition();
   int StartSequenceAcquisition(double interval);
   int StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow);
   int StopSequenceAcquisition();
   int GetBinning() const;
   int SetBinning(int bS);
   int IsExposureSequenceable(bool& isSequenceable) const;
   unsigned  GetNumberOfComponents() const;
   unsigned  GetNumberOfChannels() const;
   int GetChannelName(unsigned channel, char* name);
   bool IsCapturing();

   // action interface
   // ---------------
   int OnPhysicalCamera(MM::PropertyBase* pProp, MM::ActionType eAct, long nr);
   int OnBinning(MM::PropertyBase* pProp, MM::ActionType eAct);

   int OnScript(MM::PropertyBase *pProp, MM::ActionType eAct);

      private:
   int Logical2Physical(int logical);
   bool ImageSizesAreEqual();
   unsigned char* imageBuffer_;

   std::string scriptFile_;
   PythonImageCallback pyc_;


   std::vector<std::string> availableCameras_;
   std::vector<std::string> usedCameras_;
   std::vector<int> cameraWidths_;
   std::vector<int> cameraHeights_;
   std::vector<MM::Camera*> physicalCameras_;
   unsigned int nrCamerasInUse_;
   bool initialized_;
   ImgBuffer img_;
};



#endif //_PYCH_H_
