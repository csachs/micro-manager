///////////////////////////////////////////////////////////////////////////////
#ifndef _PYCH_H
#define _PYCH_H
#include "../../MMDevice/DeviceBase.h"
#include "../../MMDevice/ImgBuffer.h"
#include "../../MMDevice/DeviceThreads.h"
#include <string>
#include <map>
#include <algorithm>
#include <stdint.h>

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>

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


class CEmptyCam : public CCameraBase<CEmptyCam> {
public:
   CEmptyCam() : CCameraBase(), bufferSize(0), buffer(NULL) {
      CreateProperty("Width", "512", MM::Integer, false);
      CreateProperty("Height", "512", MM::Integer, false);

      CreateProperty("Bytes", "1", MM::Integer, false);

   };
   ~CEmptyCam() {};

   int Initialize() {};
   int Shutdown() {};
   void GetName(char *name) const {/* TODO */};




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

   int SnapImage() { return true; };
private:
   inline long getLongProperty(const char *name) {
      long l;
      GetProperty(name, l);
      return l;
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


#define MAX_NUMBER_PHYSICAL_CAMERAS       4


#define ERR_INVALID_DEVICE_NAME            10001

#define ERR_NO_PHYSICAL_CAMERA             10010
#define ERR_NO_EQUAL_SIZE                  10011


/**
 * CameraSnapThread: helper thread for MultiCamera
 */
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

private:
   int Logical2Physical(int logical);
   bool ImageSizesAreEqual();
   unsigned char* imageBuffer_;

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
