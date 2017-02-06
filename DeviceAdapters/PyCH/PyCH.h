///////////////////////////////////////////////////////////////////////////////
#ifndef _PYCH_H
#define _PYCH_H

#include "../../MMDevice/DeviceBase.h"
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

namespace p = boost::python;

namespace np = boost::python::numpy;

template<class O, class I>
inline O convert(I input) {
   std::stringstream s;
   s << input;
   O output;
   s >> output;
   return output;
};

inline np::dtype dtype_conversion(int n) {
   switch (n) {
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
   PythonImageCallback() : stateDevice_(0) {};
   ~PythonImageCallback() {};

   int Initialize(MM::Device *host, MM::Core *core);
   void bindBuffer(unsigned char *buffer, int channels, int height, int width, int depth);
   void execute();
   void runScript(std::string name);
   void updateValuesXYZ();
   void updateValuesChannelDevice();
   void setChannelDevice(std::string channelDevice);
   std::string getChannelDevice();

   MM::Core *GetCoreCallback();

private:

   MM::State *stateDevice_;

   std::string channelDevice_;

   MM::Device *host_;
   MM::Core *callback_;

   p::object main_module;
   p::object main_namespace;
};


class PyCHMultiCameraSnapThread : public MMDeviceThreadBase {
public:
   PyCHMultiCameraSnapThread() :
         camera_(0),
         started_(false) {}

   ~PyCHMultiCameraSnapThread() { if (started_) wait(); }

   void SetCamera(MM::Camera *camera) { camera_ = camera; }

   int svc() {
      camera_->SnapImage();
      return 0;
   }

   void Start() {
      activate();
      started_ = true;
   }

private:
   MM::Camera *camera_;
   bool started_;
};



class CPyCHCamera : public CCameraBase<CPyCHCamera> {
public:
   CPyCHCamera() : CCameraBase(), buffer_(NULL), channelCount_(1), width_(0), height_(0), bytes_(0) {
      CreateProperty("Width", "512", MM::Integer, false);
      CreateProperty("Height", "512", MM::Integer, false);

      CreateProperty("Bytes", "1", MM::Integer, false);
      CreateProperty("Binning", "1", MM::Integer, true);

      InitializeDefaultErrorMessages();

      SetErrorText(ERR_INVALID_DEVICE_NAME, "Please select a valid camera");
      SetErrorText(ERR_NO_PHYSICAL_CAMERA, "No physical camera assigned");

      CreateProperty(MM::g_Keyword_Name, "a", MM::String, true);
      CreateProperty(MM::g_Keyword_Description, "Description.", MM::String, true);

      CreateProperty("ChannelCount", "1", MM::Integer, false,
                     new CPropertyAction(this, &CPyCHCamera::OnSetChannelCount), true);


   };

   ~CPyCHCamera() {};

   int Initialize() {
      unsigned int deviceIterator = 0;
      char deviceName[MM::MaxStrLength];

      while (true) {
         GetLoadedDeviceOfType(MM::CameraDevice, deviceName, deviceIterator++);

         std::string deviceNameStr = deviceName;
         if (deviceNameStr == "") {
            break;
         }

         MM::Camera *camera = (MM::Camera *) GetDevice(deviceNameStr.c_str());

         if (camera == this) {
            continue;
         }

         if (camera) {
            cameraNames_.push_back(deviceNameStr);
            cameraDevices_.push_back(camera);
            cameraSnapstate_.push_back(0);
         }
      }

      cameraNames_.push_back("Empty Channel");
      cameraDevices_.push_back(NULL);
      cameraSnapstate_.push_back(0);

      /* ******************************************************************** */

      for (long i = 0; i < channelCount_; i++) {
         std::string cameraName = "Camera " + convert<std::string>(i + 1);
         CreateProperty(cameraName.c_str(), "Empty Channel", MM::String, false,
                        new CPropertyActionEx(this, &CPyCHCamera::OnSetChannel, i), false);
         SetAllowedValues(cameraName.c_str(), cameraNames_);
         selectedCamera_[i] = cameraDevices_.size() - 1;
      }


      CreateProperty("ScriptPath", "", MM::String, false, new CPropertyAction(this, &CPyCHCamera::OnScript), false);

      pyc_.Initialize(this, GetCoreCallback());


      return DEVICE_OK;
   };


   int OnSetChannelCount(MM::PropertyBase *pProp, MM::ActionType eAct) {
      if (eAct == MM::BeforeGet) {
         pProp->Set(channelCount_);
      } else if (eAct == MM::AfterSet) {
         long count;
         pProp->Get(count);
         if (count != channelCount_) {
            FreeBuffers();
            channelCount_ = count;
            selectedCamera_.resize(channelCount_);
            CreateBuffers();
         }
      }
      return DEVICE_OK;
   }

   int OnSetChannel(MM::PropertyBase *pProp, MM::ActionType eAct, long channel) {
      if (eAct == MM::BeforeGet) {
         pProp->Set(cameraNames_[selectedCamera_[channel]].c_str());
      } else if (eAct == MM::AfterSet) {
         std::string cameraName;
         pProp->Get(cameraName);

         long index = (std::find(cameraNames_.begin(), cameraNames_.end(), cameraName) - cameraNames_.begin());

         assert(index >= 0);

         MM::Camera *camera = cameraDevices_[selectedCamera_[channel]];

         if (camera != cameraDevices_[index]) {

            if (camera) {
               camera->RemoveTag(MM::g_Keyword_CameraChannelName);
               camera->RemoveTag(MM::g_Keyword_CameraChannelIndex);
            }

            char myName[MM::MaxStrLength];
            GetLabel(myName);
            camera->AddTag(MM::g_Keyword_CameraChannelName, myName, cameraNames_[selectedCamera_[channel]].c_str());
            camera->AddTag(MM::g_Keyword_CameraChannelIndex, myName, convert<std::string>(channel).c_str());

            selectedCamera_[(size_t) channel] = index;
         }


         width_ = GetLongProperty("Width");
         height_ = GetLongProperty("Height");
         bytes_ = GetLongProperty("Bytes");

         for (int i = 0; i < selectedCamera_.size(); i++) {
            MM::Camera *camera = cameraDevices_[selectedCamera_[i]];
            if (!camera)
               continue;

            width_ = std::max(width_, (long) camera->GetImageWidth());
            height_ = std::max(height_, (long) camera->GetImageHeight());
            bytes_ = std::max(bytes_, (long) camera->GetImageBytesPerPixel());

         }

         FreeBuffers();
         CreateBuffers();

      }
      return DEVICE_OK;
   }


   int Shutdown() { return DEVICE_OK; };

   void GetName(char *name) const { CDeviceUtils::CopyLimitedString(name, "PyCHCamera"); };

   long GetImageBufferSize() const { return planeSize_; }

   unsigned GetNumberOfChannels() const { return channelCount_; }

   int GetChannelName(unsigned channel, char *name) {
      CDeviceUtils::CopyLimitedString(name, cameraNames_[selectedCamera_[channel]].c_str());
      return DEVICE_OK;
   }

   unsigned GetBitDepth() const { return 8 * GetImageBytesPerPixel(); }

   int GetBinning() const { return 1; }

   int SetBinning(int binSize) { return DEVICE_OK; }

   void SetExposure(double exp_ms) { return; }

   double GetExposure() const { return 1.0; }

   int SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize) { return DEVICE_OK; }

   int GetROI(unsigned &x, unsigned &y, unsigned &xSize, unsigned &ySize) {
      x = 0;
      y = 0;
      xSize = GetImageWidth();
      ySize = GetImageHeight();
      return DEVICE_OK;
   }

   int ClearROI() { return DEVICE_OK; }

   int IsExposureSequenceable(bool &isSequenceable) const {
      isSequenceable = false;
      return DEVICE_OK;
   }

   const unsigned char *GetImageBuffer(unsigned channel) {
      return buffer_ + planeSize_ * channel;
   }

   const unsigned char *GetImageBuffer() {
      return GetImageBuffer(0);
   }



   unsigned GetImageWidth() const { return GetLongProperty("Width"); }

   unsigned GetImageHeight() const { return GetLongProperty("Height"); }

   unsigned GetImageBytesPerPixel() const { return GetLongProperty("Bytes"); }

   int SnapImage() {
      for (size_t i = 0; i < cameraSnapstate_.size(); i++) {
         cameraSnapstate_[i] = 0;
      }

      {
         // TODO inefficient to allocate and deallocate
         std::vector<PyCHMultiCameraSnapThread *> cameraThreads(channelCount_);

         long c = 0;
         for (size_t i = 0; i < selectedCamera_.size(); i++) {
            if (cameraSnapstate_[selectedCamera_[i]])
               continue;

            if (!cameraDevices_[selectedCamera_[i]])
               continue;

            cameraThreads[c] = new PyCHMultiCameraSnapThread();

            cameraThreads[c]->SetCamera(cameraDevices_[selectedCamera_[i]]);
            cameraSnapstate_[selectedCamera_[i]] = 1;

            cameraThreads[c]->Start();

            c++;
         }

         for (size_t i = 0; i < c; i++) {
            std::cout << "Should wait for " << i << cameraThreads[c] << " " << std::endl;
            delete cameraThreads[c];
         }
      }

      for (size_t i = 0; i < selectedCamera_.size(); i++) {
         MM::Camera *camera = cameraDevices_[selectedCamera_[i]];
         if (camera == NULL) {
            // empty camera, create empty buffer
            std::cout << "Zeroing " << (size_t) GetImageBuffer(i) << std::endl;
            memset((unsigned char *) GetImageBuffer(i), 0, planeSize_);
         } else {
            if (camera->GetImageWidth() == width_) {
               if (camera->GetImageHeight() == height_) {
                  assert(planeSize_ == camera->GetImageBufferSize());
                  std::cout << "Copying " << (size_t) GetImageBuffer(i) << std::endl;
                  memcpy((unsigned char *) GetImageBuffer(i), camera->GetImageBuffer(), camera->GetImageBufferSize());
               } else {
                  const unsigned char *otherBuffer = camera->GetImageBuffer();
                  memcpy((unsigned char *) GetImageBuffer(i), camera->GetImageBuffer(), camera->GetImageBufferSize());
                  memset((unsigned char *) GetImageBuffer(i), camera->GetImageBufferSize(),
                         planeSize_ - camera->GetImageBufferSize());
               }
            } else {
               memset((void *) GetImageBuffer(i), 0, planeSize_);
               for (int row = 0; row < camera->GetImageHeight(); row++) {
                  memcpy(
                        (unsigned char *) GetImageBuffer(i) + row * GetImageWidth() * GetImageBytesPerPixel(),
                        camera->GetImageBuffer() + row * camera->GetImageWidth() * camera->GetImageBytesPerPixel(),
                        std::min(GetImageWidth() * GetImageBytesPerPixel(),
                                 camera->GetImageWidth() * camera->GetImageBytesPerPixel())
                  );
               }
            }
         }
      }

      pyc_.execute();

      return DEVICE_OK;

   };

   int OnScript(MM::PropertyBase *pProp, MM::ActionType eAct) {
      if (eAct == MM::BeforeGet) {
         pProp->Set(scriptFile_.c_str());
      } else if (eAct == MM::AfterSet) {
         pProp->Get(scriptFile_);
         if (scriptFile_ != "") {
            pyc_.runScript(scriptFile_);
         }
      }
      return DEVICE_OK;
   }

private:

   long GetLongProperty(const char *name) const {
      char buf[MM::MaxStrLength];
      GetProperty(name, buf);
      return atoi(buf);
   }


   void FreeBuffers() {
      if (buffer_) {
         delete[] buffer_;
         buffer_ = NULL;
      }
   }

   void Empty() {
      if (buffer_) {
         memset(buffer_, 0, planeSize_ * channelCount_);
      }
   }

   void CreateBuffers() {
      planeSize_ = GetImageWidth() * GetImageHeight() * GetImageBytesPerPixel();
      buffer_ = new unsigned char[planeSize_ * channelCount_];

      pyc_.bindBuffer(buffer_, channelCount_, GetImageHeight(), GetImageWidth(), GetImageBytesPerPixel());
   }


   long channelCount_, width_, height_, bytes_;
   long planeSize_;

   unsigned char *buffer_;

   std::vector<std::string> cameraNames_;
   std::vector<MM::Camera *> cameraDevices_;
   std::vector<bool> cameraSnapstate_;

   std::vector<size_t> selectedCamera_;

   std::string scriptFile_;
   PythonImageCallback pyc_;
};

#endif //_PYCH_H_
