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

#ifdef _WIN32
#else
#include <dlfcn.h>
#include <iostream>

#endif

const char *g_CameraDeviceName = "PyCHCamera";

///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////

MODULE_API void InitializeModuleData() {
   RegisterDevice(g_CameraDeviceName, MM::CameraDevice, "PyCHCamera");
}

MODULE_API MM::Device *CreateDevice(const char *deviceName) {
   if (deviceName == 0)
      return 0;
   if (strcmp(deviceName, g_CameraDeviceName) == 0) {
      return new CPyCHCamera();
   }
   return 0;
}

MODULE_API void DeleteDevice(MM::Device *pDevice) {
   delete pDevice;
}

///////////////////////////////////////////////////////////////////////////////

int PythonImageCallback::Initialize(MM::Device *host, MM::Core *core) {
   host_ = host;
   callback_ = core;

   /*
    * Initialize Python
    */

   // Python's shared library needs to be loaded, so the linker
   // can find symbols from libraries which in turn will be later loaded by python
   #ifdef _WIN32
   #else
   dlopen(PYTHON_SHARED_OBJECT, RTLD_LAZY | RTLD_GLOBAL);
   #endif

   try {

      Py_Initialize();

      main_module = p::import("__main__");
      main_namespace = main_module.attr("__dict__");

      np::initialize();

      std::string python_startup = ""
            "_image_buffer = 0\n"
            "_x = 0.0\n"
            "_y = 0.0\n"
            "_z = 0.0\n"
            "_channel = 0\n"
            "_parameters = {}\n"
            "def _set_parameters(s):\n"
            "    import json\n"
            "    global _parameters\n"
            "    _parameters = json.loads(s)\n"
            "def callback(image_buffer, **kwargs): pass\n"
            "def _callback():\n"
            "    return callback(_image_buffer, x=_x, y=_y, z=_z, channel=_channel, parameters=_parameters)\n"
            "";

      p::exec(python_startup.c_str(), main_namespace);
   } catch (p::error_already_set const &) {
      PyErr_Print();
   }

   return DEVICE_OK;
}

void PythonImageCallback::RunScript(std::string name) {
   try {
      p::exec_file(name.c_str(), main_namespace);
   } catch (p::error_already_set const &) {
      PyErr_Print();
   }
}

void PythonImageCallback::UpdateValuesXYZ() {
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

void PythonImageCallback::UpdateValuesChannelDevice() {
   long channel = 0;

   if (stateDevice_) {
      stateDevice_->GetPosition(channel);
   }

   try {
      main_namespace["_channel"] = channel;
   } catch (p::error_already_set const &) {
      PyErr_Print();
   }
}

std::string PythonImageCallback::GetChannelDevice() { return channelDevice_; }

void PythonImageCallback::SetChannelDevice(std::string channelDevice) {
   channelDevice_ = channelDevice;
   if (channelDevice_.length() > 0) {
      stateDevice_ = GetCoreCallback()->GetStateDevice(host_, channelDevice_.c_str());
   } else {
      stateDevice_ = NULL;
   }
}

std::string PythonImageCallback::GetParameters() { return parametersStr_; }

void PythonImageCallback::SetParameters(std::string parametersStr) {
   parametersStr_ = parametersStr;
   try {
      main_namespace["_set_parameters"](parametersStr_);
   } catch (p::error_already_set const &) {
      PyErr_Print();
   }
}


MM::Core *PythonImageCallback::GetCoreCallback() { return callback_; }

void PythonImageCallback::BindBuffer(unsigned char *buffer, size_t channels, size_t height, size_t width, size_t depth) {
   try {
      np::ndarray array = np::from_data(
            buffer,
            dtype_conversion(depth),
            p::make_tuple(channels, height, width),
            p::make_tuple(height * width * depth, width * depth, depth),
            p::object()
      );
      main_namespace["_image_buffer"] = array;
      bound = true;
   } catch (p::error_already_set const &) {
      PyErr_Print();
   }
}

void PythonImageCallback::UnbindBuffer() {
   try {
      main_namespace["_image_buffer"] = 0;
      bound = false;
   } catch (p::error_already_set const &) {
      PyErr_Print();
   }
}

bool PythonImageCallback::IsBound() {
   return bound;
}

void PythonImageCallback::Execute() {
   try {
      main_namespace["_callback"]();
   } catch (p::error_already_set const &) {
      PyErr_Print();
   }
}

CPyCHCamera::CPyCHCamera() : CCameraBase(), buffer_(NULL), channelCount_(1), width_(0), height_(0), bytes_(0) {
   CreateProperty("Width", "512", MM::Integer, false);
   CreateProperty("Height", "512", MM::Integer, false);

   CreateProperty("Bytes", "1", MM::Integer, false);
   CreateProperty("Binning", "1", MM::Integer, true);

   InitializeDefaultErrorMessages();

   CreateProperty(MM::g_Keyword_Name, "PyCH", MM::String, true);
   CreateProperty(MM::g_Keyword_Description, "Python Camera Hook, create or modify image data with Python.", MM::String, true);

   CreateProperty("ChannelCount", "1", MM::Integer, false,
                  new CPropertyAction(this, &CPyCHCamera::OnChannelCount), true);


   updateChannelCount(1); // initialize internal data structures

};

CPyCHCamera::~CPyCHCamera() {

};

inline std::vector<std::string> CPyCHCamera::GetDevicesOfType(MM::DeviceType type) {
   std::vector<std::string> result;

   unsigned int deviceIterator = 0;
   char deviceName[MM::MaxStrLength];
   while (true) {
      GetLoadedDeviceOfType(type, deviceName, deviceIterator++);

      std::string deviceNameStr = deviceName;
      if (deviceNameStr == "") {
         break;
      }

      result.push_back(deviceNameStr);
   }

   return result;
}

int CPyCHCamera::Initialize() {
   std::vector<std::string> cameras = GetDevicesOfType(MM::CameraDevice);

   for(size_t i = 0; i < cameras.size(); i++) {
      MM::Camera *camera = (MM::Camera *) GetDevice(cameras[i].c_str());

      if (camera == this || !camera) {
         continue;
      }

      cameraNames_.push_back(cameras[i]);
      cameraDevices_.push_back(camera);
      cameraSnapstate_.push_back(0);
   }

   cameraNames_.push_back("Empty Channel");
   cameraDevices_.push_back(NULL);
   cameraSnapstate_.push_back(0);

   /* ******************************************************************** */

   for (long i = 0; i < channelCount_; i++) {
      std::string cameraName = "Camera " + convert<std::string>(i + 1);
      CreateProperty(cameraName.c_str(), "Empty Channel", MM::String, false,
                     new CPropertyActionEx(this, &CPyCHCamera::OnChannel, i), false);
      SetAllowedValues(cameraName.c_str(), cameraNames_);
      selectedCamera_[i] = cameraDevices_.size() - 1;
   }

   CreateProperty("ScriptPath", "", MM::String, false, new CPropertyAction(this, &CPyCHCamera::OnScript), false);

   CreateProperty("ScriptParameters", "", MM::String, false, new CPropertyAction(this, &CPyCHCamera::OnParameters), false);

   CreateProperty("ChannelDevice", "", MM::String, false, new CPropertyAction(this, &CPyCHCamera::OnScriptChannelDevice), false);

   channelDevices_ = GetDevicesOfType(MM::StateDevice);
   SetAllowedValues("ChannelDevice", channelDevices_);



   pyc_.Initialize(this, GetCoreCallback());


   return DEVICE_OK;
};

int CPyCHCamera::OnScriptChannelDevice(MM::PropertyBase *pProp, MM::ActionType eAct) {
   if (eAct == MM::BeforeGet) {
      pProp->Set(pyc_.GetChannelDevice().c_str());
   } else if (eAct == MM::AfterSet) {
      std::string channelDevice;
      pProp->Get(channelDevice);
      pyc_.SetChannelDevice(channelDevice);
   }
   return DEVICE_OK;
}

int CPyCHCamera::OnChannelCount(MM::PropertyBase *pProp, MM::ActionType eAct) {
   if (eAct == MM::BeforeGet) {
      pProp->Set((long int)channelCount_);
   } else if (eAct == MM::AfterSet) {
      long count;
      pProp->Get(count);
      if (count != channelCount_) {
         updateChannelCount(count);
      }
   }
   return DEVICE_OK;
}

void CPyCHCamera::updateChannelCount(long count) {
   FreeBuffers();
   channelCount_ = (size_t)count;
   selectedCamera_.resize(channelCount_);
   CreateBuffers();
}

int CPyCHCamera::OnChannel(MM::PropertyBase *pProp, MM::ActionType eAct, long channel) {
   if (eAct == MM::BeforeGet) {
      pProp->Set(cameraNames_[selectedCamera_[channel]].c_str());
   } else if (eAct == MM::AfterSet) {
      std::string cameraName;
      pProp->Get(cameraName);

      long index = (std::find(cameraNames_.begin(), cameraNames_.end(), cameraName) - cameraNames_.begin());

      assert(index >= 0);

      {
         MM::Camera *camera = cameraDevices_[selectedCamera_[channel]];

         if (camera != cameraDevices_[index]) {
            selectedCamera_[(size_t) channel] = (size_t)index;
         }

      }

      width_ = (size_t)GetLongProperty("Width");
      height_ = (size_t)GetLongProperty("Height");
      bytes_ = (size_t)GetLongProperty("Bytes");

      for (int i = 0; i < selectedCamera_.size(); i++) {
         MM::Camera *camera = cameraDevices_[selectedCamera_[i]];
         if (!camera)
            continue;

         width_ = std::max(width_, (size_t) camera->GetImageWidth());
         height_ = std::max(height_, (size_t) camera->GetImageHeight());
         bytes_ = std::max(bytes_, (size_t) camera->GetImageBytesPerPixel());

      }

      FreeBuffers();
      CreateBuffers();

   }
   return DEVICE_OK;
}


int CPyCHCamera::Shutdown() { return DEVICE_OK; };

void CPyCHCamera::GetName(char *name) const { CDeviceUtils::CopyLimitedString(name, g_CameraDeviceName); };

long CPyCHCamera::GetImageBufferSize() const { return planeSize_; }

unsigned CPyCHCamera::GetNumberOfChannels() const { return (unsigned)channelCount_; }

int CPyCHCamera::GetChannelName(unsigned channel, char *name) {
   CDeviceUtils::CopyLimitedString(name, cameraNames_[selectedCamera_[channel]].c_str());
   return DEVICE_OK;
}

unsigned CPyCHCamera::GetBitDepth() const { return 8 * GetImageBytesPerPixel(); }

int CPyCHCamera::GetBinning() const { return 1; }

int CPyCHCamera::SetBinning(int binSize) { return DEVICE_OK; }

void CPyCHCamera::SetExposure(double exp_ms) { return; }

double CPyCHCamera::GetExposure() const { return 1.0; }

int CPyCHCamera::SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize) { return DEVICE_OK; }

int CPyCHCamera::GetROI(unsigned &x, unsigned &y, unsigned &xSize, unsigned &ySize) {
   x = 0;
   y = 0;
   xSize = GetImageWidth();
   ySize = GetImageHeight();
   return DEVICE_OK;
}

int CPyCHCamera::ClearROI() { return DEVICE_OK; }

int CPyCHCamera::IsExposureSequenceable(bool &isSequenceable) const {
   isSequenceable = false;
   return DEVICE_OK;
}

const unsigned char *CPyCHCamera::GetImageBuffer(unsigned channel) {
   return buffer_ + planeSize_ * channel;
}

const unsigned char *CPyCHCamera::GetImageBuffer() {
   return GetImageBuffer(0);
}


unsigned CPyCHCamera::GetImageWidth() const { return (unsigned)GetLongProperty("Width"); }

unsigned CPyCHCamera::GetImageHeight() const { return (unsigned)GetLongProperty("Height"); }

unsigned CPyCHCamera::GetImageBytesPerPixel() const { return (unsigned)GetLongProperty("Bytes"); }

int CPyCHCamera::SnapImage() {

   MMThreadGuard g(snapLock_);

   for (size_t i = 0; i < cameraSnapstate_.size(); i++) {
      cameraSnapstate_[i] = 0;
   }

   {

      std::string ourModuleName = GetNameOfOtherDevice(this);  // mixing up ModuleName and Name?

      bool threadedAcquisition = true;

      assert(selectedCamera_.size() == channelCount_);

      // TODO inefficient to allocate and deallocate
      std::vector<PyCHMultiCameraSnapThread *> cameraThreads(channelCount_, NULL);

      long c = 0;
      for (size_t i = 0; i < selectedCamera_.size(); i++) {
         if (cameraSnapstate_[selectedCamera_[i]])
            continue;

         MM::Camera *camera = cameraDevices_[selectedCamera_[i]];

         if (!camera)
            continue;

         std::string cameraModuleName = GetNameOfOtherDevice(camera);

         bool isPyCHCamera = (ourModuleName == cameraModuleName) && (ourModuleName != "");

         threadedAcquisition = !isPyCHCamera;
         threadedAcquisition = false; // it segfaults ...

         cameraSnapstate_[selectedCamera_[i]] = 1;

         if(threadedAcquisition) {
            cameraThreads[c] = new PyCHMultiCameraSnapThread();
            cameraThreads[c]->SetCamera(camera);
            cameraThreads[c]->Start();
            c++;
         } else {
            camera->SnapImage();
         }
      }

      for (size_t i = 0; i < c; i++) {
         if(cameraThreads[c]) {
            delete cameraThreads[c];
         }
      }
   }

   for (unsigned int i = 0; i < selectedCamera_.size(); i++) {
      MM::Camera *camera = cameraDevices_[selectedCamera_[i]];
      if (camera == NULL) {
         // empty camera, create empty buffer
         memset((unsigned char *) GetImageBuffer(i), 0, planeSize_);
      } else {
         if (camera->GetImageWidth() == width_) {
            if (camera->GetImageHeight() == height_) {
               assert(planeSize_ == camera->GetImageBufferSize());
               memcpy((unsigned char *) GetImageBuffer(i), camera->GetImageBuffer(), (size_t)camera->GetImageBufferSize());
            } else {
               memcpy((unsigned char *) GetImageBuffer(i), camera->GetImageBuffer(), (size_t)camera->GetImageBufferSize());
               memset(
                     (unsigned char *) GetImageBuffer(i) + (size_t)camera->GetImageBufferSize(),
                     0,
                     planeSize_ - camera->GetImageBufferSize()
               );
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

   if(!pyc_.IsBound()) {
      pyc_.BindBuffer(buffer_, channelCount_, GetImageHeight(), GetImageWidth(), GetImageBytesPerPixel());
   }

   pyc_.UpdateValuesChannelDevice();
   pyc_.UpdateValuesXYZ();

   pyc_.Execute();

   return DEVICE_OK;

};

int CPyCHCamera::InsertImage() {
   MMThreadGuard g(insertLock_);

   char label[MM::MaxStrLength];
   this->GetLabel(label);

   int ret = DEVICE_OK;

   for(int channel = channelCount_-1; channel >= 0; channel--) {
      Metadata md;
      md.put("Camera", label);



      md.put(MM::g_Keyword_CameraChannelName, cameraNames_[selectedCamera_[channel]].c_str());
      md.put(MM::g_Keyword_CameraChannelIndex, convert<std::string>(channel).c_str());


      ret = GetCoreCallback()->InsertImage(this, GetImageBuffer(channel), GetImageWidth(),
                                               GetImageHeight(), GetImageBytesPerPixel(),
                                               md.Serialize().c_str());

      if(ret == DEVICE_BUFFER_OVERFLOW) {
         if(!isStopOnOverflow()) {
            channel ++;
            GetCoreCallback()->ClearImageBuffer(this);
            continue;
         } else {
            return ret;
         }
      }
   }

   return ret;
}

int CPyCHCamera::OnScript(MM::PropertyBase *pProp, MM::ActionType eAct) {
   if (eAct == MM::BeforeGet) {
      pProp->Set(scriptFile_.c_str());
   } else if (eAct == MM::AfterSet) {
      pProp->Get(scriptFile_);
      if (scriptFile_ != "") {
         pyc_.RunScript(scriptFile_);
      }
   }
   return DEVICE_OK;
}

int CPyCHCamera::OnParameters(MM::PropertyBase *pProp, MM::ActionType eAct) {
   if (eAct == MM::BeforeGet) {
      pProp->Set(pyc_.GetParameters().c_str());
   } else if (eAct == MM::AfterSet) {
      std::string p;
      pProp->Get(p);
      pyc_.SetParameters(p);
   }
   return DEVICE_OK;
}

long CPyCHCamera::GetLongProperty(const char *name) const {
   char buf[MM::MaxStrLength];
   GetProperty(name, buf);
   return atoi(buf);
}

void CPyCHCamera::CreateBuffers() {
   planeSize_ = GetImageWidth() * GetImageHeight() * GetImageBytesPerPixel();
   buffer_ = new unsigned char[planeSize_ * channelCount_];
}

void CPyCHCamera::FreeBuffers() {
   if (buffer_) {
      delete[] buffer_;
      buffer_ = NULL;
   }
}

void CPyCHCamera::Empty() {
   if (buffer_) {
      memset(buffer_, 0, planeSize_ * channelCount_);
   }
}
