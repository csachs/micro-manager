///////////////////////////////////////////////////////////////////////////////
#ifndef _PYCH_H
#define _PYCH_H

#include "../../MMDevice/DeviceBase.h"
#include "../../MMDevice/DeviceThreads.h"

#include <boost/python.hpp>
#include <boost/python/numpy.hpp>

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

inline np::dtype dtype_conversion(size_t n) {
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
   PythonImageCallback() : stateDevice_(0), bound(false) {};

   ~PythonImageCallback() {};

   int Initialize(MM::Device *host, MM::Core *core);

   void unbindBuffer();

   void bindBuffer(unsigned char *buffer, size_t channels, size_t height, size_t width, size_t depth);

   bool isBound();

   void execute();

   void runScript(std::string name);

   void updateValuesXYZ();

   void updateValuesChannelDevice();

   void setChannelDevice(std::string channelDevice);

   std::string getChannelDevice();

   MM::Core *GetCoreCallback();

private:

   bool bound;

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
   CPyCHCamera();

   ~CPyCHCamera();

   int Initialize();

   int Shutdown();

   int OnScript(MM::PropertyBase *pProp, MM::ActionType eAct);

   int OnScriptChannelDevice(MM::PropertyBase *pProp, MM::ActionType eAct);

   int OnChannelCount(MM::PropertyBase *pProp, MM::ActionType eAct);

   int OnChannel(MM::PropertyBase *pProp, MM::ActionType eAct, long channel);

   void GetName(char *name) const;

   long GetImageBufferSize() const;

   unsigned GetNumberOfChannels() const;

   int GetChannelName(unsigned channel, char *name);

   unsigned GetBitDepth() const;

   int GetBinning() const;

   int SetBinning(int binSize);

   void SetExposure(double exp_ms);

   double GetExposure() const;

   int SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize);

   int GetROI(unsigned &x, unsigned &y, unsigned &xSize, unsigned &ySize);

   int ClearROI();

   int IsExposureSequenceable(bool &isSequenceable) const;

   const unsigned char *GetImageBuffer(unsigned channel);

   const unsigned char *GetImageBuffer();

   unsigned GetImageWidth() const;

   unsigned GetImageHeight() const;

   unsigned GetImageBytesPerPixel() const;

   int SnapImage();

private:

   std::vector<std::string> GetDevicesOfType(MM::DeviceType type);

   long GetLongProperty(const char *name) const;

   void CreateBuffers();

   void FreeBuffers();

   void Empty();

   size_t channelCount_, width_, height_, bytes_;
   size_t planeSize_;

   unsigned char *buffer_;

   std::vector<std::string> cameraNames_;
   std::vector<MM::Camera *> cameraDevices_;
   std::vector<bool> cameraSnapstate_;

   std::vector<size_t> selectedCamera_;

   std::vector<std::string> channelDevices_;
   std::string scriptFile_;
   PythonImageCallback pyc_;
};

#endif //_PYCH_H_
