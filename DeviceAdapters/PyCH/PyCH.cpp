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
const char *g_CameraDeviceName = "PyCHCamera";

///////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////

MODULE_API void InitializeModuleData() {
   RegisterDevice(g_CameraDeviceName, MM::CameraDevice, "PyCHCamera");
   RegisterDevice("PyCHCamera", MM::CameraDevice, "PyCHCamera");
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

   return DEVICE_OK;
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

   if (stateDevice_) {
         stateDevice_->GetPosition(channel);
   }

   try {
      main_namespace["_channel"] = channel;
   } catch(p::error_already_set const &) {
      PyErr_Print();
   }
}


void PythonImageCallback::processBuffer(unsigned char *buffer, int channels, int height, int width, int depth) {
   try {
      np::ndarray array = np::from_data(
            buffer, //const_cast<uint8_t *>(),
            dtype_conversion(depth),
            p::make_tuple(channels, height, width),
            p::make_tuple(height * width * depth, width * depth, depth),
            p::object()
      );

      main_namespace["_image_buffer"] = array;

      main_namespace["_callback"]();

      //p::exec("_callback()", main_namespace);

   } catch(p::error_already_set const &) {
      PyErr_Print();
   }
};



const char* g_Undefined = "Undefined";
const char* g_deviceNameMultiCameraPyCH = "MultiCameraPyCH";


