
AM_CXXFLAGS = $(MMDEVAPI_CXXFLAGS) $(BOOST_CPPFLAGS) -I. -I/usr/include/python3.5m "-DPYTHON_SHARED_OBJECT=\"libpython3.5m.so\""
deviceadapter_LTLIBRARIES = libmmgr_dal_PyCH.la
libmmgr_dal_PyCH_la_SOURCES = PyCH.cpp PyCH.h ../../MMDevice/MMDevice.h
libmmgr_dal_PyCH_la_LDFLAGS = $(MMDEVAPI_LDFLAGS) -lboost_python-gcc54-mt-1_62 -lboost_numpy-gcc54-mt-1_62 -ldl -lpython3.5m -L$(MMABSOLUTE_PATH)/DeviceAdapters/PyCH
libmmgr_dal_PyCH_la_LIBADD = $(MMDEVAPI_LIBADD)

EXTRA_DIST = PyCH.vcproj
