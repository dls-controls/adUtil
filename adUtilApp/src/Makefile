TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY_IOC += adUtil

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxx.h

# xxxRecord.dbd will be installed into <top>/dbd
DBD += adUtilSupport.dbd

# The following are compiled and added to the support library
adUtil_SRCS += BPM_calc.cpp
adUtil_SRCS += bessel.cpp
adUtil_SRCS += window.cpp
adUtil_SRCS += fftpack.cpp
adUtil_SRCS += FFT_calc.cpp
adUtil_SRCS += NDPluginCircularBuff.cpp
adUtil_SRCS += NDArrayRing.cpp

# We need to link against the EPICS Base libraries
#xxx_LIBS += $(EPICS_BASE_IOC_LIBS)

#USR_CXXFLAGS = -ggdb -O0

include $(TOP)/configure/RULES
