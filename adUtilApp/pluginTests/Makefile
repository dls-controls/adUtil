TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

PROD_IOC += plugin-test
plugin-test_SRCS += plugin-test.cpp
plugin-test_SRCS += testingutilities.cpp
plugin-test_SRCS += NDPluginMock.cpp
plugin-test_SRCS += test_NDPluginReframe.cpp
USR_INCLUDES += $(HDF5_INCLUDE)
USR_INCLUDES += $(SZ_INCLUDE)
USR_INCLUDES += $(XML2_INCLUDE)

plugin-test_LIBS += boost_unit_test_framework simDetector
boost_unit_test_framework_DIR=$(BOOST_LIB)

PROD_LIBS += Com NDPlugin asyn ADBase adUtil

include $(TOP)/configure/RULES
