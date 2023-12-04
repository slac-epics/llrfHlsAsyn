TOP=../..

include $(TOP)/configure/CONFIG

USR_CXXFLAGS += -DHAVE_YAML

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#========================================

INCLUDES += $(addprefix -I,$(YAML_INCLUDE))
INCLUDES += $(addprefix -I,$(BOOST_INCLUDE))

## this part is just for yaml loader driver
INCLUDES += $(addprefix -I,$(CPSW_FRAMEWORK_INCLUDE))

INCLUDES += $(addprefix -I,$(LLRFHLS_INCLUDE))

INC += llrfHlsAsyn.h

DBD += llrfHlsAsynDriver.dbd
DBD += timeAxisGenerator.dbd

LIBRARY_IOC += llrfHlsAsyn

llrfHlsAsyn_DBD  += llrfHlsAsynDriver.dbd
llrfHlsAsyn_DBD  += timeAxisGenerator.c
llrfHlsAsyn_SRCS += llrfHlsAsyn.cpp
llrfHlsAsyn_DBD  += timeAxisGenerator.c

llrfHlsAsyn_DBD  += interlockRtmAsyn.dbd


cpsw_DIR = $(CPSW_FRAMEWORK_LIB)
llrfHls_DIR = $(LLRFHLS_LIB)

llrfHlsAsyn_LIBS += cpsw
llrfHlsAsyn_LIBS += llrfHls
llrfHlsAsyn_LIBS += asyn
llrfHlsAsyn_LIBS += yamlLoader
llrfHlsAsyn_LIBS += $(EPICS_BASE_IOC_LIBS)


LIBRARY_IOC += dummyBsaCore_SC
dummyBsaCore_SC_SRCS += dummyBsaCore_SC.cpp


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#

