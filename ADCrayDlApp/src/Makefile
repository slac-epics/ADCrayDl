TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#======== DETECTOR LIBRARY ==============

# The following gets rid of the -fno-implicit-templates flag on vxWorks,
# so we get automatic template instantiation.
# This is what we want for miscellaneous/asynPortDriver.cpp
ifeq (vxWorks,$(findstring vxWorks, $(T_A)))
CODE_CXXFLAGS=
endif

INC += ADCrayDl.h
INC += ADCrayDlStorage.h
INC += ADCrayDlRecordNames.h
INC += ADCrayDlThreadedQueue.h
INC += ADCrayDlFrameSyncObject.h

LIBRARY_IOC = ADCrayDl

LIB_SRCS += ADCrayDl.cpp
LIB_SRCS += ADCrayDlStorage.cpp
LIB_SRCS += ADCrayDlFrameSyncObject.cpp

DBD += ADCrayDlSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

