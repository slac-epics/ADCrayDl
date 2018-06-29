TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# The following gets rid of the -fno-implicit-templates flag on vxWorks,
# so we get automatic template instantiation.
# This is what we want for miscellaneous/asynPortDriver.cpp
ifeq (vxWorks,$(findstring vxWorks, $(T_A)))
CODE_CXXFLAGS=
endif

PROD_NAME = ADCrayDlApp
PROD_IOC = $(PROD_NAME)

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += ADCrayDlSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS_DEFAULT += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp
PROD_SRCS_vxWorks += $(PROD_NAME)_registerRecordDeviceDriver.cpp

PROD_LIBS += ADCrayDl

PROD_SYS_LIBS += craydl
PROD_DEPLIB_DIRS += $(RAYONIX_LIB)

PROD_DEPLIB_DIRS += /reg/g/pcds/epics/R3.15.5-2.0/modules/timesync/R2.0.0/lib/linux-x86_64
PROD_SYS_LIBS += timesync

PROD_DEPLIB_DIRS += /reg/g/pcds/epics/R3.15.5-2.0/modules/timeStampFifo/R1.5.12/lib/linux-x86_64
PROD_SYS_LIBS += timeStampFifo

PROD_DEPLIB_DIRS += /reg/g/pcds/epics/R3.15.5-2.0/modules/event2/R4.3.0/lib/linux-x86_64
PROD_SYS_LIBS += devMrfEr drvMrf evrSupport mrfApi

include $(ADCORE)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

