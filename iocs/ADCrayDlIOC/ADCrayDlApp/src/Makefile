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

include $(ADCORE)/ADApp/commonDriverMakefile

$(PROD_NAME)_DBD += ADCrayDlSupport.dbd

$(PROD_NAME)_DBD += evrSupport.dbd
$(PROD_NAME)_DBD += devMrfEr.dbd
$(PROD_NAME)_DBD += diagTimer.dbd
$(PROD_NAME)_DBD += timeStampFifo.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS_DEFAULT += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp
PROD_SRCS_vxWorks += $(PROD_NAME)_registerRecordDeviceDriver.cpp

PROD_LIBS += ADCrayDl
PROD_LIBS += ADBase

PROD_SYS_LIBS += craydl
PROD_DEPLIB_DIRS += $(RAYONIX_LIB)

# FIXME change paths to use variables
PROD_DEPLIB_DIRS += $(AUTOSAVE)/lib/rhel6-x86_64

PROD_DEPLIB_DIRS += $(TIMESTAMPFIFO)/lib/linux-x86_64
PROD_SYS_LIBS += timeStampFifo

PROD_DEPLIB_DIRS += $(EVENT2)/lib/rhel6-x86_64
PROD_SYS_LIBS += devMrfEr
PROD_SYS_LIBS += drvMrf
PROD_SYS_LIBS += evrSupport
PROD_SYS_LIBS += mrfApi

PROD_DEPLIB_DIRS += $(DIAG_TIMER)/lib/rhel6-x86_64
PROD_SYS_LIBS += diagTimer


#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


