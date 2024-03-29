# Must have loaded envPaths via st.cmd.linux or st.cmd.win32

errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/ADCrayDlApp.dbd")
ADCrayDlApp_registerRecordDeviceDriver(pdbbase)

# Prefix for all records
epicsEnvSet("PREFIX", "13SIM1:")

epicsEnvSet("CAM_PREFIX", "cam1:")

epicsEnvSet("TSS", "$(PREFIX)$(CAM_PREFIX)TSS")

# The port name for the detector
epicsEnvSet("PORT",   "SIM1")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximum image width; used to set the maximum size for this driver and for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "1024")
# The maximum image height; used to set the maximum size for this driver and for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "1024")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The maximum number of threads for plugins which can run in multiple threads
epicsEnvSet("MAX_THREADS", "8")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

# PV Prefixes
# epicsEnvSet( "IOC_PV",	"IOC:TSTRAYONIX:123" )
epicsEnvSet( "EVR",	"EVR:TSTRAYONIX:123" )
# epicsEnvSet( "TRIG_PV",	"$(EVR_PV):TRIG0" )
# Configure EVR
epicsEnvSet( "EVR_CARD",	"0" )
# EVR Type: 0=VME, 1=PMC, 15=SLAC
epicsEnvSet( "EVR_TYPE",	"15" )

# < /reg/d/iocCommon/All/pre_linux.cmd

asynSetMinTimerPeriod(0.001)

# The EPICS environment variable EPICS_CA_MAX_ARRAY_BYTES needs to be set to a value at least as large
# as the largest image that the standard arrays plugin will send.
# That vlaue is $(XSIZE) * $(YSIZE) * sizeof(FTVL data type) for the FTVL used when loading the NDStdArrays.template file.
# The variable can be set in the environment before running the IOC or it can be set here.
# It is often convenient to set it in the environment outside the IOC to the largest array any client
# or server will need.  For example 10000000 (ten million) bytes may be enough.
# If it is set here then remember to also set it outside the IOC for any CA clients that need to access the waveform record.
# Do not set EPICS_CA_MAX_ARRAY_BYTES to a value much larger than that required, because EPICS Channel Access actually
# allocates arrays of this size every time it needs a buffer larger than 16K.
# Uncomment the following line to set it in the IOC.
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "250000000")

# Configure a PMC EVR
# ErDebugLevel( 99 )
ErConfigure( $(EVR_CARD), 0, 0, 0, $(EVR_TYPE) )
dbLoadRecords("$(EVENT2)/db/evrSLAC.db", "EVR=$(EVR),CARD=$(EVR_CARD),IP0E=Enabled,IP1E=Enabled,IP2E=Enabled")

dbLoadRecords("$(ADCRAYDL)/db/ADCrayDl.template","P=$(PREFIX),R=$(CAM_PREFIX),TSS=$(TSS),PORT=$(PORT),ADDR=0,TIMEOUT=1,MAXAGEPEDESTAL=10")
# Create a CrayDl driver
# CrayDlConfig(const char *portName, int dataType,
#                   int maxBuffers, int maxMemory, int priority, int stackSize)
ADCrayDlConfig("$(PORT)", 3, 0, -1)

# Load timeStampFifo.template w/ these macros
# DEV - prefix for all timeStampFifo records
# PORT_PV - PV name for the camera asyn port
# EC_PV - PV name that can be read to get the current trigger event code
dbLoadRecords("$(TIMESTAMPFIFO)/db/timeStampFifo.template",  "DEV=$(TSS),PORT_PV=$(PREFIX)$(CAM_PREFIX)PortName_RBV,EC_PV=$(EVR):TRIG0:EC_RBV,DLY=1" )

# Create a standard arrays plugin, set it to get data from first CrayDl driver.
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0, -1)

# This creates a waveform large enough for 7680x7680 arrays.
# This waveform allows transporting 16-bit images
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX)$(CAM_PREFIX),R=IMAGE2:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),DATATYPE=3,TYPE=Int16,FTVL=USHORT,NELEMENTS=58982400,ENABLED=1")

# Set up autosave
set_requestfile_path("./")
set_requestfile_path("$(ADCORE)/ADApp/Db")
set_requestfile_path("$(ADCORE)/iocBoot")
set_savefile_path("./autosave")
set_pass0_restoreFile("auto_settings.sav")
set_pass1_restoreFile("auto_settings.sav")
save_restoreSet_status_prefix("$(PREFIX)")
dbLoadRecords("$(AUTOSAVE)/asApp/Db/save_restoreStatus.db", "P=$(PREFIX),IOC=$(PREFIX)SAVE_RESTORE_ST")

set_requestfile_path("$(ADCRAYDL)/ADCrayDlApp/Db")

iocInit()

# ADCrayDlInitTiming("$(EVR_PV):Triggers.A", "$(EVR_PV):Triggers.M", "$(PREFIX)$(CAM_PREFIX)DataDelay.VAL", "$(PREFIX)$(CAM_PREFIX)SyncStatus")

# save things every thirty seconds
set_savefile_path("./autosave")
create_monitor_set("auto_settings.req", 10, "P=$(PREFIX),IOC=$(PREFIX)$(CAM_PREFIX)")

# Readout times for different binning and readout modes.
system "caput -a $(PREFIX)$(CAM_PREFIX)BinningAndReadoutReadoutTimes 12 0.483 0.146 0.082 0.05 0.026 0.014 0.592 0.177 0.094 0.057 0.03 0.015"

dbpf $(TSS):TsPolicy SYNCED

dbpf $(PREFIX)$(CAM_PREFIX)ArrayCallbacks 1

#var DEBUG_TS_FIFO 5
dbpf $(EVR):TRIG0:TEC 45
dbpf $(EVR):TRIG0:TCTL 1


# dbpf $(PREFIX)$(CAM_PREFIX)Acquire 1

# All IOCs should dump some common info after initial startup.
#< /reg/d/iocCommon/All/post_linux.cmd
