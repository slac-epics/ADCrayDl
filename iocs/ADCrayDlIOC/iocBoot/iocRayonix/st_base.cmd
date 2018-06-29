# Must have loaded envPaths via st.cmd.linux or st.cmd.win32

errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/ADCrayDlApp.dbd")
ADCrayDlApp_registerRecordDeviceDriver(pdbbase)

# Prefix for all records
epicsEnvSet("PREFIX", "13SIM1:")
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
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "1000000000")

# Create a CrayDl driver
# CrayDlConfig(const char *portName, int dataType,
#                   int maxBuffers, int maxMemory, int priority, int stackSize)
ADCrayDlConfig("$(PORT)", 3, 0, -1)

dbLoadRecords("$(ADCRAYDL)/db/ADCrayDl.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,MAXAGEPEDESTAL=10")

# Create a standard arrays plugin, set it to get data from first CrayDl driver.
NDStdArraysConfigure("Image1", 3, 0, "$(PORT)", 0, -1)

# This creates a waveform large enough for 7680x7680 arrays.
# This waveform allows transporting 16-bit images
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),DATATYPE=3,TYPE=Int16,FTVL=USHORT,NELEMENTS=58982400,ENABLED=1")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(ADCRAYDL)/ADCrayDlApp/Db")

iocInit()

# save things every thirty seconds
set_savefile_path("./autosave")
create_monitor_set("auto_settings.req", 30, "P=$(PREFIX)")

dbpf $(PREFIX)cam1:ArrayCallbacks 1
#dbpf $(PREFIX)cam1:BinX 10
#dbpf $(PREFIX)cam1:Acquire 1
