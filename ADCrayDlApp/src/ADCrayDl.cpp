#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <array>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <registryFunction.h>
#include <epicsExport.h>
#include <subRecord.h>
#include <cantProceed.h>
#include <iocsh.h>

#include "ADDriver.h"
#include <epicsExport.h>
#include <dbAccess.h>
#include "ADCrayDl.h"
#include "ADCrayDlRecordNames.h"

namespace
{
    static const char *DRIVER_NAME = "ADCrayDl";
    static const useconds_t POLLING_INTERVAL_US = 1e6; // 1 second
    static const boost::posix_time::ptime EPOCH(boost::gregorian::date(1970, 1, 1));
}

namespace adcraydl
{

void ADCrayDl::pollDetectorStatus(const uint32_t interval_us)
{
    while (m_running.get())
    {
        const craydl::RxReturnStatus status = m_rayonixDetector->QueryStatus();
        if (status.IsError())
        {
            std::cerr << "Error querying status: " << status.ErrorText() << std::endl;
        }

        // std::cout << "polling " << std::endl;

        usleep(interval_us);
    }
}

bool ADCrayDl::handleCoolingPV(const int function, const epicsInt32 value, asynStatus &status)
{
    if (function == CoolerFunction)
    {
        craydl::RxReturnStatus error;
        if (value == 0)
        {
            error = m_rayonixDetector->DisableCoolers();
        }
        else
        {
            error = m_rayonixDetector->EnableCoolers();
        }

        if (error.IsError())
        {
            const std::string command = (value == 0) ? "disabling" : "enabling";
            std::cerr << "Error " << command << " cooler: " << error.ErrorText() << std::endl;

            status = asynError;
        }
        else
        {
            const std::string command = (value == 0) ? "disabled" : "enabled";
            std::cout << "!!!! Successfully " << command << " cooler" << std::endl;
        }

        return true;
    }

    return false;
}

bool ADCrayDl::handleCoolingPV(const int function, const epicsFloat64 value, asynStatus &status)
{
    if (function == CCDTempSetpointFunction)
    {
        const craydl::RxReturnStatus error = m_rayonixDetector->SetSensorTemperatureSetpoint(value); // FIXME is this the correct method?
        
        if (error.IsError())
        {
            std::cerr << "Error setting CCD temperature setpoint: " << error.ErrorText() << std::endl;
            status = asynError;
        }
        
        return true;
    }
    else if (function == CoolerTempSetpointFunction)
    {
        const craydl::RxReturnStatus error = m_rayonixDetector->SetColdHeadTemperatureSetpoint(value); // FIXME is this the correct method?

        if (error.IsError())
        {
            std::cerr << "Error setting cooler temperature setpoint: " << error.ErrorText() << std::endl;
            status = asynError;
        }

        return true;
    }

    return false;
}

bool ADCrayDl::handleVacuumPV(const int function, const epicsInt32 value, asynStatus &status)
{
    if (function == VacuumValveFunction)
    {
        craydl::RxReturnStatus error;

        if (value == 0)
        {
            error = m_rayonixDetector->DisableVacuumValve();
        }
        else
        {
            error = m_rayonixDetector->EnableVacuumValve();
        }

        if (error.IsError())
        {
            const std::string command = (value == 0) ? "disabling" : "enabling";
            std::cerr << "Error " << command << " vacuum valve: " << error.ErrorText() << std::endl;
            status = asynError;
        }

        return true;
    }

    return false;
}

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADColorMode, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus ADCrayDl::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    const int function = pasynUser->reason;

    std::cout << "writeInt32 func: " << function << ", value: " << value << std::endl;

    // Set the parameter and readback in the parameter library. This may be overwritten when we read back the
    // status at the end, but that's OK.
    asynStatus status = setIntegerParam(function, value);

    /* This is where the parameter is sent to the SDK */
    if (function == ADAcquire)
    {
        callParamCallbacks();

        printf("%s: Detected change of ADAcquire to %d\n", DRIVER_NAME, value );

        if (value)
        {
            do // This do-while is a convenient solution to error handling. We can jump out of the block early.
            {
                craydl::RxReturnStatus error = m_rayonixDetector->SetupAcquisitionSequence(m_numImages, 1);
                if (error.IsError())
                {
                    std::cerr << "Could not setup acquisition sequence to " << m_numImages << " images" << std::endl;
                    continue; // Skip other code.
                }

                error = m_rayonixDetector->SendParameters();
                if (error.IsError())
                {
                    std::cerr << "Could not send parameters to detector" << std::endl;
                    continue; // Skip other code.
                }

                // Starts Acquisition of series of frames - Light means operated shutter I/O output signal
                craydl::FrameAcquisitionType frame_type = craydl::FrameAcquisitionTypeLight;
                error = m_rayonixDetector->StartAcquisition(frame_type);
                if (error.IsError())
                {
                    std::cerr << "Could not start acquisition" << std::endl;
                    continue; // Skip other code.
                }
            } while (false);

            status = setIntegerParam(ADAcquire, 0);
        }
    }
    else if (function == ADNumImages)
    {
        m_numImages = value;
    }
    else if (function == BinningFunction)
    {
        std::cout << "Setting binning" << std::endl;
        int binningValue = 1;

        switch (value)
        {
            case BinningMode1x1:
            default:
                binningValue = 1;
                break;

            case BinningMode2x2:
                binningValue = 2;
                break;

            case BinningMode3x3:
                binningValue = 3;
                break;
            
            case BinningMode4x4:
                binningValue = 4;
                break;
            
            case BinningMode6x6:
                binningValue = 6;
                break;
            
            case BinningMode10x10:
                binningValue = 10;
                break;
        }

        const craydl::RxReturnStatus error = m_rayonixDetector->SetBinning(binningValue, binningValue);
        if (error.IsError())
        {
            std::cerr << "Binning " << binningValue << "x" << binningValue << " was not allowed: " << error.ErrorText() << std::endl;
            status = asynError;
        }
        else
        {
            // All ok
            updateDimensionSize();
        }
    }
    else if (function == ADTriggerMode)
    {
        craydl::FrameTriggerType_t triggerMode;

        switch (value)
        {
            case TriggerModeFreeRun:
            case TriggerModeLCLSMode: // TODO: LCLS mode is not yet implemented in the craydl SDK.
            default:
                triggerMode = craydl::FrameTriggerTypeNone;
                break;

            case TriggerModeEdge:
                triggerMode = craydl::FrameTriggerTypeFrame;
                break;

            case TriggerModeBulb:
                triggerMode = craydl::FrameTriggerTypeBulb;
                break;
        }

        const craydl::RxReturnStatus error = m_rayonixDetector->SetFrameTriggerMode(craydl::FrameTriggerType(triggerMode));
        if (error.IsError())
        {
            std::cerr << "Error setting trigger mode " << value << ", error: " << error.ErrorText() << std::endl;
            status = asynError;
        }
    }
    else if (function == ReadoutModeFunction)
    {
        craydl::ReadoutMode_t readoutMode;

        switch (value)
        {
            case ReadoutModeStandard:
            default:
                readoutMode = craydl::ReadoutModeStandard;
                break;
            
            case ReadoutModeLowNoise:
                readoutMode = craydl::ReadoutModeLowNoise;
                break;
        }

        const craydl::RxReturnStatus error = m_rayonixDetector->SetReadoutMode(craydl::ReadoutMode(readoutMode));
        if (error.IsError())
        {
            std::cerr << "Error setting readout mode " << value << ", error: " << error.ErrorText() << std::endl;
            status = asynError;
        }
    }
    else if (function == PedestalNumImagesFunction)
    {
        m_numPedestals = value;
    }
    else if (function == AcquirePedestalFunction)
    {
        std::cout << "Going to acquire pedestal" << std::endl;

        // Trigger pedestal acquisition
        const craydl::RxReturnStatus error = m_rayonixDetector->AcquireNewBackground(false, m_numPedestals);
        if (error.IsError())
        {
            std::cerr << "Error acquiring pedestals, error: " << error.ErrorText() << std::endl;
            status = asynError;
        }
    }
    else if (function == EnableDetectorQueryingFunction)
    {
        if (value == 0)
        {
            // Stop querying.
            m_running = false;
        }
        else
        {
            // Start querying.
            if (m_pollingThread.joinable())
            {
                m_pollingThread.join();
            }

            m_running = true;
            m_pollingThread = std::thread(&ADCrayDl::pollDetectorStatus, this, POLLING_INTERVAL_US);
        }
    }
    else if (!handleCoolingPV(function, value, status) && !handleVacuumPV(function, value, status)) // Check if these are cooling or vacuum PVs
    {
        // If not, use base method.
        status = ADDriver::writeInt32(pasynUser, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if (status)
    {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:writeInt32 error, status=%d function=%d, value=%d\n",
              DRIVER_NAME, status, function, value);
    }
    else
    {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:writeInt32: function=%d, value=%d\n",
              DRIVER_NAME, function, value);
    }

    return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters, including ADAcquireTime, ADGain, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus ADCrayDl::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    const int function = pasynUser->reason;

    std::cout << "writeFloat64 func: " << function << ", value: " << value << std::endl;

    // Set the parameter and readback in the parameter library. This may be overwritten when we read back the
    // status at the end, but that's OK.
    asynStatus status = setDoubleParam(function, value);

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    if (function == ADAcquireTime)
    {
        const craydl::RxReturnStatus error = m_rayonixDetector->SetExposureTime(value);
        if (error.IsError())
        {
            std::cerr << "Error setting acquire time " << value << ", error: " << error.ErrorText() << std::endl;
            status = asynError;
        }
    }
    else if (function == ADAcquirePeriod)
    {
        std::cout << "Setting interval" << std::endl;
        const craydl::RxReturnStatus error = m_rayonixDetector->SetIntervalTime(value);
        if (error.IsError())
        {
            std::cerr << "Error setting interval time " << value << ", error: " << error.ErrorText() << std::endl;
            status = asynError;
        }
    }
    else if (!handleCoolingPV(function, value, status)) // Check if these are cooling PVs
    {
        status = ADDriver::writeFloat64(pasynUser, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if (status)
    {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:writeFloat64 error, status=%d function=%d, value=%f\n",
              DRIVER_NAME, status, function, value);
    }
    else
    {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:writeFloat64: function=%d, value=%f\n",
              DRIVER_NAME, function, value);
    }

    return status;
}

void ADCrayDl::RawStatusChanged(const std::string &name, const std::string &value)
{
    std::cout << "------------- raw status changed - name " << name << " value " << value << std::endl;
}

void ADCrayDl::ParameterChanged(const std::string &name, const std::string &value)
{
    std::cout << "------------- parameter changed - name " << name << " value " << value << std::endl;
}

void ADCrayDl::SequenceStarted()
{
    // Intentionally empty.
}

void ADCrayDl::SequenceEnded()
{
    // Intentionally empty.
}

void ADCrayDl::ExposureStarted(int frame_number)
{
    // Intentionally empty.
}

void ADCrayDl::ExposureEnded(int frame_number)
{
    // Intentionally empty.
}

void ADCrayDl::ReadoutStarted(int frame_number)
{
    // Intentionally empty.
}

void ADCrayDl::ReadoutEnded(int frame_number)
{
    // Intentionally empty.
}

static time_t toUnixTimestamp(const boost::posix_time::ptime &pt)
{
    using namespace boost::posix_time;

    time_duration diff(pt - EPOCH);
    const time_t localTime = (diff.ticks() / diff.ticks_per_second());

    std::tm local_field = *std::gmtime(&localTime);
    local_field.tm_isdst = -1;
    time_t utc = std::mktime(&local_field);

    return utc;
}

void ADCrayDl::BackgroundFrameReady(const craydl::RxFrame *frame_p)
{
    // Handle pedestals
    const craydl::FrameMetaData &metadata = frame_p->metaData();
    setIntegerParam(PedestalTimestampFunction, toUnixTimestamp(metadata.AcquisitionStartTimestamp()));

    callParamCallbacks();
}

void ADCrayDl::RawFrameReady(int frame_number, const craydl::RxFrame *frame_p)
{
    // Intentionally empty.
}

void ADCrayDl::FrameReady(int frame_number, const craydl::RxFrame *frame_p)
{
    applyFrameToAD(frame_p);
}

void ADCrayDl::FrameAborted(int frame_number)
{
    // Intentionally empty.
}

void ADCrayDl::FrameCompleted(int frame_number)
{
    // Intentionally empty.
}

void ADCrayDl::FrameError(int frame_number, const craydl::RxFrame *frame_p, int error_code, const std::string &error_string)
{
    std::cerr << "Error with frame: " << error_string << std::endl;
}

void ADCrayDl::applyFrameToAD(const craydl::RxFrame *frame_p)
{
    using namespace boost::posix_time;

    // Get timestamp of frame
    epicsTimeStamp newEvrTime;
    updateTimeStamp(&newEvrTime);

    std::cout << "Current system time: " << time(0) << std::endl; 

    NDArray *inArray = pNDArrayPool->alloc(NUM_DIMS, m_dims, NDUInt16, frame_p->getSize(), NULL);

    for (size_t i = 0; i < NUM_DIMS; i++)
    {
        inArray->initDimension(&m_dimsOut[i], m_dims[i]);
    }

    // The dimensions are the same and the data type is the same, just copy the input image to the output image.
    memcpy(inArray->pData, frame_p->getBufferAddress(), frame_p->getSize());

    // Set timestamp
    inArray->epicsTS = newEvrTime;

    std::cout << "The new timestamp is " << inArray->epicsTS.secPastEpoch << std::endl;

    increaseArrayCounter();

    // Call callbacks.
    int arrayCallbacks;
    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    if (arrayCallbacks != 0)
    {
        // std::cout << "Calling image data callback" << std::endl;
        doCallbacksGenericPointer(inArray, NDArrayData, 0);
    }

    // The module is done with the frame.
    inArray->release();
}

int ADCrayDl::updateDimensionSize()
{
    int pixelsX = 0;
    int pixelsY = 0;
    int pixelsZ = 0;
    m_rayonixDetector->GetFrameSize(pixelsX, pixelsY, pixelsZ);

    const std::array<int, NUM_DIMS> sizes{{pixelsX, pixelsY}};

    for (size_t i = 0; i < NUM_DIMS; i++)
    {
        m_dimsOut[i].binning = 1;
        m_dimsOut[i].offset = 0;
        m_dimsOut[i].reverse = 0;
        m_dims[i] = sizes.at(i);
    }

    int status = asynSuccess;
    status |= setIntegerParam(ADSizeX, pixelsX);
    status |= setIntegerParam(ADSizeY, pixelsY);
    status |= setIntegerParam(NDArraySizeX, pixelsX);
    status |= setIntegerParam(NDArraySizeY, pixelsY);
    status |= setIntegerParam(NDArraySize, pixelsX * pixelsY * pixelsZ);

    return status;
}

void ADCrayDl::increaseArrayCounter()
{
    int arrayCounter;
    getIntegerParam(NDArrayCounter, &arrayCounter);

    ++arrayCounter;

    setIntegerParam(NDArrayCounter, arrayCounter);
}

static DBADDR getPVAddr(const char *evName)
{
    DBADDR addr;

    if (dbNameToAddr(evName, &addr))
    {
        printf("No PV named %s!!!\n", evName);
    }

    return addr;
}

/** Constructor
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] dataType The initial data type (NDDataType_t) of the images that this driver will create.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
ADCrayDl::ADCrayDl(const char *portName, NDDataType_t dataType,
                         int maxBuffers, size_t maxMemory, int priority, int stackSize)

    : ADDriver(portName, 1, 0, maxBuffers, maxMemory,
               0, 0, /* No interfaces beyond those set in ADDriver.cpp */
               0, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize),
       m_rayonixDetector(craydl::RxDetector::create()),
       m_running(false),
       m_numImages(1),
       m_numPedestals(1)
{
    int status = asynSuccess;
    char versionString[20];
    const char *functionName = "ADCrayDl";

    // Create custom parameters
    createParam(ACQUIRE_PEDESTAL_STR,         asynParamInt32, &AcquirePedestalFunction);
    createParam(READOUT_MODE_STR,             asynParamInt32, &ReadoutModeFunction);
    createParam(PEDESTAL_NUM_IMG_STR,         asynParamInt32, &PedestalNumImagesFunction);
    createParam(PEDESTAL_TIMESTAMP_STR,       asynParamInt32, &PedestalTimestampFunction);
    createParam(ENABLE_DETECTOR_QUERYING_STR, asynParamInt32, &EnableDetectorQueryingFunction);
    createParam(BINNING_STR,                  asynParamInt32, &BinningFunction);

    // Cooling
    createParam(COOLER_STR,               asynParamInt32,   &CoolerFunction);
    createParam(CCD_TEMP_SETPOINT_STR,    asynParamFloat64, &CCDTempSetpointFunction);
    createParam(COOLER_TEMP_SETPOINT_STR, asynParamFloat64, &CoolerTempSetpointFunction);
    createParam(MEAN_SENSOR_TEMP_STR,     asynParamFloat64, &MeanSensorTempFunction);
    createParam(MAX_SENSOR_TEMP_STR,      asynParamFloat64, &MaxSensorTempFunction);
    createParam(MIN_SENSOR_TEMP_STR,      asynParamFloat64, &MinSensorTempFunction);
    createParam(MEAN_COOLER_TEMP_STR,     asynParamFloat64, &MeanCoolerTempFunction);
    createParam(MAX_COOLER_TEMP_STR,      asynParamFloat64, &MaxCoolerTempFunction);
    createParam(MIN_COOLER_TEMP_STR,      asynParamFloat64, &MinCoolerTempFunction);
    createParam(COOLER_ENABLED_STR,       asynParamInt32,   &CoolerEnabledFunction);
    createParam(COOLER_RUNNING_STR,       asynParamInt32,   &CoolerRunningFunction);

    // Vacuum
    createParam(VACUUM_VALVE_STR,         asynParamInt32,   &VacuumValveFunction);
    createParam(LINE_PRESSURE_STR,        asynParamFloat64, &LinePressureFunction);
    createParam(CHAMBER_PRESSURE_STR,     asynParamFloat64, &ChamberPressureFunction);
    createParam(VACUUM_VALVE_OPEN_STR,    asynParamInt32,   &VacuumValveOpenFunction);
    createParam(VACUUM_VALVE_ENABLED_STR, asynParamInt32,   &VacuumValveEnabledFunction);
    createParam(VACUUM_PUMP_RUNNING_STR,  asynParamInt32,   &VacuumPumpRunningFunction);
    createParam(VACUUM_PUMP_IGNORED_STR,  asynParamInt32,   &VacuumPumpIgnoredFunction);

    if (status)
    {
        printf("%s: unable to set camera parameters\n", functionName);
        return;
    }

    // Set callback on detector.
    m_rayonixDetector->RegisterFrameCallback(this);
    m_rayonixDetector->RegisterEveryStatusChangeCallback(this);

    m_rayonixDetector->SetBackgroundFrameTriggerMode(craydl::FrameTriggerType(craydl::FrameTriggerTypeNone));
    m_rayonixDetector->SetFrameTriggerMode(craydl::FrameTriggerType(craydl::FrameTriggerTypeNone));

    std::string detectorModel;
    std::string detectorVersion;
    m_rayonixDetector->GetDetectorID(detectorModel, detectorVersion);

    std::string detectorFirmwareName;
    std::string detectorFirmwareVersion;
    m_rayonixDetector->GetDetectorFirmwareID(detectorFirmwareName, detectorFirmwareVersion);

    updateDimensionSize();

    // Set defaults
    status |= setStringParam(ADManufacturer, "Rayonix");
    status |= setStringParam(ADModel, detectorModel.c_str());
    status |= setStringParam(ADFirmwareVersion, detectorFirmwareVersion.c_str());
    status |= setIntegerParam(ADMaxSizeX, 7680);
    status |= setIntegerParam(ADMaxSizeY, 7680);
    status |= setIntegerParam(ADMinX, 0);
    status |= setIntegerParam(ADMinY, 0);
    status |= setIntegerParam(ADBinX, 1);
    status |= setIntegerParam(ADBinY, 1);
    status |= setIntegerParam(ADReverseX, 0);
    status |= setIntegerParam(ADReverseY, 0);
    status |= setIntegerParam(NDDataType, dataType);
    status |= setIntegerParam(NDColorMode, NDColorModeMono);
    status |= setIntegerParam(NDArrayCounter, 0);
    status |= setDoubleParam(ADAcquireTime, 1.0);
    status |= setIntegerParam(ADNumImages, 1);
    status |= setIntegerParam(ADNumImagesCounter, 0);
    status |= setIntegerParam(ADTriggerMode, TriggerModeFreeRun);

    // Set defaults of custom parameters
    status |= setIntegerParam(AcquirePedestalFunction, 0);
    status |= setIntegerParam(ReadoutModeFunction, ReadoutModeStandard);
    status |= setIntegerParam(PedestalNumImagesFunction, 1);
    status |= setIntegerParam(PedestalTimestampFunction, 0);
    status |= setIntegerParam(CoolerFunction, 0);

    m_rayonixDetector->SendParameters();

    printf("Opening Detector\n");
    craydl::RxReturnStatus error = m_rayonixDetector->Open();
    if (error.IsError())
    {
        printf("%s: Unable to open detector");
        return;
    }

    if (status)
    {
        printf("%s:%s epicsThreadCreate failure for image task\n",
            DRIVER_NAME, functionName);
        return;
    }
}

ADCrayDl::~ADCrayDl()
{
    m_running = false;
    if (m_pollingThread.joinable())
    {
        m_pollingThread.join();
    }

    // Close connection to the detector
    m_rayonixDetector->Close();
}

/** Configuration command, called directly or from iocsh */
extern "C" int ADCrayDlConfig(const char *portName, int dataType,
                                 int maxBuffers, int maxMemory, int priority, int stackSize)
{
    new ADCrayDl(portName, (NDDataType_t)dataType,
                    (maxBuffers < 0) ? 0 : maxBuffers,
                    (maxMemory < 0) ? 0 : maxMemory,
                    priority, stackSize);
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg ADCrayDlConfigArg0 = {"Port name", iocshArgString};
static const iocshArg ADCrayDlConfigArg1 = {"Data type", iocshArgInt};
static const iocshArg ADCrayDlConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg ADCrayDlConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg ADCrayDlConfigArg4 = {"priority", iocshArgInt};
static const iocshArg ADCrayDlConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg * const ADCrayDlConfigArgs[] =  {&ADCrayDlConfigArg0,
                                                          &ADCrayDlConfigArg1,
                                                          &ADCrayDlConfigArg2,
                                                          &ADCrayDlConfigArg3,
                                                          &ADCrayDlConfigArg4,
                                                          &ADCrayDlConfigArg5};
static const iocshFuncDef configADCrayDl = {"ADCrayDlConfig", 6, ADCrayDlConfigArgs};
static void configADCrayDlCallFunc(const iocshArgBuf *args)
{
    ADCrayDlConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival,
                      args[4].ival, args[5].ival);
}

static void ADCrayDlRegister(void)
{
    iocshRegister(&configADCrayDl, configADCrayDlCallFunc);
}

static int GetCurrentTime(subRecord *precord)
{
    precord->val = time(0);

    return 0;
}

extern "C"
{
epicsExportRegistrar(ADCrayDlRegister);
epicsRegisterFunction(GetCurrentTime);
}

} // namespace adcraydl
