/**
 * @brief Definition of the ADCRayDl class.
 * 
 * @file ADCrayDl.cpp
 * @author Domen Soklic (domen.soklic@cosylab.com)
 * @date 2018-08-02
 */

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
    static const char *DRIVER_NAME = "ADCrayDl"; //!< Name of the driver used in logging.
    static const double POLLING_INTERVAL_S = 1.0; //!< Polling interval.
    static const boost::posix_time::ptime EPICS_EPOCH(boost::gregorian::date(1990, 1, 1)); //!< Epics epoch is 1. January 1990.
    static const char *TIMESTAMP_FORMAT = "%a %b %d %Y %H:%M:%S"; //!< Timestamp format used for pedestal.
}

namespace adcraydl
{

// REVIEW: There's a lot of std::cerr usage for printing errors. With this it is
//         unclear where errors are coming from (which module, which device).
//         Have you considered the logging mechanisms provided by asynPortDriver?

void ADCrayDl::pollDetectorStatus(const double interval_s)
{
    while (m_running.get())
    {
        // REVIEW: Explain what is happening here. Where does the queried status
        //         go? Does this synchonously call some of the callbacks?

        const craydl::RxReturnStatus status = m_rayonixDetector->QueryStatus();
        if (status.IsError())
        {
            std::cerr << "Error querying status: " << status.ErrorText() << std::endl;
        }

        epicsThreadSleep(interval_s);
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

bool ADCrayDl::handleCoolingPV(const int function, const epicsFloat64 value, asynStatus &status)
{
    if (function == CCDTempSetpointFunction)
    {
        const craydl::RxReturnStatus error = m_rayonixDetector->SetSensorTemperatureSetpoint(value);
        
        if (error.IsError())
        {
            std::cerr << "Error setting CCD temperature setpoint: " << error.ErrorText() << std::endl;
            status = asynError;
        }
        
        return true;
    }
    else if (function == CoolerTempSetpointFunction)
    {
        const craydl::RxReturnStatus error = m_rayonixDetector->SetColdHeadTemperatureSetpoint(value);

        if (error.IsError())
        {
            std::cerr << "Error setting cooler temperature setpoint: " << error.ErrorText() << std::endl;
            status = asynError;
        }

        return true;
    }

    return false;
}

int ADCrayDl::getNumImagesToAcquire()
{
    int imageMode;
    const int status = getIntegerParam(ADImageMode, &imageMode);
    assert(status == 0);

    switch (imageMode)
    {
        case ADImageSingle:
            return 1;
        
        case ADImageContinuous:
            return 0; // This means that the detector should just keep going.

        case ADImageMultiple:
            return m_numImages;

        default:
            throw std::runtime_error("Unknown image mode value not in enum");
    }
}

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADColorMode, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus ADCrayDl::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    const int function = pasynUser->reason;

    // REVIEW: I think it is not appropriate to unconditionally print this to cout,
    //         the logging should be somehow configurable. The people who maintain
    //         IOCs will not like the console being spammed. Actually, IIRC asyn
    //         has some built-in logging of parameter reads and writes, but I don't
    //         know the details. Maybe look at how popular / good quality drivers
    //         do it.
    std::cout << "writeInt32 func: " << function << ", value: " << value << std::endl;

    // Set the parameter
    asynStatus status = ADDriver::writeInt32(pasynUser, value);

    // REVIEW: Are the various configuration changes safe while acquisition is active?

    /* This is where the parameter is sent to the SDK */
    if (function == ADAcquire)
    {
        callParamCallbacks();

        printf("%s: Detected change of ADAcquire to %d\n", DRIVER_NAME, value );

        if (value)
        {
            // REVIEW: Is it safe to do all these things if acquisition is already started?
            //         Maybe you could try to stop acquisition if already active, or just
            //         reject the command.

            do // This do-while is a convenient solution to error handling. We can jump out of the block early.
            {
                craydl::RxReturnStatus error = m_rayonixDetector->SetupAcquisitionSequence(getNumImagesToAcquire());
                if (error.IsError())
                {
                    std::cerr << "Could not setup acquisition sequence to " << m_numImages << " images" << std::endl;
                    status = asynError;
                    continue; // Skip other code.
                }

                error = m_rayonixDetector->SendParameters();
                if (error.IsError())
                {
                    std::cerr << "Could not send parameters to detector" << std::endl;
                    status = asynError;
                    continue; // Skip other code.
                }

                // Starts Acquisition of series of frames - Light means operated shutter I/O output signal
                craydl::FrameAcquisitionType frame_type = craydl::FrameAcquisitionTypeLight;
                error = m_rayonixDetector->StartAcquisition(frame_type);
                if (error.IsError())
                {
                    std::cerr << "Could not start acquisition" << std::endl;
                    status = asynError;
                    continue; // Skip other code.
                }
            } while (false);

            // REVIEW: Seems like you allow any non-zero value to start acquisition and
            //         will actually set ADAcquire to whatever value was written, but
            //         I don't see any different meaning. Why not call
            //         setIntegerParam(ADAcquire, 1) here to make sure the stored value
            //         is always 0 or 1?
        }
        else
        {
            // REVIEW: Explain "true" argument (e.g. add inline comment like /*argumentName=*/true).
            // Stop acquisition
            craydl::RxReturnStatus error = m_rayonixDetector->EndAcquisition(true);
            if (error.IsError())
            {
                std::cerr << "Could not end acquisition" << std::endl;
                status = asynError;
            }

            // REVIEW: Variable shadows another variable from parent scope, rename it.
            const int status = setIntegerParam(ADAcquire, 0);
            assert(status == 0);
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
            // REVIEW: Error on unknown value?
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
            // REVIEW: Error on unknown value or unsupported LCLSMode?
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
            // REVIEW: Error on unknown value?
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

        // REVIEW: Explain "false" argument (e.g. add inline comment like /*argumentName=*/false).
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
            m_pollingThread = std::thread(&ADCrayDl::pollDetectorStatus, this, POLLING_INTERVAL_S);
        }
    }
    else if (function == TriggerSignalTypeFunction)
    {
        craydl::DigitalIOSignalType_t digitalIOSignalType;

        switch (value)
        {
            case DigitalIOSignalTypeNone:
            default:
                digitalIOSignalType = craydl::DigitalIOSignalTypeNone;
                break;

            case DigitalIOSignalTypeOpto:
                digitalIOSignalType = craydl::DigitalIOSignalTypeOpto;
                break;

            case DigitalIOSignalTypeOptoInverted:
                digitalIOSignalType = craydl::DigitalIOSignalTypeOptoInverted;
                break;

            case DigitalIOSignalTypeCMOS:
                digitalIOSignalType = craydl::DigitalIOSignalTypeCMOS;
                break;

            case DigitalIOSignalTypeCMOSPullDown:
                digitalIOSignalType = craydl::DigitalIOSignalTypeCMOSPullDown;
                break;

            case DigitalIOSignalTypeCMOSPullUp:
                digitalIOSignalType = craydl::DigitalIOSignalTypeCMOSPullUp;
                break;

            case DigitalIOSignalTypeCMOSPullDownInverted:
                digitalIOSignalType = craydl::DigitalIOSignalTypeCMOSPullDownInverted;
                break;

            case DigitalIOSignalTypeCMOSPullUpInverted:
                digitalIOSignalType = craydl::DigitalIOSignalTypeCMOSPullUpInverted;
                break;

            case DigitalIOSignalTypeSoftware:
                digitalIOSignalType = craydl::DigitalIOSignalTypeSoftware;
                break;
        }

        const craydl::RxReturnStatus error = m_rayonixDetector->SetFrameTriggerSignalType(craydl::DigitalIOSignalType(digitalIOSignalType));
        if (error.IsError())
        {
            std::cerr << "Error setting trigger signal type " << value << ", error: " << error.ErrorText() << std::endl;
            status = asynError;
        }
    }
    else if (SoftwareBulbFunction)
    {
        if (value)
        {
            const craydl::RxReturnStatus error = m_rayonixDetector->SqueezeBulb();
            if (error.IsError())
            {
                std::cerr << "Error calling SqueezeBulb " << value << ", error: " << error.ErrorText() << std::endl;
                status = asynError;
            }
        }
        else
        {
            const craydl::RxReturnStatus error = m_rayonixDetector->ReleaseBulb();
            if (error.IsError())
            {
                std::cerr << "Error calling ReleaseBulb " << value << ", error: " << error.ErrorText() << std::endl;
                status = asynError;
            }
        }
    }
    else // Check if these are cooling of vacuum PVs
    {
        const bool isCooling = handleCoolingPV(function, value, status);
        if (!isCooling)
        {
            handleVacuumPV(function, value, status);
        }
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

    // REVIEW: Are the various configuration changes safe while acquisition is active?

    // Set the parameter and readback in the parameter library. This may be overwritten when we read back the
    // status at the end, but that's OK.
    asynStatus status = ADDriver::writeFloat64(pasynUser, value);

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
    else // Check if these are cooling PVs
    {
        handleCoolingPV(function, value, status);
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

// REVIEW: You must hold the asynPortDriver lock when calling functions things like set*Param,
//         get*Param, callParamCallbacks and updateTimeStamp (for the latter I'm not sure).
//         Also it it is unclear to me if any of these callbacks could be called from
//         requests to the driver (in which case the port already be locked in the callback).
//         You need to precisely understand these calls and ensure that the driver is locked
//         when calling the mentioned asynPortDriver functions, and I think trying to lock a
//         port if already locked would also be an issue.
// NOTE: Once the callbacks are fixed to lock the port, we introduce possibility for
//       deadlock in m_pollingThread.join() above, under the assumption that
//       QueryStatus() may call the callbacks. If you know that not to be true then,
//       there is no issue, otherwise it needs to be addressed. I see two fixes:
//       - Easy fix: Unlock and re-lock the driver around the join.
//       - Harder fix (and easier to mess up): Only ever start one thread and
//         synchronize with it to start and stop polling as needed, with the help of
//         condition_variable or epicsEvent.

void ADCrayDl::VirtualStatusChanged(const craydl::VStatusParameter *vstatus)
{
    // Conversion from string to double will throw an exception if the conversion isn't successful.
    switch (vstatus->key())
    {
        case craydl::StatusParameterSensorTemperatureMin:
            setDoubleParam(MinSensorTempFunction, std::stod(vstatus->value()));
            break;

        case craydl::StatusParameterSensorTemperatureMax:
            setDoubleParam(MaxSensorTempFunction, std::stod(vstatus->value()));
            break;

        case craydl::StatusParameterSensorTemperatureAve:
            setDoubleParam(MeanSensorTempFunction, std::stod(vstatus->value()));
            break;

        case craydl::StatusParameterColdHeadTemperatureMin:
            setDoubleParam(MinCoolerTempFunction, std::stod(vstatus->value()));
            break;

        case craydl::StatusParameterColdHeadTemperatureMax:
            setDoubleParam(MaxCoolerTempFunction, std::stod(vstatus->value()));
            break;

        case craydl::StatusParameterColdHeadTemperatureAve:
            setDoubleParam(MeanCoolerTempFunction, std::stod(vstatus->value()));
            break;

        case craydl::StatusParameterLinePressure:
            setDoubleParam(LinePressureFunction, std::stod(vstatus->value()));
            break;

        case craydl::StatusParameterChamberPressure:
            setDoubleParam(ChamberPressureFunction, std::stod(vstatus->value()));
            break;

        default:
            // Do nothing.
            return;
    }

    callParamCallbacks();
}

void ADCrayDl::StatusFlagChanged(const craydl::VStatusFlag *vstatus)
{
    switch (vstatus->key())
    {
        case craydl::StatusFlagCoolersEnabled:
            setIntegerParam(CoolerEnabledFunction, vstatus->value());
            break;

        case craydl::StatusFlagCoolersRunning:
            setIntegerParam(CoolerRunningFunction, vstatus->value());
            break;

        case craydl::StatusFlagVacuumValveOpen:
            setIntegerParam(VacuumValveOpenFunction, vstatus->value());
            break;

        case craydl::StatusFlagVacuumValveEnabled:
            setIntegerParam(VacuumValveEnabledFunction, vstatus->value());
            break;

        case craydl::StatusFlagVacuumPumpRunning:
            setIntegerParam(VacuumPumpRunningFunction, vstatus->value());
            break;

        case craydl::StatusFlagVacuumPumpIgnored:
            setIntegerParam(VacuumPumpIgnoredFunction, vstatus->value());
            break;

        case craydl::StatusFlagOSShutterOpen:
            setIntegerParam(ShutterStatusFunction, vstatus->value());
            break;

        default:
            // Do nothing.
            return;
    }

    callParamCallbacks();
}

void ADCrayDl::SequenceStarted()
{
    // Intentionally empty.
}

void ADCrayDl::SequenceEnded()
{
    const int status = setIntegerParam(ADAcquire, 0);
    assert(status == 0);

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
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

time_t ADCrayDl::toEpicsSecondsSinceEpoch(const boost::posix_time::ptime &posixTime)
{
    boost::posix_time::time_duration diff(posixTime - EPICS_EPOCH);
    const time_t localTime = (diff.ticks() / diff.ticks_per_second());

    std::tm local_field = *std::gmtime(&localTime);
    local_field.tm_isdst = -1;
    time_t utc = std::mktime(&local_field);

    return utc;
}

std::string ADCrayDl::secondsSinceEpochToString(const time_t timestamp)
{
    char timeString[32];

    // REVIEW: I would use a function-style cast here.
    const epicsTimeStamp stamp = { static_cast<epicsUInt32>(timestamp), 0 };
    epicsTimeToStrftime(timeString, sizeof(timeString), TIMESTAMP_FORMAT, &stamp);

    return std::string(timeString);
}

// REVIEW: Is BackgroundFrameReady called from within AcquireNewBackground? It matters
//         because it determines whether you need to lock the driver here.
void ADCrayDl::BackgroundFrameReady(const craydl::RxFrame *frame_p)
{
    // Handle pedestals
    const craydl::FrameMetaData &metadata = frame_p->metaData();

    const time_t timestamp = toEpicsSecondsSinceEpoch(metadata.AcquisitionStartTimestamp());
    int status = setIntegerParam(PedestalTimestampFunction, timestamp);
    assert(status == 0);

    status = setStringParam(StringPedestalTimestampFunction, secondsSinceEpochToString(timestamp));
    assert(status == 0);

    status = setIntegerParam(AcquirePedestalFunction, 0);
    assert(status == 0);

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

// REVIEW: Why not put this code directly into FrameReady?
void ADCrayDl::applyFrameToAD(const craydl::RxFrame *frame_p)
{
    // Get timestamp of frame
    epicsTimeStamp newEvrTime;
    updateTimeStamp(&newEvrTime);

    NDArray *inArray = pNDArrayPool->alloc(NUM_DIMS, m_dims, NDUInt16, frame_p->getSize(), NULL);

    // REVIEW: The m_dimsOut and initDimension seems to do nothing. The documentation
    //         for initDimension says: "Initializes the dimension structure to size=size,
    //         binning=1, reverse=0, offset=0.". So you're initializing these NDDimension_t
    //         structures here and also in updateDimensionSize, and you use them nowhere?
    for (size_t i = 0; i < NUM_DIMS; i++)
    {
        inArray->initDimension(&m_dimsOut[i], m_dims[i]);
    }

    // REVIEW: Use std::memcpy and include <cstring>.
    // The dimensions are the same and the data type is the same, just copy the input image to the output image.
    memcpy(inArray->pData, frame_p->getBufferAddress(), frame_p->getSize());

    // Set timestamp
    inArray->epicsTS = newEvrTime;

    // std::cout << "The new timestamp is " << inArray->epicsTS.secPastEpoch << std::endl;

    increaseArrayCounter();

    // Call callbacks.
    int arrayCallbacks;
    const int status = getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    assert(status == 0);

    if (arrayCallbacks != 0)
    {
        doCallbacksGenericPointer(inArray, NDArrayData, 0);
    }

    // The module is done with the frame.
    inArray->release();

    // REVIEW: Note that some drivers save the last frame into pArrays[0],
    //         that allows EPICS to specifically read the latest frame even while
    //         array callbacks are disabled (using record with SCAN set to
    //         something other than I/O Intr). Consider if you want to do that.
}

// REVIEW: updateDimensionSize returns an error code which is never checked.
//         I suggest to remove the return value and just call setIntegerParam
//         ignoring its the return value. I think that if setIntegerParam does
//         fail (which it should not if things are configured right), async
//         will itself log an error - which is fine if you yourself have no
//         intention to react to an error.
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
    // REVIEW: Signed integer overflow is undefined behavior - check for max value
    //         and wrap manually.

    int arrayCounter;
    int status = getIntegerParam(NDArrayCounter, &arrayCounter);
    ++arrayCounter;
    status |= setIntegerParam(NDArrayCounter, arrayCounter);

    int imageCounter;
    status |= getIntegerParam(ADNumImagesCounter, &imageCounter);
    ++imageCounter;
    status |= setIntegerParam(ADNumImagesCounter, imageCounter);

    assert(status == 0);

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
}

void ADCrayDl::registerAllStatusCallbacks()
{
    const std::set<craydl::VStatusParameter*> statusSet = m_rayonixDetector->SupportedStatusSet();

    for (std::set<craydl::VStatusParameter*>::const_iterator si = statusSet.begin(); si != statusSet.end(); si++)
    {
        m_rayonixDetector->RegisterStateChangeCallback(*si, this);
    }

    const std::set<craydl::VStatusFlag*> statusFlagSet = m_rayonixDetector->SupportedStatusFlagSet();

    for (std::set<craydl::VStatusFlag*>::const_iterator si = statusFlagSet.begin(); si != statusFlagSet.end(); si++)
    {
        m_rayonixDetector->RegisterStateChangeCallback(*si, this);
    }
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
    // REVIEW: Add comments to make it clear what is being passed, like /*maxAddr=*/1
    //         where this is not already clear.
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
    const char *functionName = "ADCrayDl";

    // Create custom parameters
    status |= createParam(ACQUIRE_PEDESTAL_STR,          asynParamInt32, &AcquirePedestalFunction);
    status |= createParam(READOUT_MODE_STR,              asynParamInt32, &ReadoutModeFunction);
    status |= createParam(PEDESTAL_NUM_IMG_STR,          asynParamInt32, &PedestalNumImagesFunction);
    status |= createParam(PEDESTAL_TIMESTAMP_STR,        asynParamInt32, &PedestalTimestampFunction);
    status |= createParam(STRING_PEDESTAL_TIMESTAMP_STR, asynParamOctet, &StringPedestalTimestampFunction);
    status |= createParam(ENABLE_DETECTOR_QUERYING_STR,  asynParamInt32, &EnableDetectorQueryingFunction);
    status |= createParam(BINNING_STR,                   asynParamInt32, &BinningFunction);
    status |= createParam(SHUTTER_STATUS_STR,            asynParamInt32, &ShutterStatusFunction);
    status |= createParam(TRIGGER_SIGNAL_TYPE_STR,       asynParamInt32, &TriggerSignalTypeFunction);
    status |= createParam(SOFTWARE_BULB_STR,             asynParamInt32, &SoftwareBulbFunction);

    // Cooling
    status |= createParam(COOLER_STR,               asynParamInt32,   &CoolerFunction);
    status |= createParam(CCD_TEMP_SETPOINT_STR,    asynParamFloat64, &CCDTempSetpointFunction);
    status |= createParam(COOLER_TEMP_SETPOINT_STR, asynParamFloat64, &CoolerTempSetpointFunction);
    status |= createParam(MEAN_SENSOR_TEMP_STR,     asynParamFloat64, &MeanSensorTempFunction);
    status |= createParam(MAX_SENSOR_TEMP_STR,      asynParamFloat64, &MaxSensorTempFunction);
    status |= createParam(MIN_SENSOR_TEMP_STR,      asynParamFloat64, &MinSensorTempFunction);
    status |= createParam(MEAN_COOLER_TEMP_STR,     asynParamFloat64, &MeanCoolerTempFunction);
    status |= createParam(MAX_COOLER_TEMP_STR,      asynParamFloat64, &MaxCoolerTempFunction);
    status |= createParam(MIN_COOLER_TEMP_STR,      asynParamFloat64, &MinCoolerTempFunction);
    status |= createParam(COOLER_ENABLED_STR,       asynParamInt32,   &CoolerEnabledFunction);
    status |= createParam(COOLER_RUNNING_STR,       asynParamInt32,   &CoolerRunningFunction);

    // Vacuum
    status |= createParam(VACUUM_VALVE_STR,         asynParamInt32,   &VacuumValveFunction);
    status |= createParam(LINE_PRESSURE_STR,        asynParamFloat64, &LinePressureFunction);
    status |= createParam(CHAMBER_PRESSURE_STR,     asynParamFloat64, &ChamberPressureFunction);
    status |= createParam(VACUUM_VALVE_OPEN_STR,    asynParamInt32,   &VacuumValveOpenFunction);
    status |= createParam(VACUUM_VALVE_ENABLED_STR, asynParamInt32,   &VacuumValveEnabledFunction);
    status |= createParam(VACUUM_PUMP_RUNNING_STR,  asynParamInt32,   &VacuumPumpRunningFunction);
    status |= createParam(VACUUM_PUMP_IGNORED_STR,  asynParamInt32,   &VacuumPumpIgnoredFunction);

    if (status)
    {
        printf("%s: unable to set camera parameters\n", functionName);
        return;
    }

    // Set callback on detector.
    m_rayonixDetector->RegisterFrameCallback(this);

    m_rayonixDetector->SetBackgroundFrameTriggerMode(craydl::FrameTriggerType(craydl::FrameTriggerTypeNone));
    m_rayonixDetector->SetFrameTriggerMode(craydl::FrameTriggerType(craydl::FrameTriggerTypeNone));

    std::string detectorModel;
    std::string detectorVersion;
    m_rayonixDetector->GetDetectorID(detectorModel, detectorVersion);

    std::string detectorFirmwareName;
    std::string detectorFirmwareVersion;
    m_rayonixDetector->GetDetectorFirmwareID(detectorFirmwareName, detectorFirmwareVersion);

    updateDimensionSize();

    // Create driver version string.
    std::stringstream driverVersionSS;
    driverVersionSS << DRIVER_VERSION << ".";
    driverVersionSS << DRIVER_REVISION << ".";
    driverVersionSS << DRIVER_MODIFICATION;

    // Set defaults
    status |= setStringParam(NDDriverVersion, driverVersionSS.str());
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
    status |= setIntegerParam(NDBitsPerPixel, 16);

    // Set defaults of custom parameters
    status |= setIntegerParam(AcquirePedestalFunction, 0);
    status |= setIntegerParam(ReadoutModeFunction, ReadoutModeStandard);
    status |= setIntegerParam(PedestalNumImagesFunction, 1);
    status |= setIntegerParam(PedestalTimestampFunction, 0);
    status |= setIntegerParam(TriggerSignalTypeFunction, DigitalIOSignalTypeOpto);
    status |= setStringParam(StringPedestalTimestampFunction, "");

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    m_rayonixDetector->SendParameters();

    // REVIEW: Use std::printf.
    printf("Opening Detector\n");
    craydl::RxReturnStatus error = m_rayonixDetector->Open();
    if (error.IsError())
    {
        printf("%s: Unable to open detector", error.ErrorText().c_str());
        return;
    }

    status |= setDoubleParam(CCDTempSetpointFunction, m_rayonixDetector->SensorTemperatureSetpoint());
    status |= setDoubleParam(CoolerTempSetpointFunction, m_rayonixDetector->ColdHeadTemperatureSetpoint());

    if (status)
    {
        // REVIEW: Bad error message.
        printf("%s:%s epicsThreadCreate failure for image task\n",
            DRIVER_NAME, functionName);
        return;
    }

    registerAllStatusCallbacks();
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

extern "C"
{
    epicsExportRegistrar(ADCrayDlRegister);
}

} // namespace adcraydl
