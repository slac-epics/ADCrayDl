/**
 * @brief This file contains the class definition for the ADCrayDl and AtomicBool classes.
 * 
 * @file ADCrayDl.h
 * @author Domen Soklic (domen.soklic@cosylab.com)
 * @date 2018-08-02
 */

#ifndef ADCRAYDL_H
#define ADCRAYDL_H

#include <epicsEvent.h>
#include <epicsMutex.h>
#include <mutex>
#include <thread>

#include "ADDriver.h"
#include "craydl/RxDetector.h"
#include "craydl/RxFrame.h"
#include "craydl/RxCallbacks.h"

namespace adcraydl
{

static const uint32_t DRIVER_VERSION      = 1;
static const uint32_t DRIVER_REVISION     = 0;
static const uint32_t DRIVER_MODIFICATION = 0;

/**
 * @brief This class ensures concurrent access to a boolean variable.
 */
class AtomicBool
{
public:
    /**
     * @brief Construct a new Atomic Bool object
     * 
     * @param value Initial value.
     */
    AtomicBool(const bool value)
        : m_value(value),
          m_mutex()
    {
    }

    /**
     * @brief Concurrent setter.
     * 
     * @param value The value that should be set.
     * @return Value that was set.
     */
    bool operator=(const bool value)
    {
        std::lock_guard<epicsMutex> lock(m_mutex);
        m_value = value;
        return m_value;
    }

    /**
     * @brief Concurrent setter.
     * 
     * @param value The value that should be set.
     */
    void set(const bool value)
    {
        std::lock_guard<epicsMutex> lock(m_mutex);
        m_value = value;
    }

    /**
     * @brief Concurrent getter.
     * 
     * @return Stored value.
     */
    bool get()
    {
        std::lock_guard<epicsMutex> lock(m_mutex);
        return m_value;
    }

private:
    bool m_value;       //!< Value stored in this class.
    epicsMutex m_mutex; //!< Mutex used by the thread.
};

/**
 * @brief areaDetector module for the Rayonix CrayDl SDK and its supported detectors.
 */
class epicsShareClass ADCrayDl : public ADDriver,
                                 public craydl::VirtualFrameCallback,
                                 public craydl::VirtualKeyedStateChangeCallback
{
public:
    /**
     * @brief Construct a new ADCrayDl object.
     */
    ADCrayDl(const char *portName, NDDataType_t dataType,
                int maxBuffers, size_t maxMemory,
                int priority, int stackSize);

    /**
     * @brief Destroy the ADCrayDl object.
     */
    virtual ~ADCrayDl();

    /**
     * @brief Called when asyn writes integer data.
     * 
     * Called when asyn clients call pasynInt32->write(). 
     * 
     * @see asynPortDriver::writeInt32()
     * @param pasynUser   Structure that encodes the reason and address. 
     * @param value       Value to write. 
     * @return asynStatus Success or failure status of the operation.
     */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    /**
     * @brief Called when asyn writes float data.
     * 
     * Called when asyn clients call pasynFloat64->write().
     * 
     * @see asynPortDriver::writeFloat64()
     * @param pasynUser   Structure that encodes the reason and address. 
     * @param value       Value to write. 
     * @return asynStatus Success or failure status of the operation.
     */
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    /**
     * @brief Called when a virtual status changes (e.g. detector temperature).
     * 
     * @param vstatus The virtual status that changed.
     */
    virtual void VirtualStatusChanged(const craydl::VStatusParameter *vstatus);

    /**
     * @brief Called when a status flag changes (e.g. cooler turned on).
     * 
     * @param vstatus The status flag that changed.
     */
    virtual void StatusFlagChanged(const craydl::VStatusFlag *vstatus);

    /**
     * @brief Executes when acquisition sequence starts.
     */
    void SequenceStarted();

    /**
     * @brief Executes when acquisition sequence ends.
     */
    void SequenceEnded();

    /**
     * @brief Executes when exposure starts.
     * 
     * @param frame_number Number of the frame that exposure started for.
     */
    void ExposureStarted(int frame_number);

    /**
     * @brief Executes when exposure ends.
     * 
     * @param frame_number Number of the frame that exposure ended for.
     */
    void ExposureEnded(int frame_number);

    /**
     * @brief Executes when readout starts.
     * 
     * @param frame_number Number of the frame that readout started for.
     */
    void ReadoutStarted(int frame_number);

    /**
     * @brief Executes when readout ends.
     * 
     * @param frame_number Number of the frame that readout ended for.
     */
    void ReadoutEnded(int frame_number);

    /**
     * @brief Executes when the background frame is ready.
     * 
     * @param frame_p Acquired background frame.
     */
    void BackgroundFrameReady(const craydl::RxFrame *frame_p);

    /**
     * @brief Executes when raw frame data is ready for further processing after readout.
     * 
     * @param frame_number Number of the raw frame.
     * @param frame_p Acquired raw frame.
     */
    void RawFrameReady(int frame_number, const craydl::RxFrame *frame_p);

    /**
     * @brief Executes when final (corrected) frame data is ready for access after readout.
     * 
     * @param frame_number Number of the frame.
     * @param frame_p Acquired frame.
     */
    void FrameReady(int frame_number, const craydl::RxFrame *frame_p);

    /**
     * @brief Executes when frame is aborted.
     * 
     * @param frame_number Number of the aborted frame.
     */
    void FrameAborted(int frame_number);

    /**
     * @brief Executes when frame is completed.
     * 
     * @param frame_number Number of the completed frame.
     */
    void FrameCompleted(int frame_number);

    /**
     * @brief Executes when an error is generated for a frame.
     * 
     * @param frame_number Number of the frame.
     * @param frame_p Frame with the error.
     * @param error_code The code indicating the error.
     * @param error_string A string describing the error.
     */
    void FrameError(int frame_number, const craydl::RxFrame *frame_p, int error_code, const std::string &error_string);

private:
    static const size_t NUM_DIMS = 2; //!< Number of dimensions in image.

    enum TriggerMode
    {
        TriggerModeFreeRun  = 0,
        TriggerModeEdge     = 1,
        TriggerModeBulb     = 2,
        TriggerModeLCLSMode = 3
    };

    enum ReadoutMode
    {
        ReadoutModeStandard = 0,
        ReadoutModeLowNoise = 1
    };

    enum BinningMode
    {
        BinningMode1x1   = 0,
        BinningMode2x2   = 1,
        BinningMode3x3   = 2,
        BinningMode4x4   = 3,
        BinningMode6x6   = 4,
        BinningMode10x10 = 5
    };

    /**
     * @brief Task that polls detector for current status.
     * 
     * @param interval_us Interval how often the detector should be polled (in us).
     */
    void pollDetectorStatus(const uint32_t interval_us);

    /**
     * @brief Takes the frame and copies it to an NDArray. It then submits the data to subscribed plugins.
     * 
     * @param frame_p Frame with the image.
     */
    void applyFrameToAD(const craydl::RxFrame *frame_p);

    /**
     * @brief Updates dimension structures based on current binning mode.
     * 
     * @return int Success status.
     */
    int updateDimensionSize();

    /**
     * @brief This method checks if the provided function corresponds to a cooling PV and tries to handle it.
     * 
     * @param function     Function of the write operation.
     * @param value        Value being written.
     * @param status [out] Used to return status.
     * @return             True when function was handled by this method, false when not.
     */
    bool handleCoolingPV(const int function, const epicsInt32 value, asynStatus &status);

    /**
     * @brief This method checks if the provided function corresponds to a cooling PV and tries to handle it.
     * 
     * @param function     Function of the write operation.
     * @param value        Value being written.
     * @param status [out] Used to return status.
     * @return             True when function was handled by this method, false when not.
     */
    bool handleCoolingPV(const int function, const epicsFloat64 value, asynStatus &status);

    /**
     * @brief This method checks if the provided function corresponds to a vacuum PV and tries to handle it.
     * 
     * @param function     Function of the write operation.
     * @param value        Value being written.
     * @param status [out] Used to return status.
     * @return             True when function was handled by this method, false when not.
     */
    bool handleVacuumPV(const int function, const epicsInt32 value, asynStatus &status);

    /**
     * @brief Increases the array count by one.
     */
    void increaseArrayCounter();

    /**
     * @brief Helper method used to get the number of images to acquire.
     * 
     * The output of this method is passed to @ref craydl::RxDetector::SetupAcquisitionSequence().
     * Values <= 0 indicate an infinite number of images.
     * 
     * @return int Number of images to acquire.
     */
    int getNumImagesToAcquire();

    /**
     * @brief Register all virtual status and status flag change callbacks.
     */
    void registerAllStatusCallbacks();

    /**
     * @brief Converts a posix time variable to seconds since Epics epoch.
     * 
     * @param posixTime Input posix time.
     * @return time_t Seconds since Epics epoch.
     */
    static time_t toEpicsSecondsSinceEpoch(const boost::posix_time::ptime &posixTime);

    /**
     * @brief This method creates a time string from the provided seconds since Epics epoch timestamp.
     * 
     * @param timestamp Seconds since Epics epoch.
     * @return std::string Formated time string.
     */
    static std::string secondsSinceEpochToString(const time_t timestamp);

    std::unique_ptr<craydl::RxDetector> m_rayonixDetector; //!< SDK detector object.
    NDDimension_t m_dimsOut[NUM_DIMS]; //!< Array of dimension properties.
    size_t m_dims[NUM_DIMS];           //!< Array of dimension sizes.

    // Polling thread
    AtomicBool m_running;        //!< Flag that indicates if the polling thread should keep running or stop.
    std::thread m_pollingThread; //!< Thread that polls the detector.

    // Custom parameters
    int AcquirePedestalFunction;
    int ReadoutModeFunction;
    int PedestalNumImagesFunction;
    int PedestalTimestampFunction;
    int StringPedestalTimestampFunction;
    int IntervalTimeFunction;
    int EnableDetectorQueryingFunction;
    int BinningFunction;
    int ShutterStatusFunction;

    // Cooling
    int CoolerFunction;
    int CCDTempSetpointFunction;
    int CoolerTempSetpointFunction;
    int MeanSensorTempFunction;
    int MaxSensorTempFunction;
    int MinSensorTempFunction;
    int MeanCoolerTempFunction;
    int MaxCoolerTempFunction;
    int MinCoolerTempFunction;
    int CoolerEnabledFunction;
    int CoolerRunningFunction;

    // Vacuum
    int VacuumValveFunction;
    int LinePressureFunction;
    int ChamberPressureFunction;
    int VacuumValveOpenFunction;
    int VacuumValveEnabledFunction;
    int VacuumPumpRunningFunction;
    int VacuumPumpIgnoredFunction;

    // State variables
    int m_numImages;
    int m_numPedestals;
};

} // namespace adcraydl

#endif // ADCRAYDL_H
