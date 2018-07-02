#ifndef ADCRAYDL_H
#define ADCRAYDL_H

#include <epicsEvent.h>
#include <epicsMutex.h>
#include <mutex>
#include <thread>

#include "ADDriver.h"
#include "craydl/RxDetector.h"
#include "craydl/RxFrame.h"
#include "ADCrayDlStorage.h"
#include "craydl/RxCallbacks.h"

#include "ADCrayDlFrameSyncObject.h"

#define DRIVER_VERSION      1
#define DRIVER_REVISION     0
#define DRIVER_MODIFICATION 0

namespace adcraydl
{

class AtomicBool
{
public:
    AtomicBool(const bool value)
        : m_value(value),
          m_mutex()
    {
    }

    bool operator=(const bool value)
    {
        std::lock_guard<epicsMutex> lock(m_mutex);
        m_value = value;
        return m_value;
    }

    void set(const bool value)
    {
        std::lock_guard<epicsMutex> lock(m_mutex);
        m_value = value;
    }

    bool get()
    {
        std::lock_guard<epicsMutex> lock(m_mutex);
        return m_value;
    }

private:
    bool m_value;
    epicsMutex m_mutex;   //!< Mutex used by the thread.
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
     * @brief Construct a new ADCrayDl object
     * 
     * @param portName 
     * @param dataType 
     * @param maxBuffers 
     * @param maxMemory 
     * @param priority 
     * @param stackSize 
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
     * @brief Task that takes care of the timestamping.
     * 
     * This task is started in a new thread. Should be private, but gets called from C, so must be public
     * @todo
     */
    void simTask();

    virtual void RawStatusChanged(const std::string &name, const std::string &value);
    virtual void ParameterChanged(const std::string &name, const std::string &value);

    void SequenceStarted();
    void SequenceEnded();
    void ExposureStarted(int frame_number);
    void ExposureEnded(int frame_number);
    void ReadoutStarted(int frame_number);
    void ReadoutEnded(int frame_number);
    void BackgroundFrameReady(const craydl::RxFrame *frame_p);
    void RawFrameReady(int frame_number, const craydl::RxFrame *frame_p);
    void FrameReady(int frame_number, const craydl::RxFrame *frame_p);
    void FrameAborted(int frame_number);
    void FrameCompleted(int frame_number);
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

    std::unique_ptr<craydl::RxDetector> m_rayonixDetector; //!< SDK detector object.
    static Storage m_storage;  //!< Static class that stores the frames which need to be timestamped.
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
    int IntervalTimeFunction;
    int EnableDetectorQueryingFunction;
    int BinningFunction;

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
    int IgnoreVacuumPumpFunction;
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
