#ifndef ADCRAYDLFRAMESYNCOBJECT_HXX
#define ADCRAYDLFRAMESYNCOBJECT_HXX

#include "epicsTime.h"
#include "timesync.h"
#include "ADCrayDlStorage.h"

#include <craydl/RxFrameMetaData.h>

namespace adcraydl
{

/**
 * @brief This object waits for acquired image frames and adds them a timestamp.
 * 
 */
class FrameSyncObject : public SyncObject
{
public:
    FrameSyncObject();
    virtual ~FrameSyncObject() = default;

    /**
     * @brief Waits for frames and encapsulates them into a DataObject.
     * 
     * @return DataObject that contains the acquired frame.
     */
    virtual DataObject *Acquire(void);

    /**
     * @brief Calculates the number of fiducials that have occurred since the last frame was acquired.
     * 
     * @param dobj 
     * @return int 
     */
    virtual int FidDiff(DataObject *dobj);
    virtual int Attributes(void);
    virtual int CheckError(DataObject *dobj);

    /**
     * @brief Applies the timestamp to the frame and publishes it.
     * 
     * @param dobj 
     * @param evtTime 
     */
    virtual void QueueData(DataObject *dobj, epicsTimeStamp &evtTime);

private:
    static Storage m_storage;  //!< Static class that stores the frames which need to be timestamped.

    epicsTimeStamp m_previousTimestamp;
};

} // namespace adcraydl

#endif // ADCRAYDLFRAMESYNCOBJECT_HXX