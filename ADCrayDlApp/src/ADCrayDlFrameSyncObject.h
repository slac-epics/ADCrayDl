#ifndef ADCRAYDLFRAMESYNCOBJECT_HXX
#define ADCRAYDLFRAMESYNCOBJECT_HXX

#include "epicsTime.h"
#include "timesync.h"
#include "ADCrayDlStorage.h"

#include <craydl/RxFrameMetaData.h>

namespace adcraydl
{

class FrameSyncObject : public SyncObject
{
public:
    FrameSyncObject();
    virtual ~FrameSyncObject() = default;

    virtual DataObject *Acquire(void);
    virtual int FidDiff(DataObject *dobj);
    virtual int Attributes(void);
    virtual int CheckError(DataObject *dobj);
    virtual void QueueData(DataObject *dobj, epicsTimeStamp &evtTime);

private:
    static Storage m_storage;  //!< Static class that stores the frames which need to be timestamped.

    craydl::RxTimestamp_t m_previousTimestamp;
};

} // namespace adcraydl

#endif // ADCRAYDLFRAMESYNCOBJECT_HXX