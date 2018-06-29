#ifndef ADCRAYDLFRAMESYNCOBJECT_HXX
#define ADCRAYDLFRAMESYNCOBJECT_HXX

#include "epicsTime.h"
#include "timesync.h"

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
};

} // namespace adcraydl

#endif // ADCRAYDLFRAMESYNCOBJECT_HXX