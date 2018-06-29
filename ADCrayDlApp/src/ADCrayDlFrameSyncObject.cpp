#include "ADCrayDlFrameSyncObject.h"

namespace adcraydl
{

FrameSyncObject::FrameSyncObject()
    : SyncObject()
{
    SyncObject::poll();
}

DataObject *FrameSyncObject::Acquire(void)
{
    return new DataObject(NULL);
}

int FrameSyncObject::FidDiff(DataObject *dobj)
{
    return -1;
}

int FrameSyncObject::Attributes(void)
{
    return HasTime;
}

} // namespace adcraydl
