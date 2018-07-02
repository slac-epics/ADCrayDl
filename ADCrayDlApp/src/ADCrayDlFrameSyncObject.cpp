#include "ADCrayDlFrameSyncObject.h"
#include <iostream>

namespace adcraydl
{

FrameSyncObject::FrameSyncObject()
    : SyncObject()
{
}

DataObject *FrameSyncObject::Acquire(void)
{
    std::cout << "--- Called acquire" << std::endl;

    NDArray *frame = NULL;
    m_storage.frameQueue.blockingPop(frame);

    std::cout << "Popped value" << std::endl;

    return new DataObject(frame);
}

int FrameSyncObject::FidDiff(DataObject *dobj)
{
    if (dobj == NULL)
    {
        std::cerr << "FidDiff got null data object!" << std::endl;
        return -1;
    }

    return -1;
}

int FrameSyncObject::Attributes(void)
{
    return HasTime;
}

void FrameSyncObject::QueueData(DataObject *dobj, epicsTimeStamp &evtTime)
{
    if (dobj == NULL)
    {
        std::cerr << "QueueData got null data object!" << std::endl;
        return;
    }

    std::cout << "Got queued data" << std::endl;

    NDArray *frame = static_cast<NDArray *>(dobj->data);

    // The module is done with the frame.
    frame->release();
    // The temporary data container also isn't needed anymore.
    delete dobj;
}

} // namespace adcraydl
