#include "ADCrayDlFrameSyncObject.h"
#include <iostream>

namespace
{
    static const double FIDUCIAL_RATE = 360.0;
}

namespace adcraydl
{

FrameSyncObject::FrameSyncObject()
    : SyncObject(),
      m_previousTimestamp({0,0})
{
}

DataObject *FrameSyncObject::Acquire(void)
{
    std::cout << "--- Called acquire" << std::endl;

    NDArray *frame;
    m_storage.frameQueue.blockingPop(frame);

    std::cout << "Popped value" << std::endl;

    // TODO: FIXME: remove this, this is just for development
    m_storage.timestampedFrameQueue.push(frame);

    return new DataObject(frame);
}

int FrameSyncObject::FidDiff(DataObject *dobj)
{
    std::cout << "Doing FidDiff" << std::endl;

    assert(dobj != NULL);

    NDArray *frame = static_cast<NDArray *>(dobj->data);

    // frame->epicsTS contains the hardware timestamp from the acquired frame.
    const double timeDiffS = epicsTimeDiffInSeconds(&frame->epicsTS, &m_previousTimestamp);
    m_previousTimestamp = frame->epicsTS;

    return round(timeDiffS / FIDUCIAL_RATE);
}

int FrameSyncObject::Attributes(void)
{
    return HasTime;
}

int FrameSyncObject::CheckError(DataObject *dobj)
{
    return 0;
}

void FrameSyncObject::QueueData(DataObject *dobj, epicsTimeStamp &evtTime)
{
    std::cout << "Got queued data" << std::endl;

    assert(dobj != NULL);

    NDArray *frame = static_cast<NDArray *>(dobj->data);

    // Set timestamp of frame.
    frame->epicsTS = evtTime;

    // Tell the main class that it should publish the data.
    m_storage.timestampedFrameQueue.push(frame);
}

} // namespace adcraydl
