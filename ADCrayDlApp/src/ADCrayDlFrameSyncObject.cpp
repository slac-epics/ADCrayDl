#include "ADCrayDlFrameSyncObject.h"
#include <iostream>

namespace adcraydl
{

FrameSyncObject::FrameSyncObject()
    : SyncObject(),
      m_previousTimestamp(boost::posix_time::second_clock::local_time())
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
    std::cout << "Doing FidDiff" << std::endl;

    if (dobj == NULL)
    {
        std::cerr << "FidDiff got null data object!" << std::endl;
        return -1;
    }

    return 4; // TODO return a real value
}

int FrameSyncObject::Attributes(void)
{
    return HasTime;
}

int FrameSyncObject::CheckError(DataObject *dobj)
{
    std::cout << "CheckError" << std::endl;
    return 0;
}

void FrameSyncObject::QueueData(DataObject *dobj, epicsTimeStamp &evtTime)
{
    std::cout << "Got queued data" << std::endl;

    if (dobj == NULL)
    {
        std::cerr << "QueueData got null data object!" << std::endl;
        return;
    }

    NDArray *frame = static_cast<NDArray *>(dobj->data);

    // Set timestamp of frame.
    frame->epicsTS = evtTime;

    // Call callbacks.
    // int arrayCallbacks;
    // getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    // if (arrayCallbacks != 0)
    // {
    //     std::cout << "Calling image data callback" << std::endl;
    //     doCallbacksGenericPointer(frame, NDArrayData, 0);
    // }

    // The module is done with the frame.
    frame->release();
}

} // namespace adcraydl
