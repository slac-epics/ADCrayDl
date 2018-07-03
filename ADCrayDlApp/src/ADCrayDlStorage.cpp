#include "ADCrayDlStorage.h"

#include <mutex>

namespace adcraydl
{

ThreadedQueue<NDArray *> Storage::frameQueue;
ThreadedQueue<NDArray *> Storage::timestampedFrameQueue;

} // namespace adcraydl
