#include "ADCrayDlStorage.h"

#include <mutex>

namespace adcraydl
{

ThreadedQueue<NDArray *> Storage::frameQueue;

} // namespace adcraydl
