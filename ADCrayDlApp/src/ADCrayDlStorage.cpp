#include "ADCrayDlStorage.h"

#include <mutex>

namespace adcraydl
{

epicsMutex Storage::m_mutex;
ThreadedQueue<NDArray *> Storage::m_frameQueue;

epicsMutex &Storage::getMutex()
{
    return m_mutex;
}

} // namespace adcraydl
