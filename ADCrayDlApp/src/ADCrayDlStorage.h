#ifndef ADCRAYDLSTORAGE_HXX
#define ADCRAYDLSTORAGE_HXX

#include "craydl/RxFrame.h"

#include <epicsMutex.h>
#include <NDArray.h>

#include "ADCrayDlThreadedQueue.h"

namespace adcraydl
{

/**
 * @brief Static storage class.
 */
class Storage
{
public:
    /**
     * @brief Construct a new Storage object
     */
    Storage() = default;

    /**
     * @brief Destroy the Storage object
     */
    virtual ~Storage() = default;

    static ThreadedQueue<NDArray *> frameQueue; //!< Queue contining un-timestamped acquired frames.
    static ThreadedQueue<NDArray *> timestampedFrameQueue; //!< Queue containing timestamped frames.
};

} // namespace adcraydl

#endif // ADCRAYDLSTORAGE_HXX
