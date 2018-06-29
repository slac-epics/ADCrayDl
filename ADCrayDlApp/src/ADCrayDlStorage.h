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

    /**
     * @brief Get the data access mutex.
     * 
     * @return Referene to data access mutex
     */
    static epicsMutex &getMutex();

private:
    static epicsMutex m_mutex; //!< Data access mutex.
    static ThreadedQueue<NDArray *> m_frameQueue; //!< Queue contining un-timestamped acquired frames.
};

} // namespace adcraydl

#endif // ADCRAYDLSTORAGE_HXX
