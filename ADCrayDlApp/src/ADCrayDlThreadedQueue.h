#ifndef ADCRAYDL_THREADEDQUEUE_HXX
#define ADCRAYDL_THREADEDQUEUE_HXX

#include <queue>
#include <deque>
#include <mutex>
#include <condition_variable>

namespace adcraydl
{

/**
 * @brief Blocking thread-safe implementation of a queue.
 * 
 * @tparam T             Type of data stored in the queue.
 * @tparam std::deque<T> The type of the underlying container to use to store the elements.
 * @see std::queue
 */
template<class T, class Container = std::deque<T>>
class ThreadedQueue : private std::queue<T, Container>
{

public:
    /**
     * @brief Construct a new Threaded Queue object.
     * 
     */
    ThreadedQueue() = default;

    /**
     * @brief Push data to the queue.
     * 
     * @param data Data that should be pushed to the queue.
     */
    void push(const T &data)
    {
        std::unique_lock<std::mutex> lock(m_mutex);

        std::queue<T, Container>::push(data);
        
        lock.unlock();
        m_condVariable.notify_one();
    }

    /**
     * @brief Check if queue is empty.
     * 
     * @return True when the queue is empty, false when not.
     */
    bool empty() const
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return std::queue<T, Container>::empty();
    }

    /**
     * @brief Pop the first element from the queue.
     * 
     * @param[out] poppedValue Reference where the element is popped to.
     * @note If there are no elements in the queue, this method blocks until an element is added to the queue.
     */
    void blockingPop(T &poppedValue)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        
        while (std::queue<T, Container>::empty())
        {
            m_condVariable.wait(lock);
        }
        
        poppedValue = std::queue<T, Container>::front();
        std::queue<T, Container>::pop();
    }

private:
    std::mutex m_mutex; //!< Mutex used to access the queue concurrently.
    std::condition_variable m_condVariable; //!< Conditional variable used to block thread while waiting for elements.

};

} // namespace adcraydl

#endif // ADCRAYDL_THREADEDQUEUE_HXX