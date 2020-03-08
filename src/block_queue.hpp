/* From https://gist.github.com/PolarNick239/f727c0cd923398dc397a05f515452123 */

#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

// Based on https://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html
template<typename Data>
class BlockingQueue {
private:
    std::queue<Data>            queue;
    mutable boost::mutex        queue_mutex;
    size_t                      queue_limit;

    bool                        is_closed = false;

    boost::condition_variable   new_item_or_closed_event;
    boost::condition_variable   item_removed_event;

#ifndef NDEBUG
    size_t                      pushes_in_progress = 0;
#endif

public:
    BlockingQueue(size_t size_limit=0) : queue_limit(size_limit)
    {}

    void set_queue_limit(size_t size_limit)
    {
        queue_limit = size_limit;
    }

    void push(const Data& data)
    {
        boost::mutex::scoped_lock lock(queue_mutex);
#ifndef NDEBUG
        ++pushes_in_progress;
#endif
        if (queue_limit > 0) {
            while (queue.size() >= queue_limit) {
                item_removed_event.wait(lock);
            }
        }
        assert (!is_closed);
        queue.push(data);
#ifndef NDEBUG
        --pushes_in_progress;
#endif
        lock.unlock();

        new_item_or_closed_event.notify_one();
    }

    bool try_push(const Data& data)
    {
        boost::mutex::scoped_lock lock(queue_mutex);
        if (queue_limit > 0) {
            if (queue.size() >= queue_limit) {
                return false;
            }
        }
        assert (!is_closed);
        queue.push(data);
        lock.unlock();

        new_item_or_closed_event.notify_one();
        return true;
    }

    void close()
    {
        boost::mutex::scoped_lock lock(queue_mutex);
        assert (!is_closed);
#ifndef NDEBUG
        assert (pushes_in_progress == 0);
#endif
        is_closed = true;
        lock.unlock();

        new_item_or_closed_event.notify_all();
    }

    bool pop(Data &popped_value)
    {
        boost::mutex::scoped_lock lock(queue_mutex);
        while (queue.empty()) {
            if (is_closed) {
                return false;
            }
            new_item_or_closed_event.wait(lock);
        }

        popped_value = queue.front();
        queue.pop();
        item_removed_event.notify_one();
        return true;
    }

    bool try_pop(Data &popped_value)
    {
        boost::mutex::scoped_lock lock(queue_mutex);
        if (queue.empty()) {
            return false;
        }

        popped_value = queue.front();
        queue.pop();
        item_removed_event.notify_one();
        return true;
    }

    bool empty() const
    {
        boost::mutex::scoped_lock lock(queue_mutex);
        return queue.empty();
    }

    bool closed() const
    {
        boost::mutex::scoped_lock lock(queue_mutex);
        return is_closed;
    }

    size_t limit() const
    {
        return queue_limit;
    }

    size_t size() const
    {
        boost::mutex::scoped_lock lock(queue_mutex);
        return queue.size();
    }

};