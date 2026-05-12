#include <algorithms/queue.hpp>

namespace nodes {
    Queue::Queue() = default;

    bool Queue::push(FSMNextIntersection intersection, bool ignore_duplicates) {
        if (ignore_duplicates && !queue_.empty() && queue_.back() == intersection) return false;
        queue_.push_back(intersection);
        return true;
    }

    FSMNextIntersection Queue::pop() {
        if (queue_.empty()) {
            throw std::out_of_range("Queue is empty");
        }
        FSMNextIntersection front = queue_.front();
        queue_.erase(queue_.begin());
        return front;
    }

    FSMNextIntersection Queue::peek() {
        if (queue_.empty()) {
            throw std::out_of_range("Queue is empty");
        }
        return queue_.front();
    }

    bool Queue::is_empty() {
        return queue_.empty();
    }
}