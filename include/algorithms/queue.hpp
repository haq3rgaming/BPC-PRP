#pragma once

#include <vector>
#include <stdexcept>
#include "algorithms/enums.hpp"

namespace nodes {
    class Queue {
    public:
        Queue();
        ~Queue() = default;
        void push(FSMNextIntersection intersection, bool ignore_duplicates = true);
        FSMNextIntersection pop();
        FSMNextIntersection peek();
        bool is_empty();

    private:
        std::vector<FSMNextIntersection> queue_ {};
    };
}