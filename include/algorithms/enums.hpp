#pragma once

namespace nodes {
    enum FSMState {
        START = 0,
        MOVE_FORWARD,
        MOVE_BACKWARD,
        TURN_LEFT,
        TURN_RIGHT,
        STOP,
        COMPUTE_NEXT_COMMAND
    };

    enum FSMNextIntersection {
        NONE = 0,
        FW,
        LEFT,
        RIGHT
    };

    enum ArucoMarkerID {
        EXIT_FW = 0,
        EXIT_LEFT = 1,
        EXIT_RIGHT = 2,
        TREASURE_FW = 10,
        TREASURE_LEFT = 11,
        TREASURE_RIGHT = 12
    };
}