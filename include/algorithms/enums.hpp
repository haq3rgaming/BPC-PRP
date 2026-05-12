#pragma once

namespace nodes {
    enum FSMState {
        CALIBRATION = 0,
        CORRIDOR,
        INTERSECTION,
        TURN,
        NO_FRONT_WALL_INTERSECTION,
        GO_TO,
        STOP
    };
    inline constexpr const char* FSMStateNames[] = {
        "CALIBRATION",
        "CORRIDOR",
        "INTERSECTION",
        "TURN",
        "NO_FRONT_WALL_INTERSECTION",
        "GO_TO",
        "STOP"
    };

    enum FSMNextIntersection {
        NONE = 0,
        FW,
        LEFT,
        RIGHT
    };
    inline constexpr const char* FSMNextIntersectionNames[] = {
        "NONE",
        "FW",
        "LEFT",
        "RIGHT"
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