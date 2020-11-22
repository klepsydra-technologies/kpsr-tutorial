#ifndef POSITION_H
#define POSITION_H

#include <numeric>

class Position {
public:
    float x;
    float y;
    Position(float _x, float _y) {
        x = _x;
        y = _y;
    }
    Position() {
        x = 0;
        y = 0;
    }
};
#endif // POSITION_H