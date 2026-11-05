#pragma once
#include "agv.h"
#include <string>
using namespace std;

enum class ObstacleType { STATIC, PERSON };

struct Obstacle {
    Point position;
    ObstacleType type = ObstacleType::STATIC;
    Point velocity = { 0.0, 0.0 };
    double distanceTraveled = 0.0;
    double crossDistance = 2.5;
    bool started = false;
    string sourceAgvId = "";
};

inline string obstacleTypeToString(ObstacleType t) {
    return t == ObstacleType::PERSON ? "person" : "static";
}