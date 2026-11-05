#include "Sensor.h"
#include <cmath>
using namespace std;

namespace {
    constexpr double PI = 3.14159265358979323846;
}

Sensor::Sensor(double yellowRadius_, double redRadius_, double detectionHalfAngleDeg_, double shortRangeRadius_)
    : yellowRadius(yellowRadius_), redRadius(redRadius_), detectionHalfAngleDeg(detectionHalfAngleDeg_),
    shortRangeRadius(shortRangeRadius_) {
}

double Sensor::normalizeAngle(double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

string Sensor::evaluateZone(double agvX, double agvY, double agvHeadingDeg,
    const vector<Obstacle>& obstacles, bool omnidirectional) const {

    bool yellowDetected = false;

    for (const auto& obs : obstacles) {
        double dx = obs.position.x - agvX;
        double dy = obs.position.y - agvY;
        double distance = sqrt(dx * dx + dy * dy);

        bool isOtherAgv = !obs.sourceAgvId.empty();

        if (isOtherAgv) {
            double agvPersonalSpace = redRadius + 0.6;
            if (distance <= agvPersonalSpace) {
                return "red";
            }
            if (distance <= yellowRadius) {
                yellowDetected = true;
            }
            continue;
        }

        if (omnidirectional && distance <= shortRangeRadius) {
            return "red";
        }

        double angleToObstacle = atan2(dy, dx) * 180.0 / PI;
        double angleDiff = normalizeAngle(angleToObstacle - agvHeadingDeg);
        if (abs(angleDiff) > detectionHalfAngleDeg) {
            continue;
        }

        if (distance <= redRadius) {
            return "red";
        }
        if (distance <= yellowRadius) {
            yellowDetected = true;
        }
    }

    return yellowDetected ? "yellow" : "clear";
}