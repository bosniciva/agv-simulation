#pragma once
#include <vector>
#include <string>
#include "agv.h"
#include "obstacle.h"
using namespace std;

class Sensor {
public:
    Sensor(double yellowRadius = 1.5, double redRadius = 0.6, double detectionHalfAngleDeg = 90.0,
        double shortRangeRadius = 0.35);

    string evaluateZone(double agvX, double agvY, double agvHeadingDeg,
        const vector<Obstacle>& obstacles, bool omnidirectional = false) const;

    void setYellowRadius(double r) { yellowRadius = r; }
    void setRedRadius(double r) { redRadius = r; }
    void setDetectionHalfAngle(double deg) { detectionHalfAngleDeg = deg; }
    void setShortRangeRadius(double r) { shortRangeRadius = r; }

    double getYellowRadius() const { return yellowRadius; }
    double getRedRadius() const { return redRadius; }
    double getDetectionHalfAngle() const { return detectionHalfAngleDeg; }
    double getShortRangeRadius() const { return shortRangeRadius; }

private:
    double yellowRadius;
    double redRadius;
    double detectionHalfAngleDeg;
    double shortRangeRadius;

    static double normalizeAngle(double deg);
};