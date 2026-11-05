#include "agv.h"
#include <cmath>
#include <sstream>
#include <iomanip>
using namespace std;

namespace {
    constexpr double PI = 3.14159265358979323846;
}

AGV::AGV(double startX, double startY)
    : posX(startX), posY(startY), headingDeg(0.0), currentSpeed(0.0),
    currentState(AGVState::IDLE), waypointIndex(0) {
}

void AGV::setRoute(const vector<Point>& waypoints) {
    route = waypoints;
    waypointIndex = 0;
}

void AGV::startMission() {
    if (!route.empty()) {
        currentState = AGVState::NAVIGATING;
    }
}

void AGV::notifyObstacleZone(const string& zone) {
    if (currentState == AGVState::MISSION_COMPLETE || currentState == AGVState::IDLE) {
        return;
    }

    if (zone == "red") {
        currentState = AGVState::OBSTACLE_STOP;
    }
    else if (zone == "yellow") {
        currentState = AGVState::OBSTACLE_SLOW;
    }
    else if (zone == "clear") {
        if (currentState == AGVState::OBSTACLE_STOP || currentState == AGVState::OBSTACLE_SLOW) {
            currentState = AGVState::NAVIGATING;
        }
    }
}

bool AGV::reachedWaypoint(const Point& target, double threshold) const {
    double dx = target.x - posX;
    double dy = target.y - posY;
    return sqrt(dx * dx + dy * dy) < threshold;
}

void AGV::moveTowardTarget(double dt) {
    if (waypointIndex >= route.size()) return;

    const Point& target = route[waypointIndex];
    double dx = target.x - posX;
    double dy = target.y - posY;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance < 0.05) {
        waypointIndex++;
        if (waypointIndex >= route.size()) {
            currentState = AGVState::MISSION_COMPLETE;
            currentSpeed = 0.0;
        }
        return;
    }

    headingDeg = atan2(dy, dx) * 180.0 / PI;

    double targetSpeed = maxSpeed;
    if (currentState == AGVState::OBSTACLE_SLOW) {
        targetSpeed = maxSpeed * 0.3;
    }

    if (currentSpeed < targetSpeed) {
        currentSpeed += acceleration * dt;
        if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
    }
    else if (currentSpeed > targetSpeed) {
        currentSpeed = targetSpeed;
    }

    double step = currentSpeed * dt;
    if (step > distance) step = distance;

    posX += (dx / distance) * step;
    posY += (dy / distance) * step;
}

void AGV::update(double dt) {
    switch (currentState) {
    case AGVState::NAVIGATING:
    case AGVState::OBSTACLE_SLOW:
        moveTowardTarget(dt);
        break;
    case AGVState::OBSTACLE_STOP:
        currentSpeed = 0.0;
        break;
    default:
        break;
    }
}

string AGV::stateToString() const {
    switch (currentState) {
    case AGVState::IDLE: return "IDLE";
    case AGVState::LEARNING: return "LEARNING";
    case AGVState::NAVIGATING: return "NAVIGATING";
    case AGVState::OBSTACLE_SLOW: return "OBSTACLE_SLOW";
    case AGVState::OBSTACLE_STOP: return "OBSTACLE_STOP";
    case AGVState::MISSION_COMPLETE: return "MISSION_COMPLETE";
    }
    return "UNKNOWN";
}

Point AGV::currentTarget() const {
    if (waypointIndex < route.size()) {
        return route[waypointIndex];
    }
    return Point{ posX, posY };
}

Point AGV::routePointAt(size_t index) const {
    if (index < route.size()) {
        return route[index];
    }
    return Point{ posX, posY };
}

vector<Point> AGV::upcomingWaypoints(int count) const {
    vector<Point> result;
    for (size_t i = waypointIndex; i < route.size() && static_cast<int>(result.size()) < count; i++) {
        result.push_back(route[i]);
    }
    return result;
}

void AGV::insertDetour(const vector<Point>& detourPoints) {
    if (detourPoints.empty()) return;
    route.insert(route.begin() + waypointIndex, detourPoints.begin(), detourPoints.end());
}

void AGV::replaceUpcoming(size_t count, const vector<Point>& newPoints) {
    size_t removeEnd = min(waypointIndex + count, route.size());
    route.erase(route.begin() + waypointIndex, route.begin() + removeEnd);
    route.insert(route.begin() + waypointIndex, newPoints.begin(), newPoints.end());
}

void AGV::resumeNavigation() {
    if (currentState == AGVState::OBSTACLE_STOP) {
        currentState = AGVState::NAVIGATING;
    }
}

void AGV::goIdle() {
    if (currentState == AGVState::MISSION_COMPLETE) {
        currentState = AGVState::IDLE;
    }
}

string AGV::toStatusLine() const {
    ostringstream oss;
    oss << fixed << setprecision(2);
    oss << "[" << stateToString() << "] "
        << "pos=(" << posX << ", " << posY << ") "
        << "heading=" << headingDeg << " deg "
        << "speed=" << currentSpeed << " m/s "
        << "waypoint=" << waypointIndex << "/" << route.size();
    return oss.str();
}