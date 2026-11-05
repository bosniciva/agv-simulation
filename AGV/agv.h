#pragma once
#include <string>
#include <vector>
using namespace std;

enum class AGVState {
    IDLE,
    LEARNING,
    NAVIGATING,
    OBSTACLE_SLOW,
    OBSTACLE_STOP,
    MISSION_COMPLETE
};

struct Point {
    double x;
    double y;
};

class AGV {
public:
    AGV(double startX, double startY);

    void setRoute(const vector<Point>& waypoints);
    void startMission();
    void update(double dt);

    void notifyObstacleZone(const string& zone);

    Point currentTarget() const;
    Point routePointAt(size_t index) const;
    vector<Point> upcomingWaypoints(int count) const;
    void insertDetour(const vector<Point>& detourPoints);
    void replaceUpcoming(size_t count, const vector<Point>& newPoints);
    void resumeNavigation();
    void goIdle();

    double x() const { return posX; }
    double y() const { return posY; }
    double heading() const { return headingDeg; }
    double speed() const { return currentSpeed; }
    AGVState state() const { return currentState; }
    size_t currentWaypointIndex() const { return waypointIndex; }
    size_t routeSize() const { return route.size(); }

    string stateToString() const;
    string toStatusLine() const;

private:
    double posX, posY;
    double headingDeg;
    double currentSpeed;

    static constexpr double maxSpeed = 1.0;
    static constexpr double acceleration = 0.5;

    AGVState currentState;
    vector<Point> route;
    size_t waypointIndex;

    void moveTowardTarget(double dt);
    bool reachedWaypoint(const Point& target, double threshold = 0.05) const;
};