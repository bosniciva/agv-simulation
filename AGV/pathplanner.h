#pragma once
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include "agv.h"
#include "obstacle.h"
#include "markers.h"
using namespace std;

struct LaneDetourResult {
    vector<Point> points;
    vector<Point> markers;
    bool skippedCorner = false;
};

class PathPlanner {
public:
    PathPlanner(double cellSize = 0.5, double inflateRadius = 0.9, double escapeClearance = 0.3)
        : cellSize(cellSize), inflateRadius(inflateRadius), escapeClearance(escapeClearance) {
    }

    LaneDetourResult planLaneDetour(const Point& start, const Point& goal,
        const vector<Obstacle>& obstacles, const Point* nextGoal = nullptr,
        double laneOffset = 1.2) const {

        double dx = goal.x - start.x;
        double dy = goal.y - start.y;
        double len = sqrt(dx * dx + dy * dy);
        if (len < 1e-6) return {};

        double dirX = dx / len, dirY = dy / len;
        double perpX = -dirY, perpY = dirX;

        double nearestFromStart = 1e9;
        double nearestFromGoal = 1e9;
        for (const auto& obs : obstacles) {
            double sdx = obs.position.x - start.x, sdy = obs.position.y - start.y;
            double sd = sqrt(sdx * sdx + sdy * sdy);
            if (sd < nearestFromStart) nearestFromStart = sd;

            double gdx = obs.position.x - goal.x, gdy = obs.position.y - goal.y;
            double gd = sqrt(gdx * gdx + gdy * gdy);
            if (gd < nearestFromGoal) nearestFromGoal = gd;
        }
        double startClearance = escapeClearance;
        if (nearestFromStart < escapeClearance) {
            startClearance = max(0.05, nearestFromStart * 0.6);
        }
        double goalClearance = escapeClearance;
        if (nearestFromGoal < escapeClearance) {
            goalClearance = max(0.05, nearestFromGoal * 0.6);
        }

        for (int side = -1; side <= 1; side += 2) {
            Point laneA = { start.x + perpX * laneOffset * side, start.y + perpY * laneOffset * side };
            Point cornerLane = { goal.x + perpX * laneOffset * side, goal.y + perpY * laneOffset * side };

            bool alignsWithNext = false;
            if (nextGoal) {
                double dx2 = nextGoal->x - goal.x;
                double dy2 = nextGoal->y - goal.y;
                double dot = (perpX * side) * dx2 + (perpY * side) * dy2;
                alignsWithNext = dot > 0.1;
            }

            if (alignsWithNext) {
                if (hasLineOfSight(start, laneA, obstacles, startClearance) &&
                    hasLineOfSight(laneA, cornerLane, obstacles, inflateRadius) &&
                    hasLineOfSight(cornerLane, *nextGoal, obstacles, goalClearance)) {

                    LaneDetourResult result;
                    result.points = { laneA, cornerLane, *nextGoal };
                    result.markers = generateCeilingMarkers({ start, laneA, cornerLane, *nextGoal }, 3.0);
                    result.skippedCorner = true;
                    return result;
                }
            }
            else {
                if (hasLineOfSight(start, laneA, obstacles, startClearance) &&
                    hasLineOfSight(laneA, cornerLane, obstacles, inflateRadius) &&
                    hasLineOfSight(cornerLane, goal, obstacles, goalClearance)) {

                    LaneDetourResult result;
                    result.points = { laneA, cornerLane, goal };
                    result.markers = generateCeilingMarkers({ start, laneA, cornerLane, goal }, 3.0);
                    return result;
                }
            }
        }

        return {};
    }

    vector<Point> planDetour(const Point& start, const Point& goal,
        const vector<Obstacle>& obstacles) const {

        double margin = 3.0;
        double minX = min(start.x, goal.x) - margin;
        double maxX = max(start.x, goal.x) + margin;
        double minY = min(start.y, goal.y) - margin;
        double maxY = max(start.y, goal.y) + margin;

        double nearestFromStart = 1e9, nearestFromGoal = 1e9;
        for (const auto& obs : obstacles) {
            double sdx = obs.position.x - start.x, sdy = obs.position.y - start.y;
            nearestFromStart = min(nearestFromStart, sqrt(sdx * sdx + sdy * sdy));
            double gdx = obs.position.x - goal.x, gdy = obs.position.y - goal.y;
            nearestFromGoal = min(nearestFromGoal, sqrt(gdx * gdx + gdy * gdy));
        }
        double closestEndpointDist = min(nearestFromStart, nearestFromGoal);
        double effectiveInflateRadius = inflateRadius;
        if (closestEndpointDist < inflateRadius) {
            effectiveInflateRadius = max(0.1, closestEndpointDist * 0.7);
        }

        auto toCell = [&](const Point& p) -> pair<int, int> {
            int cx = static_cast<int>(round((p.x - minX) / cellSize));
            int cy = static_cast<int>(round((p.y - minY) / cellSize));
            return { cx, cy };
            };
        auto toPoint = [&](int cx, int cy) -> Point {
            return Point{ minX + cx * cellSize, minY + cy * cellSize };
            };
        auto cellKey = [](int cx, int cy) -> long long {
            return (static_cast<long long>(cx) + 100000) * 1000000LL + (static_cast<long long>(cy) + 100000);
            };
        auto isOccupied = [&](int cx, int cy) -> bool {
            Point p = toPoint(cx, cy);
            for (const auto& obs : obstacles) {
                double dx = p.x - obs.position.x;
                double dy = p.y - obs.position.y;
                if (sqrt(dx * dx + dy * dy) <= effectiveInflateRadius) return true;
            }
            return false;
            };

        auto [startCx, startCy] = toCell(start);
        auto [goalCx, goalCy] = toCell(goal);

        auto heuristic = [&](int cx, int cy) -> double {
            double dx = (cx - goalCx) * cellSize;
            double dy = (cy - goalCy) * cellSize;
            return sqrt(dx * dx + dy * dy);
            };

        struct Node { int cx, cy; double f; };
        struct Compare { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };

        priority_queue<Node, vector<Node>, Compare> open;
        unordered_map<long long, double> gScore;
        unordered_map<long long, pair<int, int>> cameFrom;
        unordered_set<long long> closed;

        long long startKey = cellKey(startCx, startCy);
        gScore[startKey] = 0.0;
        open.push({ startCx, startCy, heuristic(startCx, startCy) });

        const int dxs[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
        const int dys[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

        bool found = false;
        int gridSpan = static_cast<int>((maxX - minX) / cellSize) + static_cast<int>((maxY - minY) / cellSize) + 10;
        int maxIterations = gridSpan * gridSpan;
        int iterations = 0;

        while (!open.empty() && iterations++ < maxIterations) {
            Node current = open.top();
            open.pop();
            long long curKey = cellKey(current.cx, current.cy);
            if (closed.count(curKey)) continue;
            closed.insert(curKey);

            if (current.cx == goalCx && current.cy == goalCy) {
                found = true;
                break;
            }

            for (int i = 0; i < 8; i++) {
                int ncx = current.cx + dxs[i];
                int ncy = current.cy + dys[i];
                if (isOccupied(ncx, ncy)) continue;

                double stepCost = (dxs[i] != 0 && dys[i] != 0) ? cellSize * 1.41421356 : cellSize;
                double tentativeG = gScore[curKey] + stepCost;

                long long nKey = cellKey(ncx, ncy);
                auto existing = gScore.find(nKey);
                if (existing == gScore.end() || tentativeG < existing->second) {
                    gScore[nKey] = tentativeG;
                    cameFrom[nKey] = { current.cx, current.cy };
                    open.push({ ncx, ncy, tentativeG + heuristic(ncx, ncy) });
                }
            }
        }

        if (!found) return {};

        vector<Point> fullPath;
        int cx = goalCx, cy = goalCy;
        fullPath.push_back(toPoint(cx, cy));
        while (!(cx == startCx && cy == startCy)) {
            long long key = cellKey(cx, cy);
            auto it = cameFrom.find(key);
            if (it == cameFrom.end()) break;
            cx = it->second.first;
            cy = it->second.second;
            fullPath.push_back(toPoint(cx, cy));
        }
        reverse(fullPath.begin(), fullPath.end());

        vector<Point> smoothed = smoothPath(fullPath, obstacles, effectiveInflateRadius);
        if (!smoothed.empty()) {
            smoothed.back() = goal;
        }
        return smoothed;
    }

private:
    double cellSize;
    double inflateRadius;
    double escapeClearance;

    bool hasLineOfSight(const Point& a, const Point& b, const vector<Obstacle>& obstacles, double clearance) const {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double lenSq = dx * dx + dy * dy;

        for (const auto& obs : obstacles) {
            double t = 0.0;
            if (lenSq > 1e-9) {
                t = ((obs.position.x - a.x) * dx + (obs.position.y - a.y) * dy) / lenSq;
                t = max(0.0, min(1.0, t));
            }
            double projX = a.x + t * dx;
            double projY = a.y + t * dy;
            double ddx = obs.position.x - projX;
            double ddy = obs.position.y - projY;
            if (sqrt(ddx * ddx + ddy * ddy) <= clearance) {
                return false;
            }
        }
        return true;
    }

    vector<Point> smoothPath(const vector<Point>& path, const vector<Obstacle>& obstacles, double clearance) const {
        vector<Point> result;
        if (path.size() <= 1) return result;

        size_t currentIndex = 0;
        while (currentIndex < path.size() - 1) {
            size_t farthest = currentIndex + 1;
            for (size_t j = path.size() - 1; j > currentIndex + 1; j--) {
                if (hasLineOfSight(path[currentIndex], path[j], obstacles, clearance)) {
                    farthest = j;
                    break;
                }
            }
            result.push_back(path[farthest]);
            currentIndex = farthest;
        }
        return result;
    }
};