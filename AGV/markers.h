#pragma once
#include <vector>
#include <cmath>
#include "agv.h"
using namespace std;

inline vector<Point> generateCeilingMarkers(const vector<Point>& path, double spacingMeters = 3.0) {
    vector<Point> markers;
    if (path.empty()) return markers;

    markers.push_back(path[0]);
    double carry = 0.0;

    for (size_t i = 0; i + 1 < path.size(); i++) {
        const Point& a = path[i];
        const Point& b = path[i + 1];
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double segLen = sqrt(dx * dx + dy * dy);
        if (segLen < 1e-9) continue;

        double dirX = dx / segLen;
        double dirY = dy / segLen;

        double dist = spacingMeters - carry;
        while (dist < segLen) {
            markers.push_back({ a.x + dirX * dist, a.y + dirY * dist });
            dist += spacingMeters;
        }
        carry = segLen - (dist - spacingMeters);
    }
    return markers;
}

inline int nearestMarkerIndex(const vector<Point>& markers, double x, double y) {
    int best = -1;
    double bestDistSq = 1e18;
    for (size_t i = 0; i < markers.size(); i++) {
        double dx = markers[i].x - x;
        double dy = markers[i].y - y;
        double d = dx * dx + dy * dy;
        if (d < bestDistSq) {
            bestDistSq = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

inline vector<Point> generateSideLaneMarkers(const vector<Point>& path,
    double laneOffset = 1.2, double spacingMeters = 3.0) {
    vector<Point> markers;

    for (size_t i = 0; i + 1 < path.size(); i++) {
        const Point& a = path[i];
        const Point& b = path[i + 1];
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double len = sqrt(dx * dx + dy * dy);
        if (len < 1e-6) continue;

        double dirX = dx / len, dirY = dy / len;
        double perpX = -dirY, perpY = dirX;

        for (int side = -1; side <= 1; side += 2) {
            Point laneA = { a.x + perpX * laneOffset * side, a.y + perpY * laneOffset * side };
            Point laneB = { b.x + perpX * laneOffset * side, b.y + perpY * laneOffset * side };
            vector<Point> segMarkers = generateCeilingMarkers({ laneA, laneB }, spacingMeters);
            markers.insert(markers.end(), segMarkers.begin(), segMarkers.end());
        }
    }

    return markers;
}