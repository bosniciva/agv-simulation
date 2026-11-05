#pragma once
#include "AGV.h"
#include "Sensor.h"
#include "Mission.h"
#include "PathPlanner.h"
#include "Obstacle.h"
#include <vector>
#include <string>
#include <cmath>
#include <random>
#include <iostream>
#include <iomanip>
using namespace std;

struct BenchmarkOutcome {
    bool completed;
    int ticks;
};

inline BenchmarkOutcome simulateMission(const vector<MissionStep>& steps,
    const string& missionName, vector<Obstacle> obstacles,
    bool plannerEnabled, int maxTicks = 3000, double forceObstacleClearAfterSeconds = -1.0) {

    AGV agv(0.0, 0.0);
    Mission mission(missionName, steps);
    Sensor sensor;
    PathPlanner planner;
    mission.begin(agv);

    const double dt = 0.5;
    int tickCount = 0;
    double stopElapsed = 0.0;
    bool detourAttempted = false;
    const double REPLAN_THRESHOLD = 3.0;
    double obstacleTimer = forceObstacleClearAfterSeconds;
    bool obstacleTimerStarted = false;

    while (!mission.isComplete() && tickCount < maxTicks) {
        mission.update(dt, agv);

        if (forceObstacleClearAfterSeconds > 0.0 && !obstacles.empty()) {
            if (!obstacleTimerStarted && agv.state() == AGVState::OBSTACLE_STOP) {
                obstacleTimerStarted = true;
            }
            if (obstacleTimerStarted) {
                obstacleTimer -= dt;
                if (obstacleTimer <= 0.0) {
                    obstacles.clear();
                }
            }
        }

        for (auto it = obstacles.begin(); it != obstacles.end(); ) {
            if (it->type == ObstacleType::PERSON && it->started) {
                double sx = it->velocity.x * dt;
                double sy = it->velocity.y * dt;
                it->position.x += sx;
                it->position.y += sy;
                it->distanceTraveled += sqrt(sx * sx + sy * sy);
                if (it->distanceTraveled >= it->crossDistance) {
                    it = obstacles.erase(it);
                    continue;
                }
            }
            ++it;
        }

        string zone = sensor.evaluateZone(agv.x(), agv.y(), agv.heading(), obstacles);
        agv.notifyObstacleZone(zone);

        if (zone == "red") {
            for (auto& o : obstacles) {
                if (o.type == ObstacleType::PERSON && !o.started) {
                    double dx = o.position.x - agv.x();
                    double dy = o.position.y - agv.y();
                    if (sqrt(dx * dx + dy * dy) <= sensor.getRedRadius() + 0.05) {
                        o.started = true;
                        constexpr double PI = 3.14159265358979323846;
                        constexpr double WALK_SPEED = 0.4;
                        double perpAngleRad = (agv.heading() - 90.0) * PI / 180.0;
                        o.velocity = { WALK_SPEED * cos(perpAngleRad), WALK_SPEED * sin(perpAngleRad) };
                    }
                }
            }
        }

        if (plannerEnabled && agv.state() == AGVState::OBSTACLE_STOP) {
            stopElapsed += dt;
            if (stopElapsed > REPLAN_THRESHOLD && !detourAttempted) {
                detourAttempted = true;
                Point start = { agv.x(), agv.y() };
                Point goal = agv.currentTarget();

                auto laneResult = planner.planLaneDetour(start, goal, obstacles);
                vector<Point> detour = !laneResult.points.empty() ? laneResult.points
                    : planner.planDetour(start, goal, obstacles);

                if (!detour.empty()) {
                    agv.replaceUpcoming(1, detour);
                    agv.resumeNavigation();
                }
            }
        }
        else if (agv.state() != AGVState::OBSTACLE_STOP) {
            stopElapsed = 0.0;
            detourAttempted = false;
        }

        agv.update(dt);
        tickCount++;
    }

    return { mission.isComplete(), tickCount };
}

struct BenchmarkStats {
    int totalTrials = 0;
    int completedWithout = 0;
    int completedWith = 0;
    double avgTicksWithout = 0.0;
    double avgTicksWith = 0.0;
    double avgSpeedupPct = 0.0;
    int onlyCompletedWithPlanner = 0;
};

inline BenchmarkStats runRandomizedBenchmark(
    const vector<MissionStep>& redSteps, const vector<MissionStep>& greenSteps,
    int trials = 100) {

    mt19937 rng(random_device{}());
    uniform_real_distribution<double> xDistRed(6.0, 9.0);
    uniform_real_distribution<double> xDistGreen(4.0, 7.0);
    uniform_real_distribution<double> yDist(-0.3, 0.3);
    uniform_int_distribution<int> routeDist(0, 1);
    uniform_int_distribution<int> typeDist(0, 1);

    BenchmarkStats stats;
    stats.totalTrials = trials;

    long long sumTicksWithout = 0, sumTicksWith = 0;
    double sumSpeedup = 0.0;
    int comparableCount = 0;

    for (int i = 0; i < trials; i++) {
        bool isGreen = routeDist(rng) == 1;
        const auto& steps = isGreen ? greenSteps : redSteps;
        string missionName = isGreen ? "bench_green" : "bench_red";

        Obstacle obs;
        double ox = isGreen ? xDistGreen(rng) : xDistRed(rng);
        obs.position = { ox, yDist(rng) };
        bool isPerson = typeDist(rng) == 1;
        obs.type = isPerson ? ObstacleType::PERSON : ObstacleType::STATIC;
        if (isPerson) obs.crossDistance = 2.5;

        vector<Obstacle> obstacles = { obs };

        BenchmarkOutcome without = simulateMission(steps, missionName, obstacles, false);
        BenchmarkOutcome with = simulateMission(steps, missionName, obstacles, true);

        if (without.completed) { stats.completedWithout++; sumTicksWithout += without.ticks; }
        if (with.completed) { stats.completedWith++; sumTicksWith += with.ticks; }

        if (!without.completed && with.completed) {
            stats.onlyCompletedWithPlanner++;
        }
        else if (without.completed && with.completed && without.ticks > 0) {
            sumSpeedup += 100.0 * (without.ticks - with.ticks) / static_cast<double>(without.ticks);
            comparableCount++;
        }
    }

    if (stats.completedWithout > 0) stats.avgTicksWithout = static_cast<double>(sumTicksWithout) / stats.completedWithout;
    if (stats.completedWith > 0) stats.avgTicksWith = static_cast<double>(sumTicksWith) / stats.completedWith;
    if (comparableCount > 0) stats.avgSpeedupPct = sumSpeedup / comparableCount;

    return stats;
}

inline void printRandomizedBenchmarkReport(const BenchmarkStats& s) {
    cout << "\n=== BENCHMARK (random, " << s.totalTrials << " trials) ===" << endl;
    cout << "Completed WITHOUT obstacle avoidance: " << s.completedWithout << " / " << s.totalTrials
        << " (" << fixed << setprecision(1)
        << (100.0 * s.completedWithout / s.totalTrials) << "%)" << endl;
    cout << "Completed WITH obstacle avoidance: " << s.completedWith << " / " << s.totalTrials
        << " (" << (100.0 * s.completedWith / s.totalTrials) << "%)" << endl;
    cout << "Average ticks WITHOUT (completed only): " << setprecision(1) << s.avgTicksWithout << endl;
    cout << "Average ticks WITH (completed only): " << s.avgTicksWith << endl;
    cout << "Average speedup when both complete: " << s.avgSpeedupPct << "%" << endl;
    cout << "Cases completed ONLY with obstacle avoidance: " << s.onlyCompletedWithPlanner
        << " / " << s.totalTrials << " ("
        << (100.0 * s.onlyCompletedWithPlanner / s.totalTrials) << "%)" << endl;
    cout << "=========================================================\n" << endl;
}

struct TimedBenchmarkStats {
    int totalTrials = 0;
    double avgTicksWithout = 0.0;
    double avgTicksWith = 0.0;
    double avgSpeedupPct = 0.0;
    double avgSpeedupSeconds = 0.0;
    int meaningfulCases = 0;
    double avgSpeedupSecondsWhenHelped = 0.0;
};

inline TimedBenchmarkStats runTimedObstacleBenchmark(
    const vector<MissionStep>& redSteps, const vector<MissionStep>& greenSteps,
    int trials = 100) {

    mt19937 rng(random_device{}());
    uniform_real_distribution<double> xDistRed(6.0, 9.0);
    uniform_real_distribution<double> xDistGreen(4.0, 7.0);
    uniform_real_distribution<double> yDist(-0.3, 0.3);
    uniform_int_distribution<int> routeDist(0, 1);
    uniform_real_distribution<double> clearTimeDist(3.0, 25.0);

    TimedBenchmarkStats stats;
    stats.totalTrials = trials;

    long long sumTicksWithout = 0, sumTicksWith = 0;
    double sumSpeedupPct = 0.0;
    int comparableCount = 0;

    int meaningfulCount = 0;
    double sumMeaningfulSavingsSec = 0.0;

    for (int i = 0; i < trials; i++) {
        bool isGreen = routeDist(rng) == 1;
        const auto& steps = isGreen ? greenSteps : redSteps;
        string missionName = isGreen ? "timed_green" : "timed_red";

        Obstacle obs;
        double ox = isGreen ? xDistGreen(rng) : xDistRed(rng);
        obs.position = { ox, yDist(rng) };
        obs.type = ObstacleType::STATIC;

        double clearAfter = clearTimeDist(rng);
        vector<Obstacle> obstacles = { obs };

        BenchmarkOutcome without = simulateMission(steps, missionName, obstacles, false, 3000, clearAfter);
        BenchmarkOutcome with = simulateMission(steps, missionName, obstacles, true, 3000, clearAfter);

        sumTicksWithout += without.ticks;
        sumTicksWith += with.ticks;
        if (without.ticks > 0) {
            sumSpeedupPct += 100.0 * (without.ticks - with.ticks) / static_cast<double>(without.ticks);
            comparableCount++;
        }

        int tickDiff = without.ticks - with.ticks;
        if (tickDiff > 1) {
            meaningfulCount++;
            sumMeaningfulSavingsSec += tickDiff * 0.5;
        }
    }

    stats.avgTicksWithout = static_cast<double>(sumTicksWithout) / trials;
    stats.avgTicksWith = static_cast<double>(sumTicksWith) / trials;
    if (comparableCount > 0) stats.avgSpeedupPct = sumSpeedupPct / comparableCount;
    stats.avgSpeedupSeconds = (stats.avgTicksWithout - stats.avgTicksWith) * 0.5;
    stats.meaningfulCases = meaningfulCount;
    if (meaningfulCount > 0) stats.avgSpeedupSecondsWhenHelped = sumMeaningfulSavingsSec / meaningfulCount;

    return stats;
}

inline void printTimedBenchmarkReport(const TimedBenchmarkStats& s) {
    cout << "\n=== EXECUTION TIME BENCHMARK (obstacle disappears after a random time, "
        << s.totalTrials << " trials) ===" << endl;
    cout << "Average time WITHOUT obstacle avoidance: " << fixed << setprecision(1)
        << (s.avgTicksWithout * 0.5) << "s (" << s.avgTicksWithout << " ticks)" << endl;
    cout << "Average time WITH obstacle avoidance: "
        << (s.avgTicksWith * 0.5) << "s (" << s.avgTicksWith << " ticks)" << endl;
    cout << "Average time saved (over ALL trials): " << s.avgSpeedupSeconds << "s ("
        << s.avgSpeedupPct << "%)" << endl;
    cout << "Cases where obstacle avoidance ACTUALLY helped: " << s.meaningfulCases
        << " / " << s.totalTrials << " (" << (100.0 * s.meaningfulCases / s.totalTrials) << "%)" << endl;
    cout << "Average time saved AMONG those cases: " << s.avgSpeedupSecondsWhenHelped << "s" << endl;
    cout << "===========================================================================\n" << endl;
}