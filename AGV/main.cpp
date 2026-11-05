#include "AGV.h"
#include "Sensor.h"
#include "Mission.h"
#include "MqttClient.h"
#include "Telemetry.h"
#include "Markers.h"
#include "Obstacle.h"
#include "Task.h"
#include "PathPlanner.h"
#include "Benchmark.h"
#include "cjson/cJSON.h"
#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <cmath>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
using namespace std;

static string extractAgvIdFromTopic(const string& topic) {
    size_t start = topic.find('/');
    if (start == string::npos) return "";
    size_t end = topic.find('/', start + 1);
    if (end == string::npos) return "";
    return topic.substr(start + 1, end - start - 1);
}

int main(int argc, char* argv[]) {
    string myAgvId = (argc > 1) ? argv[1] : "1";
    int myAgvIdNum = 1;
    try { myAgvIdNum = stoi(myAgvId); }
    catch (...) { myAgvIdNum = 1; }

    const string topicTelemetry = "agv/" + myAgvId + "/telemetry";
    const string topicSensors = "agv/" + myAgvId + "/sensors";
    const string topicMissionStatus = "agv/" + myAgvId + "/mission/status";
    const string topicMarkers = "agv/" + myAgvId + "/markers";
    const string topicObstacles = "agv/" + myAgvId + "/obstacles";
    const string topicTasks = "agv/" + myAgvId + "/tasks";
    const string topicAlerts = "agv/" + myAgvId + "/alerts";
    const string topicCmdObstacle = "agv/" + myAgvId + "/cmd/obstacle";
    const string topicCmdTask = "agv/" + myAgvId + "/cmd/task";
    const string topicCmdSpeed = "agv/" + myAgvId + "/cmd/speed";
    const string topicCmdPlanner = "agv/" + myAgvId + "/cmd/planner";
    const string topicCmdBenchmark = "agv/" + myAgvId + "/cmd/benchmark";
    const string wildcardTelemetry = "agv/+/telemetry";

    cout << "=== AGV #" << myAgvId << " ===" << endl;

    const Point PUT_AWAY = { 0.0, -(myAgvIdNum - 1) * 2.5 };

    const Point RED_TURN = { 10.0, 0.0 };
    const Point RED_TURN2 = { 10.0, 15.0 };
    const Point RED_PICK = { 25.0, 15.0 };

    const Point GREEN_TURN1 = { 8.0, 0.0 };
    const Point GREEN_TURN2 = { 8.0, -12.0 };
    const Point GREEN_TURN3 = { 20.0, -12.0 };
    const Point GREEN_TURN4 = { 20.0, -22.0 };
    const Point GREEN_PICK = { 32.0, -22.0 };

    vector<Point> redPathToPick = { RED_TURN, RED_TURN2, RED_PICK };
    vector<Point> redPathToPutAway = { RED_TURN2, RED_TURN, PUT_AWAY };

    vector<Point> greenPathToPick = { GREEN_TURN1, GREEN_TURN2, GREEN_TURN3, GREEN_TURN4, GREEN_PICK };
    vector<Point> greenPathToPutAway = { GREEN_TURN4, GREEN_TURN3, GREEN_TURN2, GREEN_TURN1, PUT_AWAY };

    vector<MissionStep> redSteps = {
        { "red_pick", redPathToPick,    ActionType::AUTOMATIC_PICKUP,    3.0 },
        { "put_away", redPathToPutAway, ActionType::AUTOMATIC_UNLOADING, 3.0 }
    };
    vector<MissionStep> greenSteps = {
        { "green_pick", greenPathToPick,    ActionType::AUTOMATIC_PICKUP,    3.0 },
        { "put_away",   greenPathToPutAway, ActionType::AUTOMATIC_UNLOADING, 3.0 }
    };

    atomic<int> speedMultiplier{ 1 };
    atomic<bool> plannerEnabled{ true };

    AGV agv(PUT_AWAY.x, PUT_AWAY.y);
    Sensor sensor;
    PathPlanner planner;

    vector<Obstacle> obstacles;
    mutex obstaclesMutex;

    map<string, Point> otherAgvPositions;
    mutex otherAgvMutex;

    deque<Task> taskQueue;
    mutex taskQueueMutex;
    int nextTaskId = 1;

    MqttClient mqtt("agv-simulator-" + myAgvId);

    mqtt.setMessageCallback([&](const string& topic, const string& payload) {
        if (topic == topicCmdObstacle) {
            cJSON* json = cJSON_Parse(payload.c_str());
            if (!json) {
                cerr << "[MQTT] Invalid JSON in command: " << payload << endl;
                return;
            }

            cJSON* actionItem = cJSON_GetObjectItemCaseSensitive(json, "action");
            string action = (actionItem && cJSON_IsString(actionItem)) ? actionItem->valuestring : "";

            lock_guard<mutex> lock(obstaclesMutex);

            if (action == "add") {
                cJSON* xItem = cJSON_GetObjectItemCaseSensitive(json, "x");
                cJSON* yItem = cJSON_GetObjectItemCaseSensitive(json, "y");
                cJSON* typeItem = cJSON_GetObjectItemCaseSensitive(json, "type");
                string typeStr = (typeItem && cJSON_IsString(typeItem)) ? typeItem->valuestring : "static";

                if (cJSON_IsNumber(xItem) && cJSON_IsNumber(yItem)) {
                    Obstacle obs;
                    obs.position = { xItem->valuedouble, yItem->valuedouble };

                    if (typeStr == "person") {
                        obs.type = ObstacleType::PERSON;
                        obs.crossDistance = 2.5;
                    }
                    else {
                        obs.type = ObstacleType::STATIC;
                    }

                    obstacles.push_back(obs);
                    cout << "[MQTT] Added obstacle (" << typeStr << ") at ("
                        << obs.position.x << ", " << obs.position.y << ")" << endl;
                }
            }
            else if (action == "clear") {
                obstacles.clear();
                cout << "[MQTT] All obstacles removed." << endl;
            }

            cJSON_Delete(json);
        }
        else if (topic == topicCmdTask) {
            cJSON* json = cJSON_Parse(payload.c_str());
            if (!json) {
                cerr << "[MQTT] Invalid JSON in command: " << payload << endl;
                return;
            }

            cJSON* actionItem = cJSON_GetObjectItemCaseSensitive(json, "action");
            string action = (actionItem && cJSON_IsString(actionItem)) ? actionItem->valuestring : "";

            if (action == "add") {
                cJSON* routeItem = cJSON_GetObjectItemCaseSensitive(json, "route");
                string route = (routeItem && cJSON_IsString(routeItem)) ? routeItem->valuestring : "red";

                cJSON* priorityItem = cJSON_GetObjectItemCaseSensitive(json, "priority");
                int priority = (priorityItem && cJSON_IsNumber(priorityItem)) ? static_cast<int>(priorityItem->valuedouble) : 2;
                if (priority < 1) priority = 1;
                if (priority > 3) priority = 3;

                Task t;
                {
                    lock_guard<mutex> lock(taskQueueMutex);
                    t.id = nextTaskId++;
                    t.route = route;
                    t.priority = priority;
                    taskQueue.push_back(t);
                }
                cout << "[MQTT] Task #" << t.id << " (" << route << ", priority " << priority << ") added to queue." << endl;
            }
            else if (action == "clear") {
                lock_guard<mutex> lock(taskQueueMutex);
                taskQueue.clear();
                cout << "[MQTT] Task queue cleared." << endl;
            }

            cJSON_Delete(json);
        }
        else if (topic == topicCmdSpeed) {
            cJSON* json = cJSON_Parse(payload.c_str());
            if (!json) {
                cerr << "[MQTT] Invalid JSON in command: " << payload << endl;
                return;
            }

            cJSON* multItem = cJSON_GetObjectItemCaseSensitive(json, "multiplier");
            if (cJSON_IsNumber(multItem)) {
                int m = static_cast<int>(multItem->valuedouble);
                if (m < 1) m = 1;
                if (m > 10) m = 10;
                speedMultiplier = m;
                cout << "[MQTT] Simulation speed changed to x" << m << endl;
            }

            cJSON_Delete(json);
        }
        else if (topic == topicCmdPlanner) {
            cJSON* json = cJSON_Parse(payload.c_str());
            if (!json) {
                cerr << "[MQTT] Invalid JSON in command: " << payload << endl;
                return;
            }

            cJSON* enabledItem = cJSON_GetObjectItemCaseSensitive(json, "enabled");
            if (cJSON_IsBool(enabledItem)) {
                plannerEnabled = cJSON_IsTrue(enabledItem);
                cout << "[MQTT] Obstacle avoidance "
                    << (plannerEnabled.load() ? "ENABLED" : "DISABLED") << "." << endl;
            }

            cJSON_Delete(json);
        }
        else if (topic == topicCmdBenchmark) {
            int trials = 1000;
            cJSON* json = cJSON_Parse(payload.c_str());
            if (json) {
                cJSON* trialsItem = cJSON_GetObjectItemCaseSensitive(json, "trials");
                if (cJSON_IsNumber(trialsItem)) {
                    trials = static_cast<int>(trialsItem->valuedouble);
                    if (trials < 1) trials = 1;
                    if (trials > 5000) trials = 5000;
                }
                cJSON_Delete(json);
            }

            BenchmarkStats stats = runRandomizedBenchmark(redSteps, greenSteps, trials);
            printRandomizedBenchmarkReport(stats);

            TimedBenchmarkStats timedStats = runTimedObstacleBenchmark(redSteps, greenSteps, trials);
            printTimedBenchmarkReport(timedStats);
        }
        else {
            string otherId = extractAgvIdFromTopic(topic);
            if (!otherId.empty() && otherId != myAgvId && topic == ("agv/" + otherId + "/telemetry")) {
                cJSON* json = cJSON_Parse(payload.c_str());
                if (json) {
                    cJSON* xItem = cJSON_GetObjectItemCaseSensitive(json, "x");
                    cJSON* yItem = cJSON_GetObjectItemCaseSensitive(json, "y");
                    if (cJSON_IsNumber(xItem) && cJSON_IsNumber(yItem)) {
                        lock_guard<mutex> lock(otherAgvMutex);
                        otherAgvPositions[otherId] = { xItem->valuedouble, yItem->valuedouble };
                    }
                    cJSON_Delete(json);
                }
            }
        }
        });

    if (!mqtt.connect()) {
        cerr << "Could not connect to the MQTT broker. Check whether the Mosquitto service is running." << endl;
        return 1;
    }
    mqtt.subscribe(topicCmdObstacle);
    mqtt.subscribe(topicCmdTask);
    mqtt.subscribe(topicCmdSpeed);
    mqtt.subscribe(topicCmdPlanner);
    mqtt.subscribe(topicCmdBenchmark);
    mqtt.subscribe(wildcardTelemetry);

    cout << "=== AGV #" << myAgvId << " ready - dispatcher active, waiting for tasks ===" << endl;
    cout << "(in the dashboard, select the vehicle and route and click 'Add to queue', or send" << endl;
    cout << " {\"action\":\"add\",\"route\":\"red\"} on topic " << topicCmdTask << ")" << endl;

    const double dt = 0.5;
    double waitingElapsed = 0.0;
    const double LONG_WAIT_THRESHOLD = 15.0;

    while (true) {
        Task currentTask;
        bool hasTask = false;
        {
            lock_guard<mutex> lock(taskQueueMutex);
            if (!taskQueue.empty()) {
                auto bestIt = taskQueue.begin();
                for (auto it = taskQueue.begin(); it != taskQueue.end(); ++it) {
                    if (it->priority > bestIt->priority) {
                        bestIt = it;
                    }
                }
                currentTask = *bestIt;
                taskQueue.erase(bestIt);
                hasTask = true;
            }
        }

        if (!hasTask) {
            agv.goIdle();
            {
                lock_guard<mutex> lock(taskQueueMutex);
                vector<Task> pending(taskQueue.begin(), taskQueue.end());
                mqtt.publish(topicTasks, buildTaskQueueJson(pending, nullptr));
            }
            mqtt.publish(topicTelemetry, buildTelemetryJson(agv));
            this_thread::sleep_for(chrono::milliseconds(300));
            continue;
        }

        {
            lock_guard<mutex> lock(taskQueueMutex);
            vector<Task> pending(taskQueue.begin(), taskQueue.end());
            mqtt.publish(topicTasks, buildTaskQueueJson(pending, &currentTask));
        }

        vector<MissionStep> steps = (currentTask.route == "green") ? greenSteps : redSteps;
        string missionName = (currentTask.route == "green") ? "delivery_green" : "delivery_red";

        Mission mission(missionName, steps);

        vector<Point> fullPath = steps[0].path;
        for (size_t i = 1; i < steps.size(); i++) {
            for (const auto& p : steps[i].path) fullPath.push_back(p);
        }
        vector<Point> markers = generateCeilingMarkers(fullPath, 3.0);
        vector<Point> sideMarkers = generateSideLaneMarkers(fullPath, 1.2, 3.0);
        markers.insert(markers.end(), sideMarkers.begin(), sideMarkers.end());
        mqtt.publish(topicMarkers, buildMarkersJson(markers), 0, true);

        mission.begin(agv);

        int tickCount = 0;
        const int maxTicks = 800;
        double stopElapsed = 0.0;
        bool detourAttempted = false;
        const double REPLAN_THRESHOLD = 3.0;

        bool onDetour = false;
        size_t detourEndIndex = 0;
        bool wasObstacleStop = false;

        int totalDetourAttempts = 0;
        const int MAX_DETOUR_ATTEMPTS = 12;

        cout << "=== Task #" << currentTask.id << " - mission '" << mission.name()
            << "' (x" << speedMultiplier.load() << " speed) ===" << endl;

        while (!mission.isComplete() && tickCount < maxTicks) {
            mission.update(dt, agv);

            vector<Obstacle> obstaclesSnapshot;
            {
                lock_guard<mutex> lock(obstaclesMutex);
                for (auto it = obstacles.begin(); it != obstacles.end(); ) {
                    if (it->type == ObstacleType::PERSON && it->started) {
                        double stepX = it->velocity.x * dt;
                        double stepY = it->velocity.y * dt;
                        it->position.x += stepX;
                        it->position.y += stepY;
                        it->distanceTraveled += sqrt(stepX * stepX + stepY * stepY);

                        if (it->distanceTraveled >= it->crossDistance) {
                            cout << "[SIM] Person has crossed, obstacle removed." << endl;
                            it = obstacles.erase(it);
                            continue;
                        }
                    }
                    ++it;
                }
                obstaclesSnapshot = obstacles;
            }

            vector<Obstacle> sensorObstacles = obstaclesSnapshot;
            {
                lock_guard<mutex> lock(otherAgvMutex);
                for (const auto& entry : otherAgvPositions) {
                    Obstacle synth;
                    synth.position = entry.second;
                    synth.type = ObstacleType::STATIC;
                    synth.sourceAgvId = entry.first;
                    sensorObstacles.push_back(synth);
                }
            }

            if (onDetour && agv.currentWaypointIndex() >= detourEndIndex) {
                onDetour = false;
            }

            string zone = sensor.evaluateZone(agv.x(), agv.y(), agv.heading(), sensorObstacles, onDetour);
            agv.notifyObstacleZone(zone);

            bool isStoppedNow = (agv.state() == AGVState::OBSTACLE_STOP);
            if (isStoppedNow && !wasObstacleStop) {
                cout << "[SOUND] Warning - obstacle detected, AGV is stopping!" << endl;
                mqtt.publish(topicAlerts, buildAlertJson("obstacle_detected", "Obstacle detected - AGV is stopping"));
            }
            wasObstacleStop = isStoppedNow;

            if (zone == "red") {
                lock_guard<mutex> lock(obstaclesMutex);
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
                            cout << "[SIM] Person noticed the AGV and started crossing." << endl;
                        }
                    }
                }
            }

            if (plannerEnabled.load() && agv.state() == AGVState::OBSTACLE_STOP) {
                stopElapsed += dt;

                bool yieldingToHigherPriorityAgv = false;
                for (const auto& obs : sensorObstacles) {
                    if (!obs.sourceAgvId.empty()) {
                        int otherIdNum = 999999;
                        try { otherIdNum = stoi(obs.sourceAgvId); }
                        catch (...) {}
                        if (otherIdNum < myAgvIdNum) {
                            double dx = obs.position.x - agv.x();
                            double dy = obs.position.y - agv.y();
                            if (sqrt(dx * dx + dy * dy) <= sensor.getRedRadius() + 0.5) {
                                yieldingToHigherPriorityAgv = true;
                                break;
                            }
                        }
                    }
                }

                if (stopElapsed > REPLAN_THRESHOLD && !detourAttempted && !yieldingToHigherPriorityAgv
                    && totalDetourAttempts < MAX_DETOUR_ATTEMPTS) {
                    detourAttempted = true;

                    cout << "[SOUND] Warning - attempting to bypass the obstacle!" << endl;
                    mqtt.publish(topicAlerts, buildAlertJson("attempting_bypass", "AGV is attempting to bypass the obstacle"));

                    Point start = { agv.x(), agv.y() };
                    Point planGoal;
                    Point nextGoalStorage;
                    const Point* nextGoal = nullptr;
                    size_t baseReplaceCount = 1;
                    bool useNestedTarget = onDetour;

                    if (useNestedTarget) {
                        planGoal = agv.routePointAt(detourEndIndex);

                        double distToStart = sqrt(
                            pow(planGoal.x - start.x, 2) + pow(planGoal.y - start.y, 2));
                        if (distToStart < 0.1) {
                            useNestedTarget = false;
                        }
                        else {
                            baseReplaceCount = (detourEndIndex > agv.currentWaypointIndex())
                                ? (detourEndIndex - agv.currentWaypointIndex()) : 1;
                        }
                    }

                    if (!useNestedTarget) {
                        planGoal = agv.currentTarget();
                        baseReplaceCount = 1;

                        vector<Point> upcoming2 = agv.upcomingWaypoints(2);
                        if (upcoming2.size() >= 2) {
                            nextGoalStorage = upcoming2[1];
                            nextGoal = &nextGoalStorage;
                        }
                    }

                    LaneDetourResult laneResult = planner.planLaneDetour(start, planGoal, sensorObstacles, nextGoal);
                    vector<Point> detour;
                    bool usedLane = false;

                    if (!laneResult.points.empty()) {
                        detour = laneResult.points;
                        usedLane = true;
                    }
                    else {
                        detour = planner.planDetour(start, planGoal, sensorObstacles);
                    }

                    if (!detour.empty()) {
                        size_t waypointsToReplace = baseReplaceCount;
                        if (!useNestedTarget && usedLane && laneResult.skippedCorner) {
                            waypointsToReplace = 2;
                        }

                        agv.replaceUpcoming(waypointsToReplace, detour);
                        agv.resumeNavigation();
                        onDetour = true;
                        detourEndIndex = agv.currentWaypointIndex() + detour.size();
                        totalDetourAttempts++;

                        stopElapsed = 0.0;
                        detourAttempted = false;

                        if (usedLane) {
                            cout << "[PLANNER] Switched to a side marker lane ("
                                << detour.size() << " points"
                                << (laneResult.skippedCorner ? ", directly into the next segment" : "")
                                << "), continuing (360-degree sensor during the maneuver). ["
                                << totalDetourAttempts << "/" << MAX_DETOUR_ATTEMPTS << "]" << endl;
                        }
                        else {
                            cout << "[PLANNER] No free side lane - A* alternative path ("
                                << detour.size() << " points), continuing (360-degree sensor during the maneuver). ["
                                << totalDetourAttempts << "/" << MAX_DETOUR_ATTEMPTS << "]" << endl;
                        }
                    }
                    else {
                        cout << "[PLANNER] No free path around the obstacle, still waiting." << endl;
                    }
                }
                else if (stopElapsed > REPLAN_THRESHOLD && !detourAttempted && !yieldingToHigherPriorityAgv
                    && totalDetourAttempts >= MAX_DETOUR_ATTEMPTS) {
                    detourAttempted = true;
                    cout << "[PLANNER] Reached the limit of " << MAX_DETOUR_ATTEMPTS
                        << " consecutive detour attempts - the goal is probably too close to the obstacle. "
                        << "No further attempts, just waiting." << endl;
                }
            }
            else if (agv.state() != AGVState::OBSTACLE_STOP) {
                stopElapsed = 0.0;
                detourAttempted = false;
            }

            agv.update(dt);

            if (agv.state() == AGVState::OBSTACLE_STOP) {
                waitingElapsed += dt;
            }
            else {
                waitingElapsed = 0.0;
            }
            bool longWait = waitingElapsed > LONG_WAIT_THRESHOLD;
            if (longWait && tickCount % 10 == 0) {
                cout << "[WARNING] AGV has been waiting for " << waitingElapsed << "s - obstacle is not being removed." << endl;
            }

            int nearestIdx = nearestMarkerIndex(markers, agv.x(), agv.y());

            mqtt.publish(topicTelemetry, buildTelemetryJson(agv, nearestIdx, longWait));
            mqtt.publish(topicSensors, buildSensorJson(zone));
            mqtt.publish(topicMissionStatus, buildMissionJson(mission));
            mqtt.publish(topicObstacles, buildObstaclesJson(obstaclesSnapshot));
            {
                lock_guard<mutex> lock(taskQueueMutex);
                vector<Task> pending(taskQueue.begin(), taskQueue.end());
                mqtt.publish(topicTasks, buildTaskQueueJson(pending, &currentTask));
            }

            bool interesting = (zone != "clear") || mission.isPerformingAction();
            if (tickCount % 5 == 0 || interesting) {
                cout << "[#" << myAgvId << "] " << agv.toStatusLine() << " | " << mission.statusLine() << endl;
            }

            this_thread::sleep_for(chrono::milliseconds(200 / speedMultiplier.load()));
            tickCount++;
        }

        cout << "=== Task #" << currentTask.id << " finished. Total ticks: " << tickCount << " ===" << endl;
    }

    mqtt.disconnect();
    return 0;
}