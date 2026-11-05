#pragma once
#include <string>
#include "agv.h"
#include "mission.h"
#include "obstacle.h"
#include "task.h"
#include "cjson/cJSON.h"
using namespace std;

inline string buildTelemetryJson(const AGV& agv, int nearestMarker = -1, bool longWait = false) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "x", agv.x());
    cJSON_AddNumberToObject(root, "y", agv.y());
    cJSON_AddNumberToObject(root, "heading", agv.heading());
    cJSON_AddNumberToObject(root, "speed", agv.speed());
    cJSON_AddStringToObject(root, "state", agv.stateToString().c_str());
    if (nearestMarker >= 0) {
        cJSON_AddNumberToObject(root, "nearest_marker", nearestMarker);
    }
    if (longWait) {
        cJSON_AddBoolToObject(root, "long_wait", true);
    }

    char* jsonStr = cJSON_PrintUnformatted(root);
    string result(jsonStr);
    cJSON_free(jsonStr);
    cJSON_Delete(root);
    return result;
}

inline string buildMarkersJson(const vector<Point>& markers) {
    cJSON* root = cJSON_CreateObject();
    cJSON* arr = cJSON_CreateArray();
    for (const auto& m : markers) {
        cJSON* pt = cJSON_CreateObject();
        cJSON_AddNumberToObject(pt, "x", m.x);
        cJSON_AddNumberToObject(pt, "y", m.y);
        cJSON_AddItemToArray(arr, pt);
    }
    cJSON_AddItemToObject(root, "markers", arr);

    char* jsonStr = cJSON_PrintUnformatted(root);
    string result(jsonStr);
    cJSON_free(jsonStr);
    cJSON_Delete(root);
    return result;
}

inline string buildSensorJson(const string& lidarZone) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "lidar_zone", lidarZone.c_str());

    char* jsonStr = cJSON_PrintUnformatted(root);
    string result(jsonStr);
    cJSON_free(jsonStr);
    cJSON_Delete(root);
    return result;
}

inline string buildObstaclesJson(const vector<Obstacle>& obstacles) {
    cJSON* root = cJSON_CreateObject();
    cJSON* arr = cJSON_CreateArray();
    for (const auto& o : obstacles) {
        cJSON* item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "x", o.position.x);
        cJSON_AddNumberToObject(item, "y", o.position.y);
        cJSON_AddStringToObject(item, "type", obstacleTypeToString(o.type).c_str());
        cJSON_AddItemToArray(arr, item);
    }
    cJSON_AddItemToObject(root, "obstacles", arr);

    char* jsonStr = cJSON_PrintUnformatted(root);
    string result(jsonStr);
    cJSON_free(jsonStr);
    cJSON_Delete(root);
    return result;
}

inline string buildTaskQueueJson(const vector<Task>& pending, const Task* current) {
    cJSON* root = cJSON_CreateObject();

    if (current) {
        cJSON* cur = cJSON_CreateObject();
        cJSON_AddNumberToObject(cur, "id", current->id);
        cJSON_AddStringToObject(cur, "route", current->route.c_str());
        cJSON_AddNumberToObject(cur, "priority", current->priority);
        cJSON_AddItemToObject(root, "current", cur);
    }
    else {
        cJSON_AddNullToObject(root, "current");
    }

    cJSON* arr = cJSON_CreateArray();
    for (const auto& t : pending) {
        cJSON* item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "id", t.id);
        cJSON_AddStringToObject(item, "route", t.route.c_str());
        cJSON_AddNumberToObject(item, "priority", t.priority);
        cJSON_AddItemToArray(arr, item);
    }
    cJSON_AddItemToObject(root, "pending", arr);

    char* jsonStr = cJSON_PrintUnformatted(root);
    string result(jsonStr);
    cJSON_free(jsonStr);
    cJSON_Delete(root);
    return result;
}

inline string buildAlertJson(const string& type, const string& message) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", type.c_str());
    cJSON_AddStringToObject(root, "message", message.c_str());

    char* jsonStr = cJSON_PrintUnformatted(root);
    string result(jsonStr);
    cJSON_free(jsonStr);
    cJSON_Delete(root);
    return result;
}

inline string buildMissionJson(const Mission& mission) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "mission", mission.name().c_str());
    cJSON_AddBoolToObject(root, "complete", mission.isComplete());

    size_t displayStep = mission.isComplete()
        ? mission.totalSteps()
        : mission.currentStepIndex() + 1;
    cJSON_AddNumberToObject(root, "step", static_cast<double>(displayStep));
    cJSON_AddNumberToObject(root, "total_steps", static_cast<double>(mission.totalSteps()));

    if (!mission.isComplete()) {
        cJSON_AddStringToObject(root, "step_name", mission.currentStepName().c_str());
        cJSON_AddStringToObject(root, "action", actionTypeToString(mission.currentStepAction()).c_str());
        cJSON_AddBoolToObject(root, "performing_action", mission.isPerformingAction());
        if (mission.isPerformingAction()) {
            cJSON_AddNumberToObject(root, "action_remaining", mission.remainingActionTime());
        }
    }

    char* jsonStr = cJSON_PrintUnformatted(root);
    string result(jsonStr);
    cJSON_free(jsonStr);
    cJSON_Delete(root);
    return result;
}