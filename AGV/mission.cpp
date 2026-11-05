#include "mission.h"
#include <sstream>
#include <iomanip>
using namespace std;

string actionTypeToString(ActionType action) {
    switch (action) {
    case ActionType::NONE: return "none";
    case ActionType::AUTOMATIC_PICKUP: return "automatic pickup";
    case ActionType::AUTOMATIC_UNLOADING: return "automatic unloading";
    case ActionType::STATIONARY: return "stationary";
    }
    return "unknown";
}

Mission::Mission(const string& missionName_, const vector<MissionStep>& steps_)
    : missionName(missionName_), steps(steps_), stepIndex(0),
    state(MissionState::MOVING), actionTimer(0.0) {
}

void Mission::startMovingToCurrentStep(AGV& agv) {
    agv.setRoute(steps[stepIndex].path);
    agv.startMission();
}

void Mission::begin(AGV& agv) {
    stepIndex = 0;
    state = MissionState::MOVING;
    startMovingToCurrentStep(agv);
}

void Mission::advanceOrFinish(AGV& agv) {
    stepIndex++;
    if (stepIndex >= steps.size()) {
        state = MissionState::COMPLETE;
    }
    else {
        state = MissionState::MOVING;
        startMovingToCurrentStep(agv);
    }
}

void Mission::update(double dt, AGV& agv) {
    if (state == MissionState::COMPLETE) return;

    switch (state) {
    case MissionState::MOVING:
        if (agv.state() == AGVState::MISSION_COMPLETE) {
            const MissionStep& step = steps[stepIndex];
            if (step.action == ActionType::NONE) {
                advanceOrFinish(agv);
            }
            else {
                state = MissionState::PERFORMING_ACTION;
                actionTimer = step.stopTimeSeconds;
            }
        }
        break;

    case MissionState::PERFORMING_ACTION:
        actionTimer -= dt;
        if (actionTimer <= 0.0) {
            advanceOrFinish(agv);
        }
        break;

    default:
        break;
    }
}

string Mission::statusLine() const {
    ostringstream oss;
    oss << "Mission[" << missionName << "] step " << (stepIndex + 1) << "/" << steps.size();

    if (isComplete()) {
        oss << " - COMPLETE";
    }
    else {
        oss << " (" << steps[stepIndex].name << ", " << actionTypeToString(steps[stepIndex].action) << ")";
        if (state == MissionState::PERFORMING_ACTION) {
            oss << " [performing action, " << fixed << setprecision(1) << actionTimer << "s left]";
        }
    }
    return oss.str();
}