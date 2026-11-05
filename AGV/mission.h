#pragma once
#include <vector>
#include <string>
#include "agv.h"
using namespace std;

enum class ActionType {
    NONE,
    AUTOMATIC_PICKUP,
    AUTOMATIC_UNLOADING,
    STATIONARY
};

string actionTypeToString(ActionType action);

struct MissionStep {
    string name;
    vector<Point> path;
    ActionType action;
    double stopTimeSeconds;
};

enum class MissionState {
    MOVING,
    PERFORMING_ACTION,
    COMPLETE
};

class Mission {
public:
    Mission(const string& missionName, const vector<MissionStep>& steps);

    void begin(AGV& agv);
    void update(double dt, AGV& agv);

    bool isComplete() const { return state == MissionState::COMPLETE; }
    string statusLine() const;

    const string& name() const { return missionName; }
    size_t currentStepIndex() const { return stepIndex; }
    size_t totalSteps() const { return steps.size(); }
    string currentStepName() const { return isComplete() ? "" : steps[stepIndex].name; }
    ActionType currentStepAction() const { return isComplete() ? ActionType::NONE : steps[stepIndex].action; }
    bool isPerformingAction() const { return state == MissionState::PERFORMING_ACTION; }
    double remainingActionTime() const { return actionTimer; }

private:
    string missionName;
    vector<MissionStep> steps;
    size_t stepIndex;
    MissionState state;
    double actionTimer;

    void startMovingToCurrentStep(AGV& agv);
    void advanceOrFinish(AGV& agv);
};