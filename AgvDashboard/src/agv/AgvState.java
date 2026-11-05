package agv;

import java.util.ArrayList;
import java.util.List;

public class AgvState {
    private volatile double x = 0;
    private volatile double y = 0;
    private volatile double heading = 0;
    private volatile double speed = 0;
    private volatile String state = "IDLE";
    private volatile String lidarZone = "clear";
    private volatile int nearestMarker = -1;
    private volatile boolean longWait = false;

    private volatile String missionName = "-";
    private volatile int missionStep = 0;
    private volatile int missionTotalSteps = 0;
    private volatile boolean missionComplete = false;
    private volatile String missionStepName = "";
    private volatile String missionAction = "";
    private volatile boolean performingAction = false;
    private volatile double actionRemaining = 0;

    private volatile int currentTaskId = -1;
    private volatile String currentTaskRoute = null;
    private volatile int currentTaskPriority = 2;
    private volatile List<String> pendingRoutes = new ArrayList<>();

    public synchronized void updateTelemetry(double x, double y, double heading, double speed, String state, int nearestMarker, boolean longWait) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.speed = speed;
        this.state = state;
        this.nearestMarker = nearestMarker;
        this.longWait = longWait;
    }

    public synchronized void updateSensors(String lidarZone) {
        this.lidarZone = lidarZone;
    }

    public synchronized void updateMission(String missionName, int step, int totalSteps, boolean complete,
                                            String stepName, String action, boolean performingAction, double actionRemaining) {
        this.missionName = missionName;
        this.missionStep = step;
        this.missionTotalSteps = totalSteps;
        this.missionComplete = complete;
        this.missionStepName = stepName;
        this.missionAction = action;
        this.performingAction = performingAction;
        this.actionRemaining = actionRemaining;
    }

    public synchronized void updateTasks(int currentTaskId, String currentTaskRoute, int currentTaskPriority, List<String> pendingRoutes) {
        this.currentTaskId = currentTaskId;
        this.currentTaskRoute = currentTaskRoute;
        this.currentTaskPriority = currentTaskPriority;
        this.pendingRoutes = pendingRoutes;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    public double getSpeed() { return speed; }
    public String getState() { return state; }
    public String getLidarZone() { return lidarZone; }
    public int getNearestMarker() { return nearestMarker; }
    public boolean isLongWait() { return longWait; }

    public String getMissionName() { return missionName; }
    public int getMissionStep() { return missionStep; }
    public int getMissionTotalSteps() { return missionTotalSteps; }
    public boolean isMissionComplete() { return missionComplete; }
    public String getMissionStepName() { return missionStepName; }
    public String getMissionAction() { return missionAction; }
    public boolean isPerformingAction() { return performingAction; }
    public double getActionRemaining() { return actionRemaining; }

    public int getCurrentTaskId() { return currentTaskId; }
    public String getCurrentTaskRoute() { return currentTaskRoute; }
    public int getCurrentTaskPriority() { return currentTaskPriority; }
    public List<String> getPendingRoutes() { return pendingRoutes; }
}