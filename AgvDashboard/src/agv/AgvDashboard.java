package agv;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.eclipse.paho.client.mqttv3.*;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.json.JSONArray;
import org.json.JSONObject;

public class AgvDashboard {

    private static final List<double[]> RED_WAYPOINTS = List.of(
            new double[]{0.0, 0.0},
            new double[]{10.0, 0.0},
            new double[]{10.0, 15.0},
            new double[]{25.0, 15.0}
    );
    private static final List<double[]> GREEN_WAYPOINTS = List.of(
            new double[]{0.0, 0.0},
            new double[]{8.0, 0.0},
            new double[]{8.0, -12.0},
            new double[]{20.0, -12.0},
            new double[]{20.0, -22.0},
            new double[]{32.0, -22.0}
    );

    private final Map<String, AgvState> agvStates = new ConcurrentHashMap<>();
    private final Map<String, List<double[]>> markersByAgv = new ConcurrentHashMap<>();
    private final Map<String, List<double[]>> obstaclesByAgv = new ConcurrentHashMap<>();
    private final List<double[]> globalStaticObstacles = new ArrayList<>();
    private final Map<String, Long> lastSeenByAgv = new ConcurrentHashMap<>();
    private static final long STALE_THRESHOLD_MS = 3000;

    private JLabel stateLabel, posLabel, speedLabel, zoneLabel, missionLabel, warningLabel, queueLabel, alertLabel;
    private javax.swing.Timer alertClearTimer;
    private JRadioButton redRadio, greenRadio;
    private JComboBox<String> priorityCombo;
    private JComboBox<String> agvSelector;
    private MqttClient mqttClient;
    private MapPanel mapPanelRef;

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> new AgvDashboard().start());
    }

    private void start() {
        JFrame frame = new JFrame("AGV Dashboard - Fleet");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        MapPanel mapPanel = new MapPanel(agvStates, new ArrayList<>(RED_WAYPOINTS), new ArrayList<>(GREEN_WAYPOINTS));
        mapPanel.setObstacleClickListener((x, y, isPerson) -> publishObstacleCommand(x, y, "add", isPerson));
        mapPanelRef = mapPanel;

        frame.setLayout(new BorderLayout());
        frame.add(mapPanel, BorderLayout.CENTER);

        JScrollPane statusScroll = new JScrollPane(buildStatusPanel(mapPanel));
        statusScroll.setBorder(BorderFactory.createEmptyBorder());
        statusScroll.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
        statusScroll.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED);
        statusScroll.getVerticalScrollBar().setUnitIncrement(16);
        statusScroll.setBackground(BG_PANEL);
        statusScroll.getViewport().setBackground(BG_PANEL);
        statusScroll.setPreferredSize(new Dimension(312, 950));
        frame.getContentPane().setBackground(BG_PANEL);
        frame.add(statusScroll, BorderLayout.EAST);

        frame.setMinimumSize(new Dimension(700, 500));
        frame.pack();
        if (frame.getWidth() > 1200 || frame.getHeight() > 820) {
            frame.setSize(1150, 780);
        }
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);

        connectMqtt(mapPanel);

        javax.swing.Timer staleCheckTimer = new javax.swing.Timer(1000, e -> pruneStaleVehicles());
        staleCheckTimer.start();
    }

    private String getFocusedAgvId() {
        return (String) agvSelector.getSelectedItem();
    }

    private static double[] computeHomePoint(String agvId) {
        int idNum = 1;
        try { idNum = Integer.parseInt(agvId); } catch (NumberFormatException ignored) {}
        return new double[]{0.0, -(idNum - 1) * 2.5};
    }

    private static List<double[]> computeWaypointsForAgv(List<double[]> base, String agvId) {
        List<double[]> result = new ArrayList<>(base);
        result.set(0, computeHomePoint(agvId));
        return result;
    }

    private void pruneStaleVehicles() {
        long now = System.currentTimeMillis();
        List<String> staleIds = new ArrayList<>();
        for (Map.Entry<String, Long> entry : lastSeenByAgv.entrySet()) {
            if (now - entry.getValue() > STALE_THRESHOLD_MS) {
                staleIds.add(entry.getKey());
            }
        }
        if (staleIds.isEmpty()) return;

        for (String id : staleIds) {
            agvStates.remove(id);
            markersByAgv.remove(id);
            obstaclesByAgv.remove(id);
            lastSeenByAgv.remove(id);
        }

        SwingUtilities.invokeLater(() -> {
            String currentFocus = getFocusedAgvId();
            DefaultComboBoxModel<String> model = (DefaultComboBoxModel<String>) agvSelector.getModel();
            for (String id : staleIds) {
                model.removeElement(id);
            }
            mapPanelRef.repaint();

            if (currentFocus != null && staleIds.contains(currentFocus)) {
                if (model.getSize() > 0) {
                    agvSelector.setSelectedIndex(0);
                } else {
                    stateLabel.setText("-");
                    posLabel.setText("-");
                    speedLabel.setText("-");
                    zoneLabel.setText("-");
                    warningLabel.setText(" ");
                    missionLabel.setText("-");
                    queueLabel.setText("-");
                    mapPanelRef.setMarkers(new ArrayList<>());
                    mapPanelRef.setObstacles(new ArrayList<>());
                }
            }
        });
    }

    private void ensureAgvKnown(String id) {
        lastSeenByAgv.put(id, System.currentTimeMillis());
        if (agvStates.containsKey(id)) return;
        agvStates.put(id, new AgvState());

        for (double[] obs : globalStaticObstacles) {
            sendObstacleCommandTo(id, obs[0], obs[1], "add", false);
        }

        SwingUtilities.invokeLater(() -> {
            DefaultComboBoxModel<String> model = (DefaultComboBoxModel<String>) agvSelector.getModel();
            boolean present = false;
            for (int i = 0; i < model.getSize(); i++) {
                if (model.getElementAt(i).equals(id)) { present = true; break; }
            }
            boolean hadNoSelection = agvSelector.getSelectedItem() == null;
            if (!present) {
                model.addElement(id);
            }
            if (hadNoSelection) {
                agvSelector.setSelectedItem(id);
            }
        });
    }

    private static final Color BG_PANEL = new Color(247, 248, 250);
    private static final Color CARD_BG = new Color(255, 255, 255);
    private static final Color CARD_BORDER = new Color(228, 230, 235);
    private static final Color TEXT_PRIMARY = new Color(30, 41, 59);
    private static final Color TEXT_SECONDARY = new Color(120, 128, 140);
    private static final Color ACCENT_GREEN = new Color(52, 168, 105);
    private static final Color ACCENT_RED = new Color(220, 76, 76);
    private static final Color ACCENT_BLUE = new Color(66, 133, 219);
    private static final Color ACCENT_AMBER = new Color(214, 148, 40);
    private static final Color NEUTRAL_BTN = new Color(241, 243, 246);

    private JPanel buildStatusPanel(MapPanel mapPanel) {
        JPanel panel = new JPanel();
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
        panel.setAlignmentX(Component.LEFT_ALIGNMENT);
        panel.setBackground(BG_PANEL);
        panel.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createMatteBorder(0, 1, 0, 0, CARD_BORDER),
                BorderFactory.createEmptyBorder(18, 14, 18, 14)));
        panel.setPreferredSize(new Dimension(290, 800));

        JLabel header = new JLabel("AGV Dashboard");
        header.setFont(header.getFont().deriveFont(Font.BOLD, 18f));
        header.setForeground(TEXT_PRIMARY);
        header.setAlignmentX(Component.CENTER_ALIGNMENT);
        header.setHorizontalAlignment(SwingConstants.LEFT);
        header.setMaximumSize(new Dimension(500, 30));
        panel.add(header);

        JPanel vehicleCard = beginCard(panel);
        addCardTitle(vehicleCard, "VEHICLE");
        agvSelector = new JComboBox<>();
        agvSelector.setAlignmentX(Component.LEFT_ALIGNMENT);
        agvSelector.setMaximumSize(new Dimension(500, 30));
        agvSelector.addActionListener(e -> onAgvSelectionChanged());
        vehicleCard.add(agvSelector);
        endCard(panel, vehicleCard);

        JPanel statsCard = beginCard(panel);
        stateLabel = addStatRow(statsCard, "STATE");
        posLabel = addStatRow(statsCard, "POSITION");
        speedLabel = addStatRow(statsCard, "SPEED");
        zoneLabel = addStatRow(statsCard, "LIDAR ZONE", true);
        endCard(panel, statsCard);

        alertLabel = new JLabel(" ");
        alertLabel.setFont(alertLabel.getFont().deriveFont(Font.BOLD, 12f));
        alertLabel.setForeground(ACCENT_AMBER);
        alertLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
        alertLabel.setMaximumSize(new Dimension(500, 20));
        panel.add(alertLabel);

        warningLabel = new JLabel(" ");
        warningLabel.setFont(warningLabel.getFont().deriveFont(Font.BOLD, 12f));
        warningLabel.setForeground(ACCENT_RED);
        warningLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
        warningLabel.setMaximumSize(new Dimension(500, 20));
        panel.add(warningLabel);
        panel.add(Box.createVerticalStrut(10));

        JPanel missionCard = beginCard(panel);
        addCardTitle(missionCard, "MISSION");
        missionLabel = new JLabel("-");
        missionLabel.setFont(missionLabel.getFont().deriveFont(Font.PLAIN, 13f));
        missionLabel.setForeground(TEXT_PRIMARY);
        missionLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
        missionLabel.setHorizontalAlignment(SwingConstants.LEFT);
        missionLabel.setMaximumSize(new Dimension(500, 50));
        missionCard.add(missionLabel);
        missionCard.add(Box.createVerticalStrut(10));

        addCardTitle(missionCard, "QUEUE");
        queueLabel = new JLabel("-");
        queueLabel.setFont(queueLabel.getFont().deriveFont(Font.PLAIN, 13f));
        queueLabel.setForeground(TEXT_PRIMARY);
        queueLabel.setAlignmentX(Component.LEFT_ALIGNMENT);
        queueLabel.setHorizontalAlignment(SwingConstants.LEFT);
        queueLabel.setMaximumSize(new Dimension(500, 50));
        missionCard.add(queueLabel);
        endCard(panel, missionCard);

        JPanel routeCard = beginCard(panel);
        addCardTitle(routeCard, "ROUTE");
        routeCard.add(Box.createVerticalStrut(4));

        redRadio = new JRadioButton("Red (~220m)");
        greenRadio = new JRadioButton("Green (~280m)");
        ButtonGroup group = new ButtonGroup();
        group.add(redRadio);
        group.add(greenRadio);
        redRadio.setSelected(true);

        for (JRadioButton rb : new JRadioButton[]{redRadio, greenRadio}) {
            rb.setOpaque(false);
            rb.setAlignmentX(Component.LEFT_ALIGNMENT);
            rb.setMaximumSize(new Dimension(500, 24));
            rb.setFont(rb.getFont().deriveFont(13f));
            rb.setFocusPainted(false);
        }
        redRadio.setForeground(new Color(190, 60, 60));
        greenRadio.setForeground(new Color(40, 145, 90));

        redRadio.addActionListener(e -> mapPanel.setSelectedRoute("red"));
        greenRadio.addActionListener(e -> mapPanel.setSelectedRoute("green"));

        routeCard.add(redRadio);
        routeCard.add(Box.createVerticalStrut(3));
        routeCard.add(greenRadio);
        routeCard.add(Box.createVerticalStrut(12));

        addCardTitle(routeCard, "PRIORITY");
        routeCard.add(Box.createVerticalStrut(4));
        priorityCombo = new JComboBox<>(new String[]{"Low", "Normal", "High"});
        priorityCombo.setSelectedIndex(1);
        priorityCombo.setAlignmentX(Component.LEFT_ALIGNMENT);
        priorityCombo.setMaximumSize(new Dimension(500, 28));
        routeCard.add(priorityCombo);
        endCard(panel, routeCard);

        JButton addTaskButton = makeButton("Add to queue", ACCENT_GREEN, Color.WHITE);
        addTaskButton.addActionListener(e -> publishAddTaskCommand());
        panel.add(addTaskButton);
        panel.add(Box.createVerticalStrut(8));

        JButton clearQueueButton = makeButton("Clear queue", NEUTRAL_BTN, TEXT_PRIMARY);
        clearQueueButton.addActionListener(e -> publishClearQueueCommand());
        panel.add(clearQueueButton);
        panel.add(Box.createVerticalStrut(8));

        JToggleButton speedButton = makeToggleButton("Speed up (x3)", ACCENT_BLUE, Color.WHITE);
        speedButton.addActionListener(e -> {
            boolean fast = speedButton.isSelected();
            speedButton.setText(fast ? "Normal speed (x1)" : "Speed up (x3)");
            publishSpeedCommand(fast ? 3 : 1);
        });
        panel.add(speedButton);
        panel.add(Box.createVerticalStrut(8));

        JButton clearButton = makeButton("Delete obstacles", NEUTRAL_BTN, TEXT_PRIMARY);
        clearButton.addActionListener(e -> {
            mapPanel.clearObstacles();
            publishObstacleCommand(0, 0, "clear", false);
        });
        panel.add(clearButton);
        panel.add(Box.createVerticalStrut(8));

        JToggleButton plannerToggle = makeToggleButton("Disable obstacle avoidance", ACCENT_RED, Color.WHITE);
        plannerToggle.setSelected(false);
        plannerToggle.addActionListener(e -> {
            boolean disabled = plannerToggle.isSelected();
            plannerToggle.setText(disabled ? "Enable obstacle avoidance" : "Disable obstacle avoidance");
            publishPlannerToggleCommand(!disabled);
        });
        panel.add(plannerToggle);
        panel.add(Box.createVerticalStrut(8));

        JButton benchmarkButton = makeButton("Run benchmark", NEUTRAL_BTN, TEXT_PRIMARY);
        benchmarkButton.addActionListener(e -> publishBenchmarkCommand());
        panel.add(benchmarkButton);

        panel.add(Box.createVerticalGlue());

        return panel;
    }

    private JPanel beginCard(JPanel parent) {
    	JPanel card = new JPanel();
        card.setLayout(new BoxLayout(card, BoxLayout.Y_AXIS));
        card.setAlignmentX(Component.CENTER_ALIGNMENT); 
        card.setBackground(CARD_BG);
        card.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(CARD_BORDER, 1, true),
                BorderFactory.createEmptyBorder(12, 12, 12, 12)));
        card.setMaximumSize(new Dimension(500, 900));
        return card;
    }

    private void endCard(JPanel parent, JPanel card) {
        parent.add(card);
        parent.add(Box.createVerticalStrut(10));
    }

    private void addCardTitle(JPanel card, String text) {
    	JLabel l = new JLabel(text);
        l.setFont(l.getFont().deriveFont(Font.BOLD, 10.5f));
        l.setForeground(TEXT_SECONDARY);
        l.setAlignmentX(Component.CENTER_ALIGNMENT);
        l.setHorizontalAlignment(SwingConstants.LEFT);
        l.setMaximumSize(new Dimension(500, 18));
        card.add(l);
        card.add(Box.createVerticalStrut(5));
    }

    private void onAgvSelectionChanged() {
        String id = getFocusedAgvId();
        if (id == null) {
            return;
        }
        mapPanelRef.setFocusedAgvId(id);
        mapPanelRef.setRouteWaypoints(
                computeWaypointsForAgv(RED_WAYPOINTS, id),
                computeWaypointsForAgv(GREEN_WAYPOINTS, id));

        AgvState st = agvStates.get(id);
        if (st != null) {
            stateLabel.setText(st.getState());
            posLabel.setText(String.format("(%.1f, %.1f)", st.getX(), st.getY()));
            speedLabel.setText(String.format("%.2f m/s", st.getSpeed()));
            zoneLabel.setText(st.getLidarZone());
            warningLabel.setText(st.isLongWait() ? "⚠ AGV waiting too long" : " ");
            missionLabel.setText(formatMissionText(st));
            queueLabel.setText(formatQueueText(st));
        }

        List<double[]> cachedMarkers = markersByAgv.get(id);
        mapPanelRef.setMarkers(cachedMarkers != null ? cachedMarkers : new ArrayList<>());
        List<double[]> cachedObstacles = obstaclesByAgv.get(id);
        mapPanelRef.setObstacles(cachedObstacles != null ? cachedObstacles : new ArrayList<>());
    }

    private JLabel addStatRow(JPanel parent, String title) {
        return addStatRow(parent, title, false);
    }

    private JLabel addStatRow(JPanel parent, String title, boolean isLast) {
        JPanel row = new JPanel();
        row.setLayout(new BoxLayout(row, BoxLayout.X_AXIS));
        row.setOpaque(false);
        row.setAlignmentX(Component.LEFT_ALIGNMENT);
        row.setMaximumSize(new Dimension(500, 22));

        JLabel titleLabel = new JLabel(title);
        titleLabel.setFont(titleLabel.getFont().deriveFont(Font.PLAIN, 11.5f));
        titleLabel.setForeground(TEXT_SECONDARY);

        JLabel valueLabel = new JLabel("-");
        valueLabel.setFont(valueLabel.getFont().deriveFont(Font.BOLD, 13f));
        valueLabel.setForeground(TEXT_PRIMARY);
        valueLabel.setHorizontalAlignment(SwingConstants.RIGHT);

        row.add(titleLabel);
        row.add(Box.createHorizontalGlue());
        row.add(valueLabel);

        parent.add(row);
        if (!isLast) {
            parent.add(Box.createVerticalStrut(9));
        }

        return valueLabel;
    }

    private JButton makeButton(String text, Color bg, Color fg) {
        JButton b = new RoundedButton(text, bg, fg);
        b.setAlignmentX(Component.CENTER_ALIGNMENT);
        b.setMaximumSize(new Dimension(500, 38));
        return b;
    }

    private JToggleButton makeToggleButton(String text, Color bg, Color fg) {
        JToggleButton b = new RoundedToggleButton(text, bg, fg);
        b.setAlignmentX(Component.CENTER_ALIGNMENT);
        b.setMaximumSize(new Dimension(500, 38));
        return b;
    }

    private static class RoundedButton extends JButton {
        private final Color bg;
        private final Color fg;

        RoundedButton(String text, Color bg, Color fg) {
            super(text);
            this.bg = bg;
            this.fg = fg;
            setContentAreaFilled(false);
            setBorderPainted(false);
            setFocusPainted(false);
            setForeground(fg);
            setFont(getFont().deriveFont(Font.BOLD, 12.5f));
        }

        @Override
        protected void paintComponent(Graphics g) {
            Graphics2D g2 = (Graphics2D) g.create();
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            Color fillColor = bg;
            if (getModel().isPressed()) {
                fillColor = bg.darker();
            } else if (getModel().isRollover()) {
                fillColor = brighten(bg);
            }
            g2.setColor(fillColor);
            g2.fillRoundRect(0, 0, getWidth(), getHeight(), 10, 10);
            g2.dispose();
            super.paintComponent(g);
        }

        private Color brighten(Color c) {
            return new Color(
                    Math.min(255, c.getRed() + 10),
                    Math.min(255, c.getGreen() + 10),
                    Math.min(255, c.getBlue() + 10));
        }
    }

    private static class RoundedToggleButton extends JToggleButton {
        private final Color bg;
        private final Color fg;

        RoundedToggleButton(String text, Color bg, Color fg) {
            super(text);
            this.bg = bg;
            this.fg = fg;
            setContentAreaFilled(false);
            setBorderPainted(false);
            setFocusPainted(false);
            setForeground(fg);
            setFont(getFont().deriveFont(Font.BOLD, 12.5f));
        }

        @Override
        protected void paintComponent(Graphics g) {
            Graphics2D g2 = (Graphics2D) g.create();
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            Color fillColor = isSelected() ? bg.darker() : bg;
            if (getModel().isRollover()) {
                fillColor = fillColor.darker();
            }
            g2.setColor(fillColor);
            g2.fillRoundRect(0, 0, getWidth(), getHeight(), 10, 10);
            g2.dispose();
            super.paintComponent(g);
        }
    }

    private void connectMqtt(MapPanel mapPanel) {
        try {
            mqttClient = new MqttClient("tcp://localhost:1883", "java-dashboard-1", new MemoryPersistence());
            MqttConnectOptions options = new MqttConnectOptions();
            options.setAutomaticReconnect(true);
            options.setCleanSession(true);

            mqttClient.setCallback(new MqttCallback() {
                @Override
                public void connectionLost(Throwable cause) {
                    System.out.println("[MQTT] Connection lost: " + cause.getMessage());
                }

                @Override
                public void messageArrived(String topic, MqttMessage message) {
                    String payload = new String(message.getPayload());
                    handleMessage(topic, payload, mapPanel);
                }

                @Override
                public void deliveryComplete(IMqttDeliveryToken token) {
                }
            });

            mqttClient.connect(options);
            mqttClient.subscribe("agv/+/telemetry");
            mqttClient.subscribe("agv/+/sensors");
            mqttClient.subscribe("agv/+/mission/status");
            mqttClient.subscribe("agv/+/markers");
            mqttClient.subscribe("agv/+/obstacles");
            mqttClient.subscribe("agv/+/tasks");
            mqttClient.subscribe("agv/+/alerts");

            System.out.println("[MQTT] Connected to broker.");
        } catch (MqttException e) {
            e.printStackTrace();
            JOptionPane.showMessageDialog(null,
                    "Could not connect to the MQTT broker. Check whether Mosquitto is running.",
                    "Error", JOptionPane.ERROR_MESSAGE);
        }
    }

    private String extractAgvId(String topic) {
        String[] parts = topic.split("/");
        return (parts.length >= 2) ? parts[1] : "1";
    }

    private void handleMessage(String topic, String payload, MapPanel mapPanel) {
        try {
            String id = extractAgvId(topic);
            ensureAgvKnown(id);
            boolean isFocused = id.equals(getFocusedAgvId());

            if (topic.endsWith("/markers")) {
                JSONObject json = new JSONObject(payload);
                JSONArray arr = json.getJSONArray("markers");
                List<double[]> newMarkers = new ArrayList<>();
                for (int i = 0; i < arr.length(); i++) {
                    JSONObject pt = arr.getJSONObject(i);
                    newMarkers.add(new double[]{pt.getDouble("x"), pt.getDouble("y")});
                }
                markersByAgv.put(id, newMarkers);
                if (isFocused) {
                    SwingUtilities.invokeLater(() -> mapPanel.setMarkers(newMarkers));
                }
                return;
            }

            if (topic.endsWith("/obstacles")) {
                JSONObject json = new JSONObject(payload);
                JSONArray arr = json.getJSONArray("obstacles");
                List<double[]> newObstacles = new ArrayList<>();
                for (int i = 0; i < arr.length(); i++) {
                    JSONObject o = arr.getJSONObject(i);
                    boolean isPerson = "person".equals(o.getString("type"));
                    newObstacles.add(new double[]{o.getDouble("x"), o.getDouble("y"), isPerson ? 1.0 : 0.0});
                }
                obstaclesByAgv.put(id, newObstacles);
                if (isFocused) {
                    SwingUtilities.invokeLater(() -> mapPanel.setObstacles(newObstacles));
                }
                return;
            }

            if (topic.endsWith("/alerts")) {
                JSONObject json = new JSONObject(payload);
                String type = json.getString("type");
                String message = json.getString("message");
                String prefixedMessage = "AGV " + id + ": " + message;
                SwingUtilities.invokeLater(() -> showAlert(type, prefixedMessage));
                return;
            }

            AgvState state = agvStates.get(id);
            if (state == null) return;

            if (topic.endsWith("/tasks")) {
                JSONObject json = new JSONObject(payload);
                int curId = -1;
                String curRoute = null;
                int curPriority = 2;
                if (json.has("current") && !json.isNull("current")) {
                    JSONObject cur = json.getJSONObject("current");
                    curId = cur.getInt("id");
                    curRoute = cur.getString("route");
                    curPriority = cur.optInt("priority", 2);
                }
                JSONArray arr = json.getJSONArray("pending");
                List<String> pending = new ArrayList<>();
                for (int i = 0; i < arr.length(); i++) {
                    JSONObject p = arr.getJSONObject(i);
                    pending.add(p.getString("route") + " (P" + p.optInt("priority", 2) + ")");
                }
                state.updateTasks(curId, curRoute, curPriority, pending);
                if (isFocused) {
                    SwingUtilities.invokeLater(() -> queueLabel.setText(formatQueueText(state)));
                }
                return;
            }

            JSONObject json = new JSONObject(payload);

            if (topic.endsWith("/telemetry")) {
                state.updateTelemetry(
                        json.getDouble("x"),
                        json.getDouble("y"),
                        json.getDouble("heading"),
                        json.getDouble("speed"),
                        json.getString("state"),
                        json.optInt("nearest_marker", -1),
                        json.optBoolean("long_wait", false)
                );
            } else if (topic.endsWith("/sensors")) {
                state.updateSensors(json.getString("lidar_zone"));
            } else if (topic.endsWith("/mission/status") || topic.endsWith("/status")) {
                state.updateMission(
                        json.getString("mission"),
                        json.getInt("step"),
                        json.getInt("total_steps"),
                        json.getBoolean("complete"),
                        json.optString("step_name", ""),
                        json.optString("action", ""),
                        json.optBoolean("performing_action", false),
                        json.optDouble("action_remaining", 0.0)
                );
            }

            SwingUtilities.invokeLater(() -> {
                mapPanel.repaint();
                if (isFocused) {
                    stateLabel.setText(state.getState());
                    posLabel.setText(String.format("(%.1f, %.1f)", state.getX(), state.getY()));
                    speedLabel.setText(String.format("%.2f m/s", state.getSpeed()));
                    zoneLabel.setText(state.getLidarZone());
                    warningLabel.setText(state.isLongWait() ? "⚠ AGV waiting too long" : " ");
                    missionLabel.setText(formatMissionText(state));
                }
            });

        } catch (Exception e) {
            System.err.println("Error parsing MQTT message: " + payload);
        }
    }

    private void showAlert(String type, String message) {
        String icon = "obstacle_detected".equals(type) ? "\uD83D\uDD0A" : "\uD83D\uDD01";
        alertLabel.setText(icon + " " + message);

        Toolkit.getDefaultToolkit().beep();
        if ("attempting_bypass".equals(type)) {
            javax.swing.Timer secondBeep = new javax.swing.Timer(180, e -> Toolkit.getDefaultToolkit().beep());
            secondBeep.setRepeats(false);
            secondBeep.start();
        }

        if (alertClearTimer != null) {
            alertClearTimer.stop();
        }
        alertClearTimer = new javax.swing.Timer(3000, e -> alertLabel.setText(" "));
        alertClearTimer.setRepeats(false);
        alertClearTimer.start();
    }

    private String formatMissionText(AgvState state) {
        if (state.getMissionTotalSteps() == 0) {
            return "-";
        }
        StringBuilder sb = new StringBuilder("<html>" + state.getMissionName()
                + " (" + state.getMissionStep() + "/" + state.getMissionTotalSteps() + ")");

        if (state.isMissionComplete()) {
            sb.append("<br><b>FINISHED</b>");
        } else {
            sb.append("<br>").append(state.getMissionStepName())
                    .append(" - ").append(state.getMissionAction());
            if (state.isPerformingAction()) {
                sb.append(String.format("<br>[%.1fs left]", state.getActionRemaining()));
            }
        }
        sb.append("</html>");
        return sb.toString();
    }

    private String formatQueueText(AgvState state) {
        StringBuilder sb = new StringBuilder("<html>");
        if (state.getCurrentTaskRoute() == null) {
            sb.append("Free");
        } else {
            sb.append("In progress: #").append(state.getCurrentTaskId())
                    .append(" (").append(state.getCurrentTaskRoute())
                    .append(", P").append(state.getCurrentTaskPriority()).append(")");
        }

        List<String> pending = state.getPendingRoutes();
        if (!pending.isEmpty()) {
            sb.append("<br>Pending: ").append(pending.size());
            sb.append(" (").append(String.join(", ", pending)).append(")");
        }
        sb.append("</html>");
        return sb.toString();
    }

    private void publishAddTaskCommand() {
        if (mqttClient == null || !mqttClient.isConnected()) {
            System.err.println("[MQTT] Not connected to broker, command not sent.");
            return;
        }
        String focusedId = getFocusedAgvId();
        if (focusedId == null) {
            System.err.println("[MQTT] No vehicle connected yet, command not sent.");
            return;
        }
        try {
            JSONObject json = new JSONObject();
            json.put("action", "add");
            json.put("route", redRadio.isSelected() ? "red" : "green");
            json.put("priority", priorityCombo.getSelectedIndex() + 1);

            MqttMessage message = new MqttMessage(json.toString().getBytes());
            message.setQos(0);
            mqttClient.publish("agv/" + focusedId + "/cmd/task", message);

        } catch (MqttException e) {
            e.printStackTrace();
        }
    }

    private void publishClearQueueCommand() {
        if (mqttClient == null || !mqttClient.isConnected()) {
            System.err.println("[MQTT] Not connected to broker, command not sent.");
            return;
        }
        String focusedId = getFocusedAgvId();
        if (focusedId == null) {
            System.err.println("[MQTT] No vehicle connected yet, command not sent.");
            return;
        }
        try {
            JSONObject json = new JSONObject();
            json.put("action", "clear");

            MqttMessage message = new MqttMessage(json.toString().getBytes());
            message.setQos(0);
            mqttClient.publish("agv/" + focusedId + "/cmd/task", message);

        } catch (MqttException e) {
            e.printStackTrace();
        }
    }

    private void publishSpeedCommand(int multiplier) {
        if (mqttClient == null || !mqttClient.isConnected()) {
            System.err.println("[MQTT] Not connected to broker, command not sent.");
            return;
        }
        for (String id : agvStates.keySet()) {
            try {
                JSONObject json = new JSONObject();
                json.put("multiplier", multiplier);

                MqttMessage message = new MqttMessage(json.toString().getBytes());
                message.setQos(0);
                mqttClient.publish("agv/" + id + "/cmd/speed", message);

            } catch (MqttException e) {
                e.printStackTrace();
            }
        }
    }

    private void publishPlannerToggleCommand(boolean enabled) {
        if (mqttClient == null || !mqttClient.isConnected()) {
            System.err.println("[MQTT] Not connected to broker, command not sent.");
            return;
        }
        String focusedId = getFocusedAgvId();
        if (focusedId == null) {
            System.err.println("[MQTT] No vehicle connected yet, command not sent.");
            return;
        }
        try {
            JSONObject json = new JSONObject();
            json.put("enabled", enabled);

            MqttMessage message = new MqttMessage(json.toString().getBytes());
            message.setQos(0);
            mqttClient.publish("agv/" + focusedId + "/cmd/planner", message);

        } catch (MqttException e) {
            e.printStackTrace();
        }
    }

    private void publishBenchmarkCommand() {
        if (mqttClient == null || !mqttClient.isConnected()) {
            System.err.println("[MQTT] Not connected to broker, command not sent.");
            return;
        }
        String focusedId = getFocusedAgvId();
        if (focusedId == null) {
            System.err.println("[MQTT] No vehicle connected yet, command not sent.");
            return;
        }
        try {
            JSONObject json = new JSONObject();
            json.put("trials", 1000);

            MqttMessage message = new MqttMessage(json.toString().getBytes());
            message.setQos(0);
            mqttClient.publish("agv/" + focusedId + "/cmd/benchmark", message);
            System.out.println("[INFO] Benchmark (1000 random trials) started for AGV "
                    + focusedId + " - result in that C++ simulator's console.");

        } catch (MqttException e) {
            e.printStackTrace();
        }
    }

    private void publishObstacleCommand(double x, double y, String action, boolean isPerson) {
        if (mqttClient == null || !mqttClient.isConnected()) {
            System.err.println("[MQTT] Not connected to broker, command not sent.");
            return;
        }
        if (action.equals("add") && !isPerson) {
            globalStaticObstacles.add(new double[]{x, y});
        } else if (action.equals("clear")) {
            globalStaticObstacles.clear();
        }
        for (String id : agvStates.keySet()) {
            sendObstacleCommandTo(id, x, y, action, isPerson);
        }
    }

    private void sendObstacleCommandTo(String id, double x, double y, String action, boolean isPerson) {
        if (mqttClient == null || !mqttClient.isConnected()) {
            return;
        }
        try {
            JSONObject json = new JSONObject();
            json.put("action", action);
            if (action.equals("add")) {
                json.put("x", x);
                json.put("y", y);
                json.put("type", isPerson ? "person" : "static");
            }

            MqttMessage message = new MqttMessage(json.toString().getBytes());
            message.setQos(0);
            mqttClient.publish("agv/" + id + "/cmd/obstacle", message);

        } catch (MqttException e) {
            e.printStackTrace();
        }
    }
}