package agv;

import javax.swing.JPanel;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

public class MapPanel extends JPanel {

    private final Map<String, AgvState> agvStates;
    private String focusedAgvId = "1";

    private List<double[]> redWaypoints;
    private List<double[]> greenWaypoints;
    private String selectedRoute = "red";

    private List<double[]> markers = new ArrayList<>();
    private final List<double[]> obstacles = new ArrayList<>();

    private double scale = 1.0;
    private double originX = 0.0;
    private double originY = 0.0;

    private static final int MARGIN = 50;
    private static final double YELLOW_RADIUS = 1.5;
    private static final double RED_RADIUS = 0.6;

    private static final Color BG_COLOR = new Color(20, 24, 32);
    private static final Color GRID_COLOR = new Color(32, 38, 50);
    private static final Color RED_ROUTE_COLOR = new Color(235, 99, 99);
    private static final Color GREEN_ROUTE_COLOR = new Color(84, 199, 140);
    private static final Color PERSON_OBSTACLE_COLOR = new Color(230, 158, 46);

    public interface ObstacleClickListener {
        void onClick(double worldX, double worldY, boolean isPerson);
    }

    private ObstacleClickListener obstacleClickListener;

    public MapPanel(Map<String, AgvState> agvStates, List<double[]> redWaypoints, List<double[]> greenWaypoints) {
        this.agvStates = agvStates;
        this.redWaypoints = redWaypoints;
        this.greenWaypoints = greenWaypoints;
        setBackground(BG_COLOR);
        setPreferredSize(new Dimension(800, 600));

        addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                double worldX = toWorldX(e.getX());
                double worldY = toWorldY(e.getY());
                boolean isPerson = e.isShiftDown();
                addObstacle(worldX, worldY, isPerson);
                if (obstacleClickListener != null) {
                    obstacleClickListener.onClick(worldX, worldY, isPerson);
                }
            }
        });
    }

    public void setObstacleClickListener(ObstacleClickListener listener) {
        this.obstacleClickListener = listener;
    }

    public void setFocusedAgvId(String id) {
        this.focusedAgvId = id;
        repaint();
    }

    public void setRouteWaypoints(List<double[]> newRedWaypoints, List<double[]> newGreenWaypoints) {
        this.redWaypoints = newRedWaypoints;
        this.greenWaypoints = newGreenWaypoints;
        repaint();
    }

    public void setSelectedRoute(String route) {
        this.selectedRoute = route;
        this.markers = new ArrayList<>();
        repaint();
    }

    public void setMarkers(List<double[]> newMarkers) {
        this.markers = newMarkers;
        repaint();
    }

    public void addObstacle(double worldX, double worldY, boolean isPerson) {
        obstacles.add(new double[]{worldX, worldY, isPerson ? 1.0 : 0.0});
        repaint();
    }

    public void setObstacles(List<double[]> newObstacles) {
        obstacles.clear();
        obstacles.addAll(newObstacles);
        repaint();
    }

    public void clearObstacles() {
        obstacles.clear();
        repaint();
    }

    private int toPixelX(double worldX) {
        return (int) Math.round(originX + worldX * scale);
    }

    private int toPixelY(double worldY) {
        return (int) Math.round(originY - worldY * scale);
    }

    private double toWorldX(int pixelX) {
        return (pixelX - originX) / scale;
    }

    private double toWorldY(int pixelY) {
        return (originY - pixelY) / scale;
    }

    private void computeTransform() {
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;

        for (List<double[]> wps : List.of(redWaypoints, greenWaypoints)) {
            for (double[] p : wps) {
                minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
                minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
            }
        }
        for (AgvState st : agvStates.values()) {
            minX = Math.min(minX, st.getX()); maxX = Math.max(maxX, st.getX());
            minY = Math.min(minY, st.getY()); maxY = Math.max(maxY, st.getY());
        }

        double worldW = Math.max(maxX - minX, 1.0);
        double worldH = Math.max(maxY - minY, 1.0);

        int availW = Math.max(getWidth() - 2 * MARGIN, 50);
        int availH = Math.max(getHeight() - 2 * MARGIN, 50);

        scale = Math.min(availW / worldW, availH / worldH);

        double usedW = worldW * scale;
        double usedH = worldH * scale;
        double extraX = (availW - usedW) / 2.0;
        double extraY = (availH - usedH) / 2.0;

        originX = MARGIN + extraX - minX * scale;
        originY = getHeight() - MARGIN - extraY + minY * scale;
    }

    private double chooseGridStep(double worldSpanMeters) {
        double[] candidates = { 1, 2, 5, 10, 20, 25, 50, 100, 200, 500 };
        for (double c : candidates) {
            if (worldSpanMeters / c <= 12) return c;
        }
        return 500;
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        computeTransform();

        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        drawGrid(g2);
        drawRoute(g2, redWaypoints, RED_ROUTE_COLOR, selectedRoute.equals("red"));
        drawRoute(g2, greenWaypoints, GREEN_ROUTE_COLOR, selectedRoute.equals("green"));
        drawPointLabels(g2);
        drawMarkersDim(g2);
        drawSensorZones(g2);
        drawObstacles(g2);
        drawAllAgvs(g2);
        drawNearestMarkerHalo(g2);
        drawLegend(g2);
    }

    private void drawGrid(Graphics2D g2) {
        double worldSpan = Math.max(getWidth(), getHeight()) / Math.max(scale, 0.0001);
        double step = chooseGridStep(worldSpan);

        g2.setColor(GRID_COLOR);

        double startX = Math.floor(toWorldX(0) / step) * step;
        for (double wx = startX; toPixelX(wx) <= getWidth() + (int) step; wx += step) {
            int px = toPixelX(wx);
            g2.drawLine(px, 0, px, getHeight());
        }
        double startY = Math.floor(toWorldY(getHeight()) / step) * step;
        for (double wy = startY; toPixelY(wy) >= -(int) step; wy += step) {
            int py = toPixelY(wy);
            g2.drawLine(0, py, getWidth(), py);
        }
    }

    private void drawPointLabels(Graphics2D g2) {
        g2.setFont(g2.getFont().deriveFont(Font.BOLD, 11f));

        double[] putAway = redWaypoints.get(0);
        drawTagLabel(g2, "PUT AWAY", putAway[0], putAway[1], new Color(230, 230, 235));

        double[] redPick = redWaypoints.get(redWaypoints.size() - 1);
        drawTagLabel(g2, "PICK", redPick[0], redPick[1], RED_ROUTE_COLOR.brighter());

        double[] greenPick = greenWaypoints.get(greenWaypoints.size() - 1);
        drawTagLabel(g2, "PICK", greenPick[0], greenPick[1], GREEN_ROUTE_COLOR.brighter());
    }

    private void drawTagLabel(Graphics2D g2, String text, double worldX, double worldY, Color textColor) {
        int px = toPixelX(worldX);
        int py = toPixelY(worldY) - 16;

        FontMetrics fm = g2.getFontMetrics();
        int tw = fm.stringWidth(text);
        int padding = 5;

        g2.setColor(new Color(0, 0, 0, 170));
        g2.fillRoundRect(px - tw / 2 - padding, py - fm.getAscent(), tw + padding * 2, fm.getHeight(), 6, 6);
        g2.setColor(textColor);
        g2.drawString(text, px - tw / 2, py);
    }

    private void drawRoute(Graphics2D g2, List<double[]> waypoints, Color baseColor, boolean isActive) {
        Color lineColor = isActive ? baseColor : new Color(baseColor.getRed(), baseColor.getGreen(), baseColor.getBlue(), 90);
        float strokeWidth = isActive ? 3f : 2f;

        if (waypoints.size() > 1) {
            g2.setColor(lineColor);
            g2.setStroke(new BasicStroke(strokeWidth, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND,
                    1f, new float[]{7f, 7f}, 0f));
            for (int i = 0; i < waypoints.size() - 1; i++) {
                int x1 = toPixelX(waypoints.get(i)[0]);
                int y1 = toPixelY(waypoints.get(i)[1]);
                int x2 = toPixelX(waypoints.get(i + 1)[0]);
                int y2 = toPixelY(waypoints.get(i + 1)[1]);
                g2.drawLine(x1, y1, x2, y2);
            }
            g2.setStroke(new BasicStroke(1f));
        }

        Color dotColor = isActive ? baseColor : new Color(baseColor.getRed(), baseColor.getGreen(), baseColor.getBlue(), 110);
        g2.setColor(dotColor);
        for (double[] wp : waypoints) {
            int px = toPixelX(wp[0]);
            int py = toPixelY(wp[1]);
            g2.fillOval(px - 6, py - 6, 12, 12);
            g2.setColor(new Color(230, 235, 245, isActive ? 255 : 140));
            g2.fillOval(px - 2, py - 2, 4, 4);
            g2.setColor(dotColor);
        }
    }

    private void drawMarkersDim(Graphics2D g2) {
        if (markers.isEmpty()) return;
        AgvState focused = agvStates.get(focusedAgvId);
        int nearest = (focused != null) ? focused.getNearestMarker() : -1;

        g2.setColor(new Color(110, 118, 132));
        for (int i = 0; i < markers.size(); i++) {
            if (i == nearest) continue;
            int px = toPixelX(markers.get(i)[0]);
            int py = toPixelY(markers.get(i)[1]);
            g2.fillOval(px - 2, py - 2, 4, 4);
        }
    }

    private void drawNearestMarkerHalo(Graphics2D g2) {
        AgvState focused = agvStates.get(focusedAgvId);
        if (focused == null) return;
        int nearest = focused.getNearestMarker();
        if (markers.isEmpty() || nearest < 0 || nearest >= markers.size()) return;

        int px = toPixelX(markers.get(nearest)[0]);
        int py = toPixelY(markers.get(nearest)[1]);

        g2.setStroke(new BasicStroke(2.5f));
        g2.setColor(new Color(0, 225, 245));
        g2.drawOval(px - 14, py - 14, 28, 28);
        g2.setStroke(new BasicStroke(1f));
    }

    private void drawSensorZones(Graphics2D g2) {
        for (Map.Entry<String, AgvState> entry : agvStates.entrySet()) {
            boolean isFocused = entry.getKey().equals(focusedAgvId);
            drawSensorZoneFor(g2, entry.getValue(), isFocused);
        }
    }

    private void drawSensorZoneFor(Graphics2D g2, AgvState state, boolean isFocused) {
        int px = toPixelX(state.getX());
        int py = toPixelY(state.getY());

        int yellowPx = Math.max((int) Math.round(YELLOW_RADIUS * scale), 5);
        int redPx = Math.max((int) Math.round(RED_RADIUS * scale), 3);

        int yellowAlpha = isFocused ? 90 : 45;
        int redAlpha = isFocused ? 110 : 55;

        g2.setColor(new Color(255, 205, 0, yellowAlpha));
        g2.fillOval(px - yellowPx, py - yellowPx, yellowPx * 2, yellowPx * 2);

        g2.setColor(new Color(255, 70, 70, redAlpha));
        g2.fillOval(px - redPx, py - redPx, redPx * 2, redPx * 2);
    }

    private void drawObstacles(Graphics2D g2) {
        for (double[] obs : obstacles) {
            int px = toPixelX(obs[0]);
            int py = toPixelY(obs[1]);
            boolean isPerson = obs.length > 2 && obs[2] > 0.5;

            if (isPerson) {
                g2.setColor(PERSON_OBSTACLE_COLOR);
                g2.fillOval(px - 7, py - 7, 14, 14);
                g2.setColor(PERSON_OBSTACLE_COLOR.darker());
                g2.drawOval(px - 7, py - 7, 14, 14);
            } else {
                g2.setColor(new Color(255, 70, 70));
                int s = 10;
                g2.fillRect(px - s / 2, py - s / 2, s, s);
            }
        }
    }

    private void drawAllAgvs(Graphics2D g2) {
        Map<String, AgvState> sorted = new TreeMap<>(agvStates);
        for (Map.Entry<String, AgvState> entry : sorted.entrySet()) {
            boolean isFocused = entry.getKey().equals(focusedAgvId);
            drawSingleAgv(g2, entry.getValue(), entry.getKey(), isFocused);
        }
    }

    private void drawSingleAgv(Graphics2D g2, AgvState agvState, String id, boolean isFocused) {
        int px = toPixelX(agvState.getX());
        int py = toPixelY(agvState.getY());

        Color color;
        switch (agvState.getState()) {
            case "OBSTACLE_STOP": color = new Color(235, 60, 60); break;
            case "OBSTACLE_SLOW": color = new Color(250, 165, 40); break;
            case "NAVIGATING": color = new Color(50, 190, 90); break;
            case "MISSION_COMPLETE": color = new Color(150, 155, 165); break;
            default: color = new Color(130, 135, 145);
        }
        if (!isFocused) {
            color = new Color(color.getRed(), color.getGreen(), color.getBlue(), 160);
        }

        Graphics2D g2c = (Graphics2D) g2.create();
        g2c.translate(px, py);
        g2c.rotate(Math.toRadians(-agvState.getHeading()));

        int size = 16;
        Polygon triangle = new Polygon();
        triangle.addPoint(size, 0);
        triangle.addPoint(-size / 2, size / 2);
        triangle.addPoint(-size / 2, -size / 2);

        g2c.setColor(color);
        g2c.fillPolygon(triangle);
        g2c.setColor(isFocused ? Color.WHITE : color.darker());
        g2c.setStroke(new BasicStroke(isFocused ? 2f : 1f));
        g2c.drawPolygon(triangle);
        g2c.dispose();

        g2.setFont(g2.getFont().deriveFont(Font.BOLD, 10f));
        g2.setColor(new Color(0, 0, 0, 170));
        g2.fillOval(px + 9, py - 17, 16, 16);
        g2.setColor(isFocused ? Color.WHITE : new Color(200, 200, 205));
        FontMetrics fm = g2.getFontMetrics();
        g2.drawString(id, px + 17 - fm.stringWidth(id) / 2, py - 6);
    }

    private void drawLegend(Graphics2D g2) {
        int boxX = 16;
        int boxY = 16;
        int boxW = 320;
        int boxH = 280;

        g2.setColor(new Color(14, 17, 23, 240));
        g2.fillRoundRect(boxX, boxY, boxW, boxH, 14, 14);
        g2.setColor(new Color(58, 64, 78));
        g2.drawRoundRect(boxX, boxY, boxW, boxH, 14, 14);

        g2.setFont(g2.getFont().deriveFont(12f));
        int textX = boxX + 36;
        int rowY = boxY + 34;
        int rowH = 26;
        Color textColor = new Color(224, 226, 232);

        g2.setStroke(new BasicStroke(3.5f));
        g2.setColor(RED_ROUTE_COLOR);
        g2.drawLine(boxX + 8, rowY - 5, boxX + 24, rowY - 5);
        g2.setColor(textColor);
        g2.drawString("Red route", textX, rowY);
        rowY += rowH;

        g2.setColor(GREEN_ROUTE_COLOR);
        g2.drawLine(boxX + 8, rowY - 5, boxX + 24, rowY - 5);
        g2.setColor(textColor);
        g2.drawString("Green route", textX, rowY);
        g2.setStroke(new BasicStroke(1f));
        rowY += rowH;

        drawLegendSwatch(g2, boxX + 16, rowY - 9, GREEN_ROUTE_COLOR);
        g2.setColor(textColor);
        g2.drawString("AGV - moving", textX, rowY);
        rowY += rowH;

        drawLegendSwatch(g2, boxX + 16, rowY - 9, new Color(240, 165, 60));
        g2.setColor(textColor);
        g2.drawString("AGV - slowing down", textX, rowY);
        rowY += rowH;

        drawLegendSwatch(g2, boxX + 16, rowY - 9, RED_ROUTE_COLOR);
        g2.setColor(textColor);
        g2.drawString("AGV - stopped", textX, rowY);
        rowY += rowH;

        g2.setColor(RED_ROUTE_COLOR);
        g2.fillRect(boxX + 10, rowY - 17, 13, 13);
        g2.setColor(textColor);
        g2.drawString("Obstacle ", textX, rowY);
        rowY += rowH;

        g2.setColor(PERSON_OBSTACLE_COLOR);
        g2.fillOval(boxX + 9, rowY - 15, 14, 14);
        g2.setColor(textColor);
        g2.drawString("Dynamic obstacle ", textX, rowY);
        rowY += rowH;

        g2.setColor(new Color(110, 118, 132));
        g2.fillOval(boxX + 12, rowY - 13, 7, 7);
        g2.setColor(textColor);
        g2.drawString("Ceiling marker", textX, rowY);
        rowY += rowH;

        g2.setStroke(new BasicStroke(1.5f));
        g2.setColor(new Color(0, 225, 245));
        g2.drawOval(boxX + 7, rowY - 16, 11, 11);
        g2.setColor(textColor);
        g2.drawString("Marker the AGV is reading now", textX, rowY);
        rowY += rowH;

        g2.setColor(textColor);
        g2.drawString("Number - vehicle ID", textX, rowY);
    }

    private void drawLegendSwatch(Graphics2D g2, int cx, int cy, Color color) {
        g2.setColor(color);
        g2.fillOval(cx - 8, cy - 8, 16, 16);
    }
}