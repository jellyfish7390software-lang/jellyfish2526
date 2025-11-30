package com.example.simulator.sim;

import com.example.simulator.math.Bezier;
import com.example.simulator.math.Pose;
import com.example.simulator.math.PurePursuit;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

public class Robot {
    double x, y, heading;
    RobotController controller;
    List<Bezier> beziers = new ArrayList<>();
    public static Pose targetPoint;

    public Robot(double x, double y, double heading, RobotController controller) {
        RobotState.x = x;
        RobotState.y = y;
        RobotState.heading = heading;
        RobotState.velocityX = 0;
        RobotState.velocityY = 0;
        RobotState.angularVelocity = 0;

        this.x = RobotState.x;
        this.y = RobotState.y;
        this.heading = RobotState.heading;
        this.controller = controller;
    }

    public void update(double dt) {
        controller.update(dt);
    }
    public void addBezier(Bezier curve) {beziers.add(curve);}

    public void draw(Graphics g, int panelWidth, int panelHeight) {
        this.x = RobotState.x;
        this.y = RobotState.y;
        this.heading = RobotState.heading;

        int size = 60;

        double scale = 10.0; // 10 pixels = 1 inch
        int centerX = panelWidth / 2;
        int centerY = panelHeight / 2;

        int drawX = (int)(centerX + (x/2) * scale - size / 2);
        int drawY = (int)(centerY - (y/2) * scale - size / 2);

        Graphics2D g2d = (Graphics2D) g;

        Rectangle2D rect = new Rectangle2D.Double(drawX, drawY, size, size);

        AffineTransform tx = new AffineTransform();
        tx.rotate(-heading, drawX + size / 2.0, drawY + size / 2.0);

        Shape rotated = tx.createTransformedShape(rect);

        g2d.setColor(Color.cyan);
        g2d.fill(rotated);
        g2d.setColor(Color.BLACK);
        g2d.draw(rotated);

        g2d.setStroke(new BasicStroke(2));
        g2d.drawLine(
                drawX + size / 2,
                drawY + size / 2,
                (int) (drawX + size / 2 + (size / 2.0) * Math.cos(-heading)),
                (int) (drawY + size / 2 + (size / 2.0) * Math.sin(-heading))
        );

        if (!beziers.isEmpty()) {
            int INC_AMOUNT = 40;

            int originX = centerX*2;
            int originY = centerY*2;

            for (Bezier curve : beziers) {
                g2d.setColor(new Color(0xfa98ce));

                List<Double> xPoints = new ArrayList<>();
                List<Double> yPoints = new ArrayList<>();

                for (int i = 0; i <= INC_AMOUNT; i++) {
                    double t = i / (double) INC_AMOUNT;
                    Pose pose = curve.solve(t);

                    if (pose != null) {
                        double screenX = originX + pose.x * scale;
                        double screenY = originY - pose.y * scale; // flip Y axis

//                        System.out.printf("t=%.2f, pose=(%.2f, %.2f)%n", t, screenX, screenY);

                        xPoints.add(screenX);
                        yPoints.add(screenY);
                    }
                }
                g2d.setTransform(new AffineTransform());  // Reset transform once before drawing
                g2d.setStroke(new BasicStroke(5));

                for (int i = 0; i < xPoints.size() - 1; i++) {
                    int x1 = (int) Math.round(xPoints.get(i));
                    int y1 = (int) Math.round(yPoints.get(i));
                    int x2 = (int) Math.round(xPoints.get(i + 1));
                    int y2 = (int) Math.round(yPoints.get(i + 1));

                    g2d.drawLine(x1, y1, x2, y2);
                }
            }
        }

        int originX = 800;
        int originY = 745;

        g2d.setTransform(new AffineTransform());  // Reset transform once before drawing
        g2d.setColor(new Color(0xF1AA00));
        g2d.setStroke(new BasicStroke(
                20f, // line thickness
                BasicStroke.CAP_ROUND, // rounded line end
                BasicStroke.JOIN_MITER // how line joins are drawn (irrelevant for a single line)
        ));

        int x = (int) (originX + targetPoint.x * scale);
        int y = (int) (originY - targetPoint.y * scale);

        g2d.drawLine(x,y,x,y);

        int radiusPixels = (int) (PurePursuit.searchRad * scale);
        int robotCenterX = drawX*2 + size;
        int robotCenterY = drawY*2 + size;

        g2d.setStroke(new BasicStroke(4));
        g2d.drawOval(
                robotCenterX - radiusPixels,
                robotCenterY - radiusPixels,
                radiusPixels * 2,
                radiusPixels * 2
        );

    }
}