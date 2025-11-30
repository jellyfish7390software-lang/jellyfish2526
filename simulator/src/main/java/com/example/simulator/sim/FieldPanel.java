package com.example.simulator.sim;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.image.*;
import javax.imageio.ImageIO;
import java.io.*;
import java.util.Objects;

public class FieldPanel extends JPanel {
    private BufferedImage fieldImage;
    private Robot robot;
    public MecanumKinematics kinematics = new MecanumKinematics(1, 135, 1.8898, 16.5, 16.5);

    public static boolean paused = false;

    private Timer simTimer;
    private Timer autoStopTimer;

    public FieldPanel(Robot robot) {
        try {
            BufferedImage img = ImageIO.read(Objects.requireNonNull(getClass().getClassLoader().getResource("field.png")));

            int size = img.getWidth();
            fieldImage = new BufferedImage(size, size, img.getType());

            Graphics2D g2d = fieldImage.createGraphics();
            g2d.translate(size, 0);
            g2d.rotate(Math.toRadians(90));
            g2d.drawImage(img, 0, 0, null);
            g2d.dispose();

        } catch (IOException e) {
            e.printStackTrace();
        }
        this.robot = robot;

        autoStopTimer = new Timer(DriveSimulatorApp.MAX_TIME_MS, e -> {
            RobotState.setVelocities(new RobotVelocity(0, 0, 0));
        });
        autoStopTimer.setRepeats(false);
        autoStopTimer.start();

        simTimer = new Timer(16, e -> {
            if (!paused) {
                double dt = 0.016;
                RobotState.update(dt);
                robot.update(dt);
                repaint();
            }
        });
        simTimer.start();

        setupKeyBindings();
        setFocusable(true);
        requestFocusInWindow();
        setPreferredSize(new Dimension(1440, 1440));
        setMinimumSize(new Dimension(1440, 1440));
        setMaximumSize(new Dimension(1440, 1440));

    }

    private void setupKeyBindings() {
        InputMap inputMap = getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
        ActionMap actionMap = getActionMap();

        inputMap.put(KeyStroke.getKeyStroke("SPACE"), "togglePause");

        actionMap.put("togglePause", new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent e) {
                paused = !paused;

                if (paused) {
                    System.out.println("Paused");
                    autoStopTimer.stop();
                } else {
                    System.out.println("Resumed");
                    autoStopTimer.start();
                }
            }
        });
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        if (fieldImage != null) {
            g.drawImage(fieldImage, 0, 0, getWidth(), getHeight(), null);
        }

//        System.out.println("Panel size: " + getWidth() + " x " + getHeight());

        robot.draw(g, getWidth(), getHeight());
    }
}