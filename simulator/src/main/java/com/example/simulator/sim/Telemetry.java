package com.example.simulator.sim;

import javax.swing.*;
import java.awt.*;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;

public class Telemetry extends JPanel {
    private final JPanel contentPanel;
    private final Map<String, JLabel> labelMap = new LinkedHashMap<>();
    private final Map<String, Supplier<String>> supplierMap = new LinkedHashMap<>();

    public Telemetry() {
        setLayout(new BorderLayout());
        setPreferredSize(new Dimension(200, 0));

        JLabel title = new JLabel("Telemetry");
        title.setHorizontalAlignment(SwingConstants.CENTER);
        title.setFont(new Font("SansSerif", Font.BOLD, 16));
        title.setBorder(BorderFactory.createEmptyBorder(10, 0, 10, 0));
        add(title, BorderLayout.NORTH);

        contentPanel = new JPanel();
        contentPanel.setLayout(new BoxLayout(contentPanel, BoxLayout.Y_AXIS));
        JScrollPane scrollPane = new JScrollPane(contentPanel);
        add(scrollPane, BorderLayout.CENTER);
    }

    public void addLine(String line) {
        JLabel label = new JLabel(line);
        contentPanel.add(label);
    }


    // Simple API: direct value (captured at time of call)
    public void addData(String caption, Object value) {
        addData(caption, () -> value);  // Wrap in supplier internally
    }

    // Dynamic API: supplier for real-time updates
    public void addData(String caption, Supplier<?> valueSupplier) {
        Supplier<String> stringSupplier = () -> {
            Object value = valueSupplier.get();
            if (value instanceof Double) {
                return String.format("%.2f", (Double) value);
            } else if (value instanceof Float) {
                return String.format("%.2f", (Float) value);
            } else {
                return String.valueOf(value);
            }
        };

        JLabel label = new JLabel(caption + ": " + stringSupplier.get());
        contentPanel.add(label);
        labelMap.put(caption, label);
        supplierMap.put(caption, stringSupplier);
    }

    public void updateAll() {
        for (String caption : supplierMap.keySet()) {
            Supplier<String> supplier = supplierMap.get(caption);
            JLabel label = labelMap.get(caption);
            label.setText(caption + ": " + supplier.get());
        }
    }

    public void clear() {
        contentPanel.removeAll();
        contentPanel.revalidate();
        contentPanel.repaint();
        labelMap.clear();
        supplierMap.clear();
    }
}
