package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.RobotConstants;

public class SpindexSubsystem {

    private Servo leftIndex, rightIndex, flicker;
    private ColorSensor colorSensor;

    private final double[] intakePositions = {0.55, 0.813, 0.3};
    private final double[] shootPositions  = {0.68, 0.42, 0.17};
    private final String[] slots = {"unknown", "unknown", "unknown"};

    private final double flickerUp = 0.5;
    private final double flickerDown = 0.75;

    public SpindexSubsystem(HardwareMap hardwareMap) {
        leftIndex = hardwareMap.get(Servo.class, RobotConstants.Hardware.SPINDEX_LEFT);
        rightIndex = hardwareMap.get(Servo.class, RobotConstants.Hardware.SPINDEX_RIGHT);
        flicker = hardwareMap.get(Servo.class, "flicker");

        colorSensor = hardwareMap.get(ColorSensor.class, RobotConstants.Hardware.COLOR_SENSOR);

        setIntakePosition(0);
        flicker.setPosition(flickerDown);
    }

    public void updateSlots() {
        String color = detectColor();
        for (int i = 0; i < slots.length; i++) {
            if (slots[i].equals("unknown") && !color.equals("unknown")) {
                slots[i] = color;
                setIntakePosition(i + 1);
                break;
            }
        }
    }

    public void setIntakePosition(int index) {
        if (index >= intakePositions.length) index = intakePositions.length - 1;
        leftIndex.setPosition(intakePositions[index]);
        rightIndex.setPosition(1.0 - intakePositions[index]);
    }

    public void setShootPosition(int index) {
        if (index >= shootPositions.length) index = shootPositions.length - 1;
        leftIndex.setPosition(shootPositions[index]);
        rightIndex.setPosition(1.0 - shootPositions[index]);
    }

    public void flickUp() {
        flicker.setPosition(flickerUp);
    }

    public void flickDown() {
        flicker.setPosition(flickerDown);
    }

    public boolean anySlotLoaded() {
        for (String s : slots) {
            if (!s.equals("unknown")) return true;
        }
        return false;
    }

    public String[] getSlots() {
        return slots;
    }

    public void resetSlots() {
        for (int i = 0; i < slots.length; i++) {
            slots[i] = "unknown";
        }
        setIntakePosition(0);
    }

    public void clearSlot(int index) {
        if (index >= 0 && index < slots.length) {
            slots[index] = "unknown";
        }
    }

    private String detectColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        if (g > 1.2 * r && g > 1.2 * b && g > 20) return "green";

        int maxRB = Math.max(r, b);
        int minRB = Math.min(r, b);
        if (maxRB > 40 && minRB >= 0.5 * maxRB && g < 0.7 * maxRB) return "purple";

        return "unknown";
    }
}
