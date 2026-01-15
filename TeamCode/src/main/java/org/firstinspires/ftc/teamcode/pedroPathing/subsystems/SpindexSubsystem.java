package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.RobotConstants;

public class SpindexSubsystem {

    private Servo leftIndex, rightIndex, flicker;
    private ColorSensor colorSensor;

    private final double flickerUp = RobotConstants.Kicker.FIRE_POS;
    private final double flickerDown = RobotConstants.Kicker.REST_POS;

    private final double[] intakePositions = {RobotConstants.Spindex.INTAKE_1, RobotConstants.Spindex.INTAKE_2, RobotConstants.Spindex.INTAKE_3};
    private final double[] shootPositions  = {RobotConstants.Spindex.SHOOT_1, RobotConstants.Spindex.SHOOT_2, RobotConstants.Spindex.SHOOT_3};

    private final String[] slots = {"unknown", "unknown", "unknown"};
    private int currentIndex = 0;

    private long ignoreSensorUntil = 0;
    private static final int SENSOR_IGNORE_MS = RobotConstants.Spindex.SENSOR_IGNORE;

    private long initialIgnoreUntil = 0;
    private static final int INITIAL_IGNORE_MS = RobotConstants.Spindex.INITIAL_IGNORE;

    public SpindexSubsystem(HardwareMap hardwareMap) {
        leftIndex = hardwareMap.get(Servo.class, RobotConstants.Hardware.SPINDEX_LEFT);
        rightIndex = hardwareMap.get(Servo.class, RobotConstants.Hardware.SPINDEX_RIGHT);
        flicker = hardwareMap.get(Servo.class, RobotConstants.Hardware.KICKER);
        colorSensor = hardwareMap.get(ColorSensor.class, RobotConstants.Hardware.COLOR_SENSOR);

        setIntakePosition(0);
        flicker.setPosition(flickerDown);
        initialIgnoreUntil = System.currentTimeMillis() + INITIAL_IGNORE_MS;
    }

    public String updateSlots() {
        long now = System.currentTimeMillis();
        String detectedColor = detectColor();

        if (!detectedColor.equals("unknown")
                && now >= ignoreSensorUntil
                && now >= initialIgnoreUntil
                && currentIndex < slots.length) {

            slots[currentIndex] = detectedColor;
            currentIndex++;
            setIntakePosition(currentIndex);
            ignoreSensorUntil = now + SENSOR_IGNORE_MS;
        }

        return detectedColor;
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

    public void resetSlots() {
        for (int i = 0; i < slots.length; i++) slots[i] = "unknown";
        currentIndex = 0;
        setIntakePosition(0);
    }

    public String[] getSlots() {
        return slots.clone();
    }

    public int getCurrentIndex() {
        return currentIndex;
    }

    private String detectColor() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        if (g > RobotConstants.Ball.GREEN_THRESHOLD * r && g > RobotConstants.Ball.GREEN_THRESHOLD * b && g > 20) return "green";

        int maxRB = Math.max(r, b);
        int minRB = Math.min(r, b);
        if (maxRB > 40 && minRB >= 0.5 * maxRB && g < 0.7 * maxRB) return "purple";

        return "unknown";
    }
}
