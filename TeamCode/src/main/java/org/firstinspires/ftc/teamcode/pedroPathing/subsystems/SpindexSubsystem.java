package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class SpindexSubsystem {

    private final Servo leftIndex, rightIndex;
    private final double[] intakePositions = {0.34, 0.603, 1.0};
    private final double[] shootPositions = {0.73, 0.46, 0.20};

    private int currentIndex = 0;
    private double targetPosition = 0;
    private static final double POSITION_EPSILON = 0.02;

    private boolean sensorEnabled = true;

    public SpindexSubsystem(Servo leftIndex, Servo rightIndex) {
        this.leftIndex = leftIndex;
        this.rightIndex = rightIndex;
        setIntakePosition(0);
    }

    public boolean isSensorEnabled() {
        return sensorEnabled;
    }

    public int getCurrentIndex() {
        return currentIndex;
    }

    public void update() {
        // Unlock the sensor once servo reaches target
        if (!sensorEnabled) {
            double error = Math.abs(leftIndex.getPosition() - targetPosition);
            if (error < POSITION_EPSILON) {
                sensorEnabled = true;
            }
        }
    }

    public void onBallDetected() {
        if (!sensorEnabled || currentIndex >= intakePositions.length) return;

        currentIndex++;
        setIntakePosition(currentIndex);
        sensorEnabled = false; // lock sensor until servo moves
    }

    public void setIntakePosition(int index) {
        if (index >= intakePositions.length) index = intakePositions.length - 1;
        targetPosition = intakePositions[index];
        leftIndex.setPosition(intakePositions[index]);
        rightIndex.setPosition(1.0 - intakePositions[index]);
    }

    public void setShootPosition(int index) {
        if (index >= shootPositions.length) index = shootPositions.length - 1;
        leftIndex.setPosition(shootPositions[index]);
        rightIndex.setPosition(1.0 - shootPositions[index]);
    }

    public void reset() {
        currentIndex = 0;
        setIntakePosition(0);
        sensorEnabled = true;
    }
}
