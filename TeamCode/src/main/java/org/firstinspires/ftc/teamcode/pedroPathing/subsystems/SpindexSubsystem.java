package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.RobotConstants.Ball;

public class SpindexSubsystem {

    private final Servo spindexLeft;
    private final Servo spindexRight;
    private final ColorSensor colorSensor;

    private static final int TOTAL_SLOTS = 6;

    private int currentSlot = 1;  // Start at first intake slot
    private int targetSlot = 1;   // Desired slot
    private Ball.Color[] slotColors = new Ball.Color[TOTAL_SLOTS];

    public SpindexSubsystem(HardwareMap hardwareMap) {
        spindexLeft = hardwareMap.get(Servo.class, RobotConstants.Hardware.SPINDEX_LEFT);
        spindexRight = hardwareMap.get(Servo.class, RobotConstants.Hardware.SPINDEX_RIGHT);
        colorSensor = hardwareMap.get(ColorSensor.class, RobotConstants.Hardware.COLOR_SENSOR);

        spindexLeft.setDirection(Servo.Direction.FORWARD);
        spindexRight.setDirection(Servo.Direction.REVERSE);

        // Initialize all slots as UNKNOWN
        for (int i = 0; i < TOTAL_SLOTS; i++) slotColors[i] = Ball.Color.UNKNOWN;

        // Move to first intake position
        moveToSlot(targetSlot);
    }

    // -------------------------
    // High-level commands
    // -------------------------

    /** Automatically detect ball and advance to next intake slot if needed */
    public void intakeBallIfDetected() {
        // Check intake positions only: 1, 3, 5
        int intakeIndex = getNextEmptyIntakeSlot();
        if (intakeIndex == -1) return; // All intake slots full

        Ball.Color detected = detectBallColor();
        if (detected != Ball.Color.UNKNOWN) {
            slotColors[intakeIndex] = detected;
            targetSlot = intakeIndex;
            moveToSlot(targetSlot);

            // If all intake slots now full, rotate to first shooting position
            if (areAllIntakeSlotsFull()) {
                targetSlot = 0; // first shoot slot
                moveToSlot(targetSlot);
            }
        }
    }

    /** Move directly to next slot for shooting or manual control */
    public void setSlot(int slot) {
        targetSlot = slot % TOTAL_SLOTS;
        moveToSlot(targetSlot);
    }

    /** Move spindex to home (first intake position) */
    public void home() {
        targetSlot = 1;
        moveToSlot(targetSlot);
    }

    // -------------------------
    // Helpers
    // -------------------------

    /** Returns the index of the next empty intake slot (1, 3, 5) */
    private int getNextEmptyIntakeSlot() {
        int[] intakeSlots = {1, 3, 5};
        for (int slot : intakeSlots) {
            if (slotColors[slot] == Ball.Color.UNKNOWN) return slot;
        }
        return -1; // all full
    }

    /** Checks if all intake slots are full */
    private boolean areAllIntakeSlotsFull() {
        return getNextEmptyIntakeSlot() == -1;
    }

    /** Detects the color of a ball at the sensor */
    private Ball.Color detectBallColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();

        if (red + green < 10) return Ball.Color.UNKNOWN; // no ball detected
        return (green > red) ? Ball.Color.GREEN : Ball.Color.PURPLE;
    }

    /** Moves the servo to a slot (full-speed) */
    private void moveToSlot(int slot) {
        double servoPosition = slot * RobotConstants.Spindex.SLOT_ANGLE_DEG / 180.0;
        spindexLeft.setPosition(servoPosition);
        spindexRight.setPosition(servoPosition);
        currentSlot = slot;
    }

    // -------------------------
    // Accessors
    // -------------------------
    public int getCurrentSlot() { return currentSlot; }
    public Ball.Color getSlotColor(int slot) { return slotColors[slot % TOTAL_SLOTS]; }
}
