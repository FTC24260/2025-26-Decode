package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Spindex Position Test")
public class SpindexPositionTest extends OpMode {

    private Servo leftIndex, rightIndex;

    // Spindex positions (RB / LB)
    private final double[] positions = {0.2311, 0.3589, 0.4756};
    //0.2311 | 0.3589 | 0.4756 | INTAKE
    //0.1756 | 0.2956 | 0.4216 | SHOOT
    private int currentIndex = 0;

    // A-button toggle positions

    // Edge detection
    private boolean prevRB = false;
    private boolean prevLB = false;
    private boolean prevA  = false;

    @Override
    public void init() {
        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");

        // Initial spindex position
        setSpindexPosition(currentIndex);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        double rt = gamepad1.right_trigger;
        double lt = gamepad1.left_trigger;
        boolean a  = gamepad1.a;

        /* =====================
           SPINDEX (RB / LB)
           ===================== */

        // Right bumper → forward
        if (rb && !prevRB) {
            currentIndex++;
            if (currentIndex >= positions.length) {
                currentIndex = 0;
            }
            setSpindexPosition(currentIndex);
        }

        // Left bumper → backward
        if (lb && !prevLB) {
            currentIndex--;
            if (currentIndex < 0) {
                currentIndex = positions.length - 1;
            }
            setSpindexPosition(currentIndex);
        }

        // Right trigger -> Rotate CW
        if (rt > 0.1){
            double pos = rightIndex.getPosition();
            rightIndex.setPosition(pos + 0.0001);
            leftIndex.setPosition(rightIndex.getPosition() + 0.004);
        }
        // Left trigger -> Rotate CCW
        if (lt > 0.1){
            double pos = rightIndex.getPosition();
            rightIndex.setPosition(pos - 0.0001);
            leftIndex.setPosition(rightIndex.getPosition() + 0.004);
        }


        prevRB = rb;
        prevLB = lb;

        /* =====================
           A BUTTON TOGGLE
           ===================== */

        telemetry.addData("Spindex Index", currentIndex);
        telemetry.addData("Left Servo Pos", leftIndex.getPosition());
        telemetry.update();
    }

    private void setSpindexPosition(int index) {
        double pos = positions[index];
        leftIndex.setPosition(pos + 0.004);
        rightIndex.setPosition(pos);
    }
}
