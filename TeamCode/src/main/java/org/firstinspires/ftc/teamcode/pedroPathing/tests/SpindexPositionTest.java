package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Spindex Position Test")
public class SpindexPositionTest extends OpMode {

    private Servo leftIndex, rightIndex, flicker;
    private DcMotor intake, shooter;

    // Spindex positions (RB / LB)
    private final double[] positions = {0.34, 0.603, 1.0};
    private int currentIndex = 0;

    // A-button toggle positions
    private final double[] aPositions = {0.2, 0.46, 0.71};
    private int aIndex = -1; // start so first press goes to 0.44

    // Edge detection
    private boolean prevRB = false;
    private boolean prevLB = false;
    private boolean prevA  = false;

    @Override
    public void init() {
        leftIndex = hardwareMap.get(Servo.class, "leftIndex");
        rightIndex = hardwareMap.get(Servo.class, "rightIndex");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flicker = hardwareMap.get(Servo.class, "flicker");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        // Initial spindex position
        setSpindexPosition(currentIndex);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
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

        prevRB = rb;
        prevLB = lb;

        /* =====================
           A BUTTON TOGGLE
           ===================== */

        if (a && !prevA) {
            aIndex++;
            if (aIndex >= aPositions.length) {
                aIndex = 0;
            }

            leftIndex.setPosition(aPositions[aIndex]);
            rightIndex.setPosition(1.0 - leftIndex.getPosition());
        }

        prevA = a;

        /* =====================
           INTAKE
           ===================== */

        if (gamepad1.right_trigger > 0.1) {
            intake.setPower(-1);
        }
        else if (gamepad1.left_trigger > 0.1) {
            intake.setPower(1);
        }
        else {
            intake.setPower(0);
        }

        /* =====================
           FLICKER
           ===================== */

        if (gamepad1.start) {          // flicker down
            flicker.setPosition(0.7);
        }
        else if (gamepad1.back) {      // flicker up
            flicker.setPosition(0.5);
        }

        /* =====================
           Shooter
           ===================== */

        if (gamepad1.right_stick_button) { shooter.setPower(1); }
        else { shooter.setPower(0);
        }

        /* =====================
           TELEMETRY
           ===================== */

        telemetry.addData("Spindex Index", currentIndex);
        telemetry.addData("A Index", aIndex);
        telemetry.addData("Left Servo Pos", leftIndex.getPosition());
        telemetry.addData("Shooter Speed", shooter.getPower());
        telemetry.update();
    }

    private void setSpindexPosition(int index) {
        double pos = positions[index];
        leftIndex.setPosition(pos);
        rightIndex.setPosition(1.0 - pos);
    }
}
