package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RGB Light Test")
public class RGBlight extends LinearOpMode {

    @Override
    public void runOpMode() {

        // ── Pick ONE of these two approaches based on your hardware config ──

        // Approach A: CRServo — power ranges from -1.0 to 1.0

        // Approach B: Standard Servo — position ranges from 0.0 to 1.0
        Servo rgbLight = hardwareMap.get(Servo.class, "rgbIndicator");
        // Servo prismServo = hardwareMap.get(Servo.class, "RGB light");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            //── Approach B: Standard Servo color control (uncomment if using Servo) ──

            if (gamepad1.a) {
                double colordetect = 0.5;
                rgbLight.setPosition(colordetect);  // ~1000µs
            }
            if (gamepad1.b){
                double colordetect = 0.6;
                rgbLight.setPosition(colordetect);  // ~1000µs

            }

        }
    }
}