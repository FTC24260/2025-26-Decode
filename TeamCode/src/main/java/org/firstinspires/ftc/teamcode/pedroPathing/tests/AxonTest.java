package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Axon Servo Test")
public class AxonTest extends OpMode {

    private Servo flicker;
    private final double flickerUp = 0.333 ;
    private final double flickerDown = 0.575;
    @Override
    public void init() {
        // Name must match the Robot Configuration on the Driver Hub
        flicker = hardwareMap.get(Servo.class, "flicker");
    }

    @Override
    public void loop() {
        // Positions range from 0.0 to 1.0 (maps to your configured servo angle range)
        if (gamepad1.a) {
            flicker.setPosition(flickerUp);  // Minimum position
        } else if (gamepad1.y) {
            flicker.setPosition(flickerDown);  // Maximum position
        }

        telemetry.addData("Servo Position (commanded)", flicker.getPosition());
        telemetry.update();
    }
}