package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class ServoTest extends OpMode {

    public Servo testServo;

    public void init() {
        testServo = hardwareMap.get(Servo.class, "flicker");

    }

    public void loop() {
        testServo.setPosition(0.54);

    }
}
