package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class ServoTest extends OpMode {

    public Servo testServo1, testServo2;

    public void init() {
        testServo1 = hardwareMap.get(Servo.class, "testServo1");
        testServo2 = hardwareMap.get(Servo.class, "testServo2");

    }

    public void loop() {
        testServo1.setPosition(1);
        testServo2.setPosition(0);

    }
}
