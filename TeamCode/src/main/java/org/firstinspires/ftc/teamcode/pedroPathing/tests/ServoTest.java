package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest extends OpMode {

    public Servo testServo;

    public void init() {
        testServo = hardwareMap.get(Servo.class, "testServo");

    }

    public void loop() {
        testServo.setPosition(1);

    }
}
