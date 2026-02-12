package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class ServoTest extends OpMode {

    public Servo flicker;

    public void init() {
        flicker = hardwareMap.get(Servo.class, "flicker");


    }

    public void loop() {
        flicker.setPosition(0.575);
    }
}
