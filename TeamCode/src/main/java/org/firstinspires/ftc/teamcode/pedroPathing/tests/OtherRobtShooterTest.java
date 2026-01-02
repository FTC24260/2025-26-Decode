package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "Shooter Test")
public class OtherRobtShooterTest extends OpMode {

    // Declare motors
    private DcMotor shooter;


    @Override
    public void init() {
        // Initialize motors from the hardware map

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
    }

    @Override
    public void loop() {
//        double shooterPower = 0;
//        if (gamepad1.dpad_up) {
//            shooterPower += 0.1;
//        }
//        else if (gamepad1.dpad_down) {
//            shooterPower -= 0.1;
//        }
//
//        if (gamepad1.right_trigger > 0.1) {
//            shooter.setPower(shooterPower);
//        }
        if (gamepad1.left_trigger > 0.1) {
            double shooterPower = 0.95;
            shooter.setPower(-shooterPower);
        }




        // Telemetry for debugging

        telemetry.addData("Shooter power:", shooter.getPower());
        telemetry.update();

    }
}
