package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Basic TeleOp")
public class BasicTeleop extends OpMode {

    // Declare motors
    //Initialize motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor intake;
    private DcMotor turret;

    @Override
    public void init() {
        // Initialize motors from the hardware map
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        intake = hardwareMap.get(DcMotor.class, "intake");
        turret = hardwareMap.get(DcMotor.class, "turret");

        // Reverse one side if necessary
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection((DcMotor.Direction.REVERSE));

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
    }

    @Override
    public void loop() {

        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        double intakePower  = -gamepad1.left_trigger;
        double turretPower = gamepad1.right_stick_y;

        // Mecanum drive calculations
        double flPower = (drive + strafe - turn);
        double frPower = (drive - strafe - turn);
        double blPower = (drive - strafe + turn);
        double brPower = (drive + strafe + turn);

        // Set motor power
        leftFront.setPower(flPower);
        rightFront.setPower(frPower);
        leftRear.setPower(blPower);
        rightRear.setPower(brPower);
        intake.setPower(intakePower);
        turret.setPower(turretPower);


        // Telemetry for debugging
        telemetry.addData("FL", flPower);
        telemetry.addData("FR", frPower);
        telemetry.addData("BL", blPower);
        telemetry.addData("BR", brPower);
        telemetry.update();

    }
}
