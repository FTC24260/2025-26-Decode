package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.RobotConstants;

public class IntakeSubsystem {

    private final DcMotor intakeMotor;
    private double targetPower = 0.0;
    private double currentPower = 0.0;
    private double intakePower = -1.0;
    private double outtakePower = 1.0;
    private double stopPower = 0.0;

    private static final double RAMP_RATE = 0.05; // smooth power ramping

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, RobotConstants.Hardware.INTAKE_MOTOR);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0.0);
    }

    public void intake() { targetPower = intakePower; }      // run forward
    public void outtake() { targetPower = outtakePower; }    // run backward
    public void stop() { targetPower = stopPower; }        // stop



    public void update() {
        // Ramp smoothly towards target power
        currentPower += clamp(targetPower - currentPower, -RAMP_RATE, RAMP_RATE);
        intakeMotor.setPower(currentPower);
    }


    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }


    public double getTargetPower() { return targetPower; }
    public double getCurrentPower() { return currentPower; }
}
