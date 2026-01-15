package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.RobotConstants;

public class ShooterSubsystem {

    private DcMotorEx shooterL, shooterR;
    private DcMotorEx turret;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterL = hardwareMap.get(DcMotorEx.class, RobotConstants.Hardware.SHOOTER_L);
        shooterR = hardwareMap.get(DcMotorEx.class, RobotConstants.Hardware.SHOOTER_R);
        turret   = hardwareMap.get(DcMotorEx.class, RobotConstants.Hardware.TURRET_MOTOR);

        shooterR.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(
                RobotConstants.Turret.IF_REVERSED
                        ? DcMotor.Direction.REVERSE
                        : DcMotor.Direction.FORWARD
        );
    }

    public void setTargetVelocity(double targetVelocity) {
        double power = clamp(feedforward(targetVelocity), 0, 1);
        shooterL.setPower(power);
        shooterR.setPower(power);
    }

    public void setPowerFromDistance(double distanceInches) {
        double power = interpolatePower(distanceInches);
        shooterL.setPower(power);
        shooterR.setPower(power);
    }

    public void setTurretPower(double power) {
        turret.setPower(clamp(power, -1, 1));
    }

    public void stopShooter() {
        shooterL.setPower(0);
        shooterR.setPower(0);
    }

    public void stopTurret() {
        turret.setPower(0);
    }

    private double interpolatePower(double distance) {
        double[][] table = RobotConstants.Shooter.POWER_TABLE;

        if (distance <= table[0][0]) return table[0][1];
        if (distance >= table[table.length - 1][0]) return table[table.length - 1][1];

        for (int i = 0; i < table.length - 1; i++) {
            double d0 = table[i][0];
            double p0 = table[i][1];
            double d1 = table[i + 1][0];
            double p1 = table[i + 1][1];

            if (distance >= d0 && distance <= d1) {
                double t = (distance - d0) / (d1 - d0);
                return p0 + t * (p1 - p0);
            }
        }

        return 0;
    }

    private double feedforward(double targetVel) {
        return RobotConstants.Shooter.KS * Math.signum(targetVel)
                + RobotConstants.Shooter.KV * targetVel;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
