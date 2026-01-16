package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.RobotConstants;

public class TurretSubsystem {

    private final DcMotor turret;
    private final Limelight3A limelight;

    private boolean enabled = true;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotor.class, RobotConstants.Hardware.TURRET_MOTOR);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, RobotConstants.Hardware.LIMELIGHT_NAME);
        limelight.start();
    }

    public void update() {
        if (!enabled) {
            turret.setPower(0);
            return;
        }

        LLResult result = null;

        for (int pipeline = 0; pipeline <= 2; pipeline++) {
            limelight.pipelineSwitch(pipeline);
            try { Thread.sleep(40); } catch (Exception ignored) {}
            LLResult temp = limelight.getLatestResult();
            if (temp != null && temp.isValid()) {
                result = temp;
                break;
            }
        }

        double power = 0;

        if (result != null && result.isValid()) {
            double tx = result.getTx();
            if (Math.abs(tx) > 1.0) {
                power = tx * RobotConstants.Turret.KP;
                power = Math.max(-0.4, Math.min(0.4, power));
            }
        }

        turret.setPower(RobotConstants.Turret.IF_REVERSED ? -power : power);
    }

    public boolean hasTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public void disable() {
        enabled = false;
        turret.setPower(0);
    }

    public void enable() {
        enabled = true;
    }

    public void stop() {
        turret.setPower(0);
        limelight.stop();
    }
}
