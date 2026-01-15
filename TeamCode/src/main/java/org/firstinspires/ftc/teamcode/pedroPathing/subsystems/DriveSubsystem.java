package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.RobotConstants;

import com.bylazar.telemetry.TelemetryManager;

public class DriveSubsystem {

    private final Follower follower;
    RobotConstants robotConstants;

    private final DcMotor leftFront, leftRear, rightFront, rightRear;

    private final Telemetry telemetry;
    private final TelemetryManager telemetryM;

    private boolean automatedDrive = false;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetryM = com.bylazar.telemetry.PanelsTelemetry.INSTANCE.getTelemetry();

        leftFront  = hardwareMap.get(DcMotor.class, RobotConstants.Hardware.LEFT_FRONT);
        leftRear   = hardwareMap.get(DcMotor.class, RobotConstants.Hardware.LEFT_REAR);
        rightFront = hardwareMap.get(DcMotor.class, RobotConstants.Hardware.RIGHT_FRONT);
        rightRear  = hardwareMap.get(DcMotor.class, RobotConstants.Hardware.RIGHT_REAR);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();
    }

    public void update() {
        follower.update();
        telemetryM.update();

        telemetry.addData("pose", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.update();
    }

    public void teleopDrive(double forward, double strafe, double turn) {
        automatedDrive = false;

        double lf = forward + strafe + turn;
        double lr = forward - strafe + turn;
        double rf = forward - strafe - turn;
        double rr = forward + strafe - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(lr),
                                Math.max(Math.abs(rf), Math.abs(rr)))));

        leftFront.setPower(lf / max);
        leftRear.setPower(lr / max);
        rightFront.setPower(rf / max);
        rightRear.setPower(rr / max);
    }

    public void stopMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void followPath(PathChain pathChain) {
        follower.followPath(pathChain);
        automatedDrive = true;
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public void cancelPathFollowing() {
        follower.breakFollowing();
        stopMotors();
        automatedDrive = false;
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public double getX() {
        return follower.getPose().getX();
    }

    public double getY() {
        return follower.getPose().getY();
    }

    public double getHeading() {
        return follower.getPose().getHeading();
    }

    public void setPoseEstimate(Pose p) {
        follower.setStartingPose(p);
        follower.update();
    }

    public boolean isAutomatedDrive() {
        return automatedDrive;
    }
}
