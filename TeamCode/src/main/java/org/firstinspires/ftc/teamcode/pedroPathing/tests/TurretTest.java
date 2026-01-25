package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;

//@TeleOp(name = "Turret Aim Goal", group = "Pedro Pathing")
public class TurretTest extends OpMode {

    private Follower follower;
    private DcMotor turret;

    private final int TURRET_MAX = 430;
    private final int TURRET_MIN = -530;
    private final double MAX_POWER = 0.6;
    private final double WRAP_POWER = 0.4;
    private final double Kp = 0.021;

    private final double goalX = 0;
    private final double goalY = 144;

    private int turretZero = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 0, Math.PI / 2));

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setDirection(DcMotor.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretZero = turret.getCurrentPosition();
    }

    @Override
    public void loop() {
        follower.update();

        Pose robotPose = follower.getPoseTracker().getPose();
        double dx = goalX - robotPose.getX();
        double dy = goalY - robotPose.getY();

        // Desired turret angle in robot frame
        double targetAngle = Math.atan2(dy, dx) - robotPose.getHeading();

        // Convert to turret ticks
        double ticksPerRadian = (TURRET_MAX - TURRET_MIN) / (2 * Math.PI);
        int targetTicks = turretZero + (int) (targetAngle * ticksPerRadian);

        // Clamp targetTicks within physical limits
        targetTicks = Math.max(TURRET_MIN, Math.min(TURRET_MAX, targetTicks));

        // Compute error and proportional power
        int error = targetTicks - turret.getCurrentPosition();
        double power = Kp * error;

        // Clamp power
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        // Stop turret if at physical limits
        if ((turret.getCurrentPosition() >= TURRET_MAX && power > 0) ||
                (turret.getCurrentPosition() <= TURRET_MIN && power < 0)) {
            power = 0;
        }

        turret.setPower(power);

        telemetry.addData("Robot Pose", "X=%.1f Y=%.1f Heading=%.1f", robotPose.getX(), robotPose.getY(), Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Turret Position", turret.getCurrentPosition());
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);
        telemetry.update();
    }
}