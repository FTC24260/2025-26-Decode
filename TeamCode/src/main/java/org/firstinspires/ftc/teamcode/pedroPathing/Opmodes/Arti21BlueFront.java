package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@Autonomous(name = "21 Blue Front", group = "Autonomous")
public class Arti21BlueFront extends OpMode {
    private Follower follower;
    private Timer pathTimer;

    // ----------------- STATE MACHINE ENUM -------------------
    public enum PathState {
        PRELOAD_INTAKE2,
        SHOOT2,
        INTAKE_GATE1,
        SHOOT_GATE1,
        INTAKE_GATE2,
        SHOOT_GATE2,
        INTAKE_GATE3,
        SHOOT_GATE3,
        INTAKE1,
        SHOOT_INTAKE1,
        INTAKE3,
        SHOOT_INTAKE3,
        END
    }

    PathState pathState;

    // ----------------- DRIVE POINTS -------------------
    private final Pose startPose = new Pose(21, 124,Math.toRadians(144));
    private final Pose intake3 = new Pose(24, 84, Math.toRadians(180));
    private final Pose intake2 = new Pose(24, 60, Math.toRadians(180));
    private final Pose intake1 = new Pose(24, 84, Math.toRadians(180));
    private final Pose shoot = new Pose(60, 84, Math.toRadians(138));
    private final Pose intakeGate = new Pose(11, 61, Math.toRadians(150));

    // -------------------- CONTROL POINTS -------------------
    private final Pose intake2Control = new Pose(96, 60);
    private final Pose intake3Control = new Pose(62,29);
    private final Pose intakeGateControl = new Pose(52, 24);

    // -------------------- PATH CHAINS -----------------------
    public PathChain preshoot_Intake2;
    public PathChain shootIntake2;
    public PathChain gateIntake;
    public PathChain shootGate;
    public PathChain grabPickup1;
    public PathChain shootIntake1;
    public PathChain grabPickup3;
    public PathChain shootIntake3;

    // -------------------- CONSTRUCT PATHS ---------------------
    public void preStart() {
        preshoot_Intake2 = follower.pathBuilder()
                .addPath(
                new BezierCurve(
                   startPose,
                   intake2Control,
                   intake2
                )
        )
                .setLinearHeadingInterpolation(startPose.getHeading(), intake2.getHeading())
                .build();

        shootIntake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intake2,
                                shoot
                        )
                )
                .setLinearHeadingInterpolation(intake2.getHeading(), shoot.getHeading())
                .build();

        gateIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shoot,
                                intakeGateControl,
                                intakeGate
                        )
                )
                .setLinearHeadingInterpolation(shoot.getHeading(), intakeGate.getHeading())
                .build();

        shootGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakeGate,
                                shoot
                        )
                )
                .setLinearHeadingInterpolation(intakeGate.getHeading(), shoot.getHeading())
                .build();

        gateIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shoot,
                                intakeGateControl,
                                intakeGate
                        )
                )
                .setLinearHeadingInterpolation(shoot.getHeading(), intakeGate.getHeading())
                .build();

        shootGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakeGate,
                                shoot
                        )
                )
                .setLinearHeadingInterpolation(intakeGate.getHeading(), shoot.getHeading())
                .build();

        gateIntake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shoot,
                                intakeGateControl,
                                intakeGate
                        )
                )
                .setLinearHeadingInterpolation(shoot.getHeading(), intakeGate.getHeading())
                .build();

        shootGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intakeGate,
                                shoot
                        )
                )
                .setLinearHeadingInterpolation(intakeGate.getHeading(), shoot.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shoot,
                                intake1
                        )
                )
                .setLinearHeadingInterpolation(shoot.getHeading(), intake1.getHeading())
                .build();

        shootIntake1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intake1,
                                shoot
                        )
                )
                .setLinearHeadingInterpolation(intake1.getHeading(), shoot.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                shoot,
                                intake3Control,
                                intake3
                        )
                )
                .setLinearHeadingInterpolation(shoot.getHeading(), intake3.getHeading())
                .build();

        shootIntake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                intake3,
                                shoot
                        )
                )
                .setLinearHeadingInterpolation(intake3.getHeading(), shoot.getHeading())
                .build();

    }


    // ----------------- STATE MACHINE LOGIC -----------------
    public void autonomousPathUpdate() {
        switch (pathState) {
            case PRELOAD_INTAKE2:
                if (!follower.isBusy()) {
                    follower.followPath(preshoot_Intake2, true);
                    setPathState(PathState.SHOOT2);

                    //TODO with time delay, add shooting logic
                }
            case SHOOT2:
                if (!follower.isBusy()) {
                    follower.followPath(shootIntake2, true);
                    setPathState(PathState.INTAKE_GATE1);

                    //TODO add shooting logic
                }

            case INTAKE_GATE1:
                if (!follower.isBusy()) {
                    follower.followPath(gateIntake, true);
                    setPathState(PathState.SHOOT_GATE1);
                }
            case SHOOT_GATE1:
                if (!follower.isBusy()) {
                    follower.followPath(shootGate, true);
                    setPathState(PathState.INTAKE_GATE2);

                    //TODO add shooting logic
                }

            case INTAKE_GATE2:
                if (!follower.isBusy()) {
                    follower.followPath(gateIntake, true);
                    setPathState(PathState.SHOOT_GATE2);
                }
            case SHOOT_GATE2:
                if (!follower.isBusy()) {
                    follower.followPath(shootGate, true);
                    setPathState(PathState.INTAKE_GATE3);

                    //TODO add shooting logic
                }

            case INTAKE_GATE3:
                if (!follower.isBusy()) {
                    follower.followPath(gateIntake, true);
                    setPathState(PathState.SHOOT_GATE3);
                }
            case SHOOT_GATE3:
                if (!follower.isBusy()) {
                    follower.followPath(shootGate, true);
                    setPathState(PathState.INTAKE1);

                    //TODO add shooting logic
                }

            case INTAKE1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(PathState.SHOOT_INTAKE1);
                }

            case SHOOT_INTAKE1:
                if (!follower.isBusy()) {
                    follower.followPath(shootIntake1, true);
                    setPathState(PathState.INTAKE3);

                    //TODO add shooting logic
                }

            case INTAKE3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(PathState.SHOOT_INTAKE3);
                }

            case SHOOT_INTAKE3:
                if (!follower.isBusy()) {
                    follower.followPath(shootIntake3, true);
                    setPathState(PathState.END);

                    //TODO add shooting logic
                }
                break;
        }
    }
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        preStart();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(PathState.PRELOAD_INTAKE2);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        publishTelemetry();
    }

    public void publishTelemetry() {
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean up if needed
    }
}