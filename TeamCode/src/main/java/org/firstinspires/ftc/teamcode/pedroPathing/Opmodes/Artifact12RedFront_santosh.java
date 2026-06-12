package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

public class Artifact12RedFront_santosh {
    public static class Paths {
        public PathChain MainChain;

        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(123.000, 121.000),
                                    new Pose(82.000, 82.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(82.000, 82.000),
                                    new Pose(110.000, 82.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(110.000, 82.000),
                                    new Pose(115.000, 82.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(115.000, 82.000),
                                    new Pose(120.000, 82.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(120.000, 82.000),
                                    new Pose(82.000, 82.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.000, 82.000),
                                    new Pose(78.416, 56.423),
                                    new Pose(100.000, 58.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.000, 58.000),
                                    new Pose(110.000, 58.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(110.000, 58.000),
                                    new Pose(115.000, 58.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(115.000, 58.000),
                                    new Pose(120.000, 58.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(120.000, 58.000),
                                    new Pose(82.000, 82.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.000, 82.000),
                                    new Pose(78.246, 30.226),
                                    new Pose(110.000, 35.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(110.000, 35.000),
                                    new Pose(115.000, 35.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(115.000, 35.000),
                                    new Pose(120.000, 35.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(120.000, 35.000),
                                    new Pose(82.000, 82.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}
