package org.firstinspires.ftc.teamcode.pedroPathing.Opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants.Constants;

@TeleOp
public class IntroToPedro extends OpMode   {

    private Follower follower;
    private Timer pathtimer,opmodeTimer;
    public enum PathState{
        // START POSITION to END POSITION
        // DRIVE > MOVEMENT AROUND THE MAP
        // SHOOT > ATTEMPTS TO SHOOT ARTIFACTS
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD

    }
    PathState pathstate;
    private final Pose startpose = new Pose(19.32,121.683,2.53073);
    private final Pose shootpose = new Pose(46.000, 95.000,2.35619);
    private PathChain driveStartPosShootPos;
    public void buildpaths () {
        //puts cords for starting and ending pos
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(startpose, shootpose))
                .setLinearHeadingInterpolation(startpose.getHeading(), shootpose.getHeading())
                .build();
    }

    public void statepathupdate(){
        switch (pathstate){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathstate(PathState.SHOOT_PRELOAD); //resets the timer & makes new state
                break;
            case SHOOT_PRELOAD:
                //TODO: ADD LOGIC TO FLYWHEEL SHOOTER
                //CHECK THE FOLLOWER IF IT DONE THE PATH
                if (!follower.isBusy()){
                    //TODO - FLYWHEEL LOGIC
                    telemetry.addLine("DONE PATH!");
                }
                break;
            default:
                telemetry.addLine("No State Commanded");
                break;

        }


    }
    public void setPathstate(PathState newstate){

        pathstate = newstate;
        pathtimer.resetTimer();

    }

    @Override
    public void init(){

        pathstate = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathtimer = new Timer();
        opmodeTimer = new Timer();
        //opmodeTimer.resetTimer();;
        //follower = Constants.createFollower(hardwaremap);
        //TODO - ADD IN ALL THE STUFF OF THE HARDWARE MAP

        buildpaths();
        follower.setPose(startpose);

    }
    public void start(){

        opmodeTimer.resetTimer();
        setPathstate(pathstate);

    }

    @Override
    public void loop(){}

}
