package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;

public class IntakeCommand {

    private final IntakeSubsystem intake;
    private final ElapsedTime timer = new ElapsedTime();

    private final boolean reverse;
    private final double duration; // seconds (0 = indefinite)
    private boolean isFinished = false;

    /**
     * Creates a new IntakeCommand.
     *
     * @param intake   The intake subsystem to control.
     * @param reverse  true = outtake, false = intake.
     * @param duration How long to run (seconds). 0 = indefinite.
     */
    public IntakeCommand(IntakeSubsystem intake, boolean reverse, double duration) {
        this.intake = intake;
        this.reverse = reverse;
        this.duration = duration;
    }

    /** Called once when the command starts */
    public void initialize() {
        timer.reset();
        if (reverse) {
            intake.outtake();
        } else {
            intake.intake();
        }
    }

    /** Called periodically while the command runs */
    public void execute() {
        intake.update();
        if (duration > 0 && timer.seconds() >= duration) {
            isFinished = true;
        }
    }

    /** Called once when the command ends or is interrupted */
    public void end() {
        intake.stop();
    }

    /** Returns true when the command should finish */
    public boolean isFinished() {
        return isFinished;
    }
}
