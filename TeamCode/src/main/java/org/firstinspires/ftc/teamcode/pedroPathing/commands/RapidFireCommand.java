package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.SpindexSubsystem;

public class RapidFireCommand {

    private ShooterSubsystem shooter;
    private SpindexSubsystem spindex;
    private double targetVelocity;

    private enum State {
        IDLE,
        SPINUP_WAIT,
        FLICK_UP
    }

    private State state = State.IDLE;
    private int currentIndex = 0;
    private long timer = 0;
    private long intakeBurstUntil = 0;

    private final long SPINUP_DELAY_MS = 1000;
    private final long FLICK_UP_MS = 200;
    private final long INTAKE_BURST_MS = 800;

    public RapidFireCommand(ShooterSubsystem shooter, SpindexSubsystem spindex, double targetVelocity) {
        this.shooter = shooter;
        this.spindex = spindex;
        this.targetVelocity = targetVelocity;
    }

    public void update(boolean triggerPressed, long currentTime) {
        // Start the rapid fire if trigger pressed and any slot loaded
        if (triggerPressed && state == State.IDLE && spindex.anySlotLoaded()) {
            intakeBurstUntil = currentTime + INTAKE_BURST_MS;
            currentIndex = 0;
            timer = currentTime + SPINUP_DELAY_MS;
            state = State.SPINUP_WAIT;
        }

        // Run FSM
        switch (state) {
            case SPINUP_WAIT:
                spindex.setShootPosition(currentIndex);
                shooter.setTargetVelocity(targetVelocity);

                if (currentTime >= timer) {
                    spindex.flickUp();
                    timer = currentTime + FLICK_UP_MS;
                    state = State.FLICK_UP;
                }
                break;

            case FLICK_UP:
                if (currentTime >= timer) {
                    spindex.flickDown();
                    spindex.clearSlot(currentIndex);

                    if (currentIndex < 2 && spindex.anySlotLoaded()) {
                        currentIndex++;
                        intakeBurstUntil = currentTime + INTAKE_BURST_MS;
                        timer = currentTime + SPINUP_DELAY_MS;
                        state = State.SPINUP_WAIT;
                    } else {
                        state = State.IDLE;
                        shooter.stopShooter();
                        currentIndex = 0;
                    }
                }
                break;

            case IDLE:
            default:
                shooter.stopShooter();
                break;
        }
    }

    public long getIntakeBurstUntil() {
        return intakeBurstUntil;
    }

    public State getState() {
        return state;
    }
}
