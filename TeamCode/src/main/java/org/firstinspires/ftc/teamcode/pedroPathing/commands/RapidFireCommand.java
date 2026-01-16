package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.SpindexSubsystem;

public class RapidFireCommand {

    private enum State {
        IDLE,
        SPINUP_WAIT,
        FLICK_UP
    }

    private State state = State.IDLE;

    private final ShooterSubsystem shooter;
    private final SpindexSubsystem spindex;

    private int index = 0;
    private long timer = 0;

    private static final long SPINUP_DELAY_MS = 1000;
    private static final long FLICK_UP_MS = 200;

    private double targetVelocity = 0;

    public RapidFireCommand(ShooterSubsystem shooter, SpindexSubsystem spindex) {
        this.shooter = shooter;
        this.spindex = spindex;
    }

    public void start(double velocity) {
        if (state != State.IDLE || !spindex.anySlotLoaded()) return;

        targetVelocity = velocity;
        index = 0;
        timer = System.currentTimeMillis() + SPINUP_DELAY_MS;
        state = State.SPINUP_WAIT;
    }

    public void update() {
        long now = System.currentTimeMillis();

        if (state != State.IDLE) {
            shooter.setTargetVelocity(targetVelocity);
        }

        switch (state) {
            case SPINUP_WAIT:
                spindex.setShootPosition(index);
                if (now >= timer) {
                    spindex.flickUp();
                    timer = now + FLICK_UP_MS;
                    state = State.FLICK_UP;
                }
                break;

            case FLICK_UP:
                if (now >= timer) {
                    spindex.flickDown();
                    spindex.clearSlot(index);

                    if (index < 2 && spindex.anySlotLoaded()) {
                        index++;
                        timer = now + SPINUP_DELAY_MS;
                        state = State.SPINUP_WAIT;
                    } else {
                        shooter.stopShooter();
                        index = 0;
                        state = State.IDLE;
                    }
                }
                break;
        }
    }

    public boolean isActive() {
        return state != State.IDLE;
    }

    public void stop() {
        shooter.stopShooter();
        index = 0;
        state = State.IDLE;
    }
}
