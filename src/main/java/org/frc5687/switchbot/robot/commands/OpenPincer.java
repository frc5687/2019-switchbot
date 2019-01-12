package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frc5687.switchbot.robot.subsystems.Pincer;

/**
 * Created by Ben Bernard on 6/5/2018.
 */
public class OpenPincer extends Command {
    private Pincer _pincer;
    private boolean _done = false;

    public OpenPincer(Pincer pincer) {
        _pincer = pincer;
        requires(_pincer);
    }

    @Override
    protected void initialize() {
        _done = false;
    }

    @Override
    protected boolean isFinished() {
        return _done;
    }

    @Override
    protected void execute() {
        _pincer.open();
        _pincer.setIntakeState(Pincer.IntakeState.INTAKE);
        _pincer.runIntake(0);
        _done = true;
    }
}
