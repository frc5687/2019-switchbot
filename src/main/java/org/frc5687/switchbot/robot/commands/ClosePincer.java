package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import org.frc5687.switchbot.robot.subsystems.Pincer;

/**
 * Created by Ben Bernard on 6/5/2018.
 */
public class ClosePincer extends Command {
    private Pincer _pincer;
    private boolean _done = false;

    public ClosePincer(Pincer pincer) {
        _pincer = pincer;
        requires(_pincer);
    }

    @Override
    protected void initialize() {
        DriverStation.reportError("Starting ClosePincer", false);
        _done = false;
    }

    @Override
    protected boolean isFinished() {
        return _done;
    }

    @Override
    protected void execute() {

        _pincer.close();
        _pincer.setIntakeState(Pincer.IntakeState.HOLD);
        _pincer.runIntake(0);
        _done = true;
    }

    @Override
    protected void end() {
        DriverStation.reportError("Ending ClosePincer", false);
    }
}
