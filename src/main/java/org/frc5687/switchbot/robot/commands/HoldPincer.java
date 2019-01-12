package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frc5687.switchbot.robot.Robot;
import org.frc5687.switchbot.robot.subsystems.Pincer;

/**
 * Created by Ben Bernard on 6/5/2018.
 */
public class HoldPincer extends Command {
    private Pincer _pincer;

    public HoldPincer(Pincer pincer) {
        _pincer = pincer;
        requires(_pincer);
    }
    @Override
    protected void execute() {
        _pincer.runIntake(0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
