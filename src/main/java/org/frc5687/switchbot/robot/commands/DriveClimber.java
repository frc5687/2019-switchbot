package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frc5687.switchbot.robot.OI;
import org.frc5687.switchbot.robot.Robot;
import org.frc5687.switchbot.robot.subsystems.Climber;

public class DriveClimber extends Command {

    private Climber _climber;
    private OI _oi;

    public DriveClimber(Robot robot, Climber climber) {
        _climber = climber;
        _oi = robot.getOI();
        requires(_climber);
    }

    @Override
    protected void initialize() {
    }

    @Override
    public void execute() {
        // Read fore and aft stick positions from OI
        double fore = _oi.getIntakeSpeed();
        double aft = _oi.getElevatorSpeed();

        // Send to the climber
        _climber.setSpeeds(fore, aft);
    }


    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        // Set climber motor speeds to 0 and set break mode?s
    }
}
