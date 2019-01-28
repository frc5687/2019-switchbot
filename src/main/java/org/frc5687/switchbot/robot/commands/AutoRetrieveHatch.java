package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.switchbot.robot.Robot;

public class AutoRetrieveHatch extends CommandGroup {
    public AutoRetrieveHatch(Robot robot) {
        addSequential(new AutoDriveToTarget(robot, .2, 16, .5, "Initial approach"));
        addSequential(new OpenPincer(robot.getPincer()));
    }
}
