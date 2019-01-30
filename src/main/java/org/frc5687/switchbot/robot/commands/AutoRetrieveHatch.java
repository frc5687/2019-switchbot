package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.switchbot.robot.Robot;

public class AutoRetrieveHatch extends CommandGroup {
    public AutoRetrieveHatch(Robot robot) {
        addSequential(new AutoDriveToTarget(robot, .5, 6, .5, "Initial approach"));
        addSequential(new OpenPincer(robot.getPincer()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), -12, 0.25, ""));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), -24, 0.5, ""));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), 170, 1));
        addSequential(new AutoDriveToTarget(robot, .5, 6, .5, "Deposit"));
        addSequential(new ClosePincer(robot.getPincer()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), -12, 0.25, ""));
    }
}
