package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.subsystems.DriveTrain;

/**
 * Created by Ben Bernard on 6/8/2018.
 */
public class SwitchDriveMode extends Command {

    DriveTrain.DriveMode _driveMode;
    DriveTrain _driveTrain;

    public SwitchDriveMode(DriveTrain driveTrain, DriveTrain.DriveMode driveMode) {
        _driveTrain = driveTrain;
        _driveMode = driveMode;
    }


    @Override
    protected boolean isFinished() {
        return _driveTrain.getDriveMode()==_driveMode;
    }


    @Override
    protected void execute() {
        SmartDashboard.putString("DriveTrain/Mode", _driveTrain.getDriveMode().name());
        _driveTrain.setDriveMode(_driveMode);
    }
}
