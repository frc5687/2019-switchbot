package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.frc5687.switchbot.robot.OI;
import org.frc5687.switchbot.robot.subsystems.DriveTrain;

/**
 * Created by Ben Bernard on 6/4/2018.
 */
public class AllDrive extends Command {

    private OI _oi;
    private DriveTrain _driveTrain;

    public AllDrive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        requires(_driveTrain);
    }


 @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void execute() {
        DriveTrain.DriveMode driveMode = _driveTrain.getDriveMode();
        switch (driveMode) {
            case ARCADE:
                // Get the base speed from the throttle
                double speed = _oi.getDriveSpeed(driveMode);

                // Get the rotation from the tiller
                double rotation = _oi.getDriveRotation(driveMode);

                _driveTrain.arcadeDrive(speed, rotation);
                break;
            case CHEESY_ARCADE:
                // Get the base speed from the throttle
                // Get the base speed from the throttle
                double stickSpeed = _oi.getDriveSpeed(driveMode);

                // Get the rotation from the tiller
                double wheelRotation = _oi.getDriveRotation(driveMode);

                _driveTrain.cheesyDrive(stickSpeed, wheelRotation);

                break;

            case TANK:
                double leftSpeed = _oi.getLeftSpeed();
                double rightSpeed = _oi.getRightSpeed();

                _driveTrain.tankDrive(leftSpeed, rightSpeed, false);
                break;
        }
    }
}
