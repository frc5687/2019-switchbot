package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.OI;
import org.frc5687.switchbot.robot.subsystems.Arm;

/**
 * Created by Ben Bernard on 6/5/2018.
 */
public class DriveArm extends Command {

    private Arm _arm;
    private OI _oi;

    public DriveArm(Arm arm,  OI oi) {
        _arm = arm;
        _oi = oi;
        requires(_arm);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void initialize() {
        double newAngle =_arm.getAngle();
        _arm.setSetpoint(newAngle);
        _arm.enable();
    }

    @Override
    protected void execute() {
        // Get the base speed from the throttle
        double speed = DriverStation.getInstance().isAutonomous() ? 0 : _oi.getArmSpeed();
        if (speed == 0) {
            double newSetpoint = _arm.getSetpoint();
            double setPointSpeed = _oi.getArmSetpointSpeed();
            if (setPointSpeed != 0) {
                double oldSetpoint = _arm.getSetpoint();
                newSetpoint = oldSetpoint + (setPointSpeed* Constants.Arm.SETPOINT_SCALE_FACTOR);
            }
            _arm.setTargetAngle(newSetpoint);
            if (!_arm.isEnabled()){
                _arm.enable();
            }
        } else {
            if (_arm.isEnabled()) {
                _arm.disable();
            }
            _arm.setSetpoint(_arm.getAngle());
            _arm.drive(speed);
        }


    }
}
