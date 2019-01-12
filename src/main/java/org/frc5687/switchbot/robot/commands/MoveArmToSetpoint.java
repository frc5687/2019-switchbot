package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import org.frc5687.switchbot.robot.OI;
import org.frc5687.switchbot.robot.subsystems.Arm;

/**
 * Created by Ben Bernard on 6/8/2018.
 */
public class MoveArmToSetpoint extends Command {
    private double _target;
    private Arm _arm;
    private long _timeout = 10000;
    private long _endMillis;
    private OI _oi;


    public MoveArmToSetpoint(Arm arm, OI oi, double target) {
        requires(arm);
        _arm = arm;
        _target = target;
        _oi = oi;
    }

    public MoveArmToSetpoint(Arm arm, OI oi, double target, long timeout) {
        this.setInterruptible(true);
        requires(arm);
        _arm = arm;
        _target = target;
        _timeout = timeout;
        _oi = oi;
    }


    @Override
    protected void end() {
        DriverStation.reportError("MoveArmToSetpointPID Ending", false);

    }

    @Override
    protected boolean isFinished() {
        return _arm.onTarget();
    }


    @Override
    protected void initialize() {
        super.initialize();
        _endMillis = System.currentTimeMillis() + _timeout;
        DriverStation.reportError("Starting MoveArmToSetpointPID to " + _target + " for max " + _timeout + "ms", false);
        _arm.setSetpoint(_target);
        _arm.setAbsoluteTolerance(1.0);
        _arm.enable();
    }

    @Override
    protected void execute() {
        // DriverStation.reportError("MoveArmToSetpointPID at " + _arm.getAngle(), false);
    }

}
