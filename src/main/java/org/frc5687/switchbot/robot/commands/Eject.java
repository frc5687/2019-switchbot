package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.subsystems.Pincer;
import org.frc5687.switchbot.robot.OI;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Created by Ben Bernard on 6/5/2018.
 */
public class Eject extends Command {
    private Pincer _pincer;
    private OI _oi;
    private long _endtime = 0;
    public boolean triggerUp = false;
    public Eject(Pincer pincer, OI oi) {
        _oi = oi;
        _pincer = pincer;
        requires(_pincer);
    }

    @Override
    protected void initialize() {
        _endtime = System.currentTimeMillis() + Constants.Intake.EJECT_TIME_MILLIS;
    }

    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _endtime && (DriverStation.getInstance().isAutonomous() || _oi==null || !_oi.isEjectButtonPressed());
    }

    @Override
    protected void execute() {
        double speed = (DriverStation.getInstance().isAutonomous() || _oi==null) ? Constants.Intake.EJECT_SPEED : _oi.getEjectSpeed();
        SmartDashboard.putNumber("Pincer/intakeSpeed", speed);
        _pincer.setIntakeState(Pincer.IntakeState.EJECT);
        _pincer.runIntake(speed);
    }

    @Override
    protected void end() {
        _pincer.setIntakeState(Pincer.IntakeState.HOLD);
    }
}
