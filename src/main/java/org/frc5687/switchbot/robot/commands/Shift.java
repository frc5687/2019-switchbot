package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.subsystems.DriveTrain;
import org.frc5687.switchbot.robot.subsystems.Shifter;

public class Shift extends Command {
    private Shifter _shifter;
    private DriveTrain _driveTrain;
    private Shifter.Gear gear;

    private double initialLeftSpeed, initialRightSpeed;
    private long endTime;
    private State state = State.STOP_MOTOR;
    private boolean auto;

    public Shift(DriveTrain driveTrain, Shifter shifter, Shifter.Gear gear, boolean auto) {
        _driveTrain = driveTrain;
        _shifter = shifter;

        requires(_driveTrain);
        requires(_shifter);

        this.gear = gear;
        this.auto = auto;
    }

    @Override
    protected void initialize() {
        DriverStation.reportError("Shifting to " + gear, false);
        state = State.STOP_MOTOR;
    }

    @Override
    protected void execute() {
        switch (state) {
            case STOP_MOTOR:
                initialLeftSpeed = _driveTrain.getLeftSpeed();
                initialRightSpeed = _driveTrain.getRightSpeed();
                _driveTrain.tankDrive(0, 0, true);
                endTime = System.currentTimeMillis() + Constants.Shifter.STOP_MOTOR_TIME;
                state = State.WAIT_FOR_MOTOR;
                break;
            case WAIT_FOR_MOTOR:
                if (System.currentTimeMillis() >= endTime) state = State.SHIFT;
                break;
            case SHIFT:
                _shifter.shift(gear, auto);
                endTime = System.currentTimeMillis() + Constants.Shifter.SHIFT_TIME;
                state = State.WAIT_FOR_SHIFT;
                break;
            case WAIT_FOR_SHIFT:
                if (System.currentTimeMillis() >= endTime) state = State.START_MOTOR;
                break;
            case START_MOTOR:
                _driveTrain.tankDrive(initialLeftSpeed, initialRightSpeed, true);
                state = State.DONE;
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        return state == State.DONE;
    }

    @Override
    protected void interrupted() {
    }

    public enum State {
        STOP_MOTOR,
        WAIT_FOR_MOTOR,
        SHIFT,
        WAIT_FOR_SHIFT,
        START_MOTOR,
        DONE;
    }

}
