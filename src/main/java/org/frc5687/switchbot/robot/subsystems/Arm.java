package org.frc5687.switchbot.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.Robot;
import org.frc5687.switchbot.robot.RobotMap;
import org.frc5687.switchbot.robot.commands.DriveArm;
import org.frc5687.switchbot.robot.utils.AnglePotentiometer;
import org.frc5687.switchbot.robot.utils.PDP;

/**
 * Created by Ben Bernard on 6/5/2018.
 */
public class Arm extends PIDSubsystem {
    private Robot _robot;
    private VictorSP _motor;
    private PDP _pdp;
    private AnglePotentiometer _pot;
    private DigitalInput _frontLimit;
    private DigitalInput _rearLimit;



    private int _direction = 0;
    private boolean _atFrontLimit = false;
    private boolean _atRearLimit = false;

    private long _capTimeout = 0;
    private int _capDirection = 0;

    public static final double kP = 0.03;
    public static final double kI = 0.0001;
    public static final double kD = 0.01;
    public static final double kF = 0;

    public Arm(Robot robot) {
        super("Arm", kP, kI, kD, kF, 0.02);
        this.setOutputRange(-Constants.Arm.SPEED_MAX, Constants.Arm.SPEED_MAX);
        _robot = robot;
        _motor = new VictorSP(RobotMap.PWM.ARM_VICTORSP);
        _pdp = robot.getPDP();
        _pot = new AnglePotentiometer(RobotMap.Analog.ARM_POTENTIOMETER, Constants.Arm.ANGLE_MIN, Constants.Arm.POT_MIN, Constants.Arm.ANGLE_MAX,  Constants.Arm.POT_MAX);
        _frontLimit = new DigitalInput(RobotMap.DIO.ARM_FRONT_LIMIT);
        _rearLimit = new DigitalInput(RobotMap.DIO.ARM_REAR_LIMIT);
    }


    public void drive(double speed) {
        _direction = (int)Math.copySign(1, speed);

        // See if we are drawing too much power...
        if (_pdp.getCurrent(RobotMap.PDP.ARM_VICTORSP) > Constants.Arm.CURRENT_CAP) {
            // If this is the start of an excess draw condition, record it
            DriverStation.reportError("Arm cap of " + Constants.Arm.CURRENT_CAP + " exceeded at " + _pdp.getCurrent(RobotMap.PDP.ARM_VICTORSP), false);
            if (_capTimeout==0) {
                _capTimeout = System.currentTimeMillis() + Constants.Arm.TIMEOUT_CAP;
                _capDirection = _direction;
                DriverStation.reportError("Arm cap setting direction to " + _capDirection, false);
            } else if (_capDirection !=_direction) {
                // If the direction has changed, reset!
                _capTimeout = 0;
                _capDirection = 0;
                DriverStation.reportError("Arm cap resetting direction to " + _capDirection, false);
            } else if (System.currentTimeMillis() > Constants.Arm.TIMEOUT_CAP) {
                // Timeout exceeded...
                _atFrontLimit  = _direction>0;
                _atRearLimit = _direction<0;
                DriverStation.reportError("Arm cap atFront set to " + _atFrontLimit, false);
                DriverStation.reportError("Arm cap atRear set to " + _atRearLimit, false);
            }
        } else {
            // Overdraw condition ended
            if (_capTimeout!=0) {
                DriverStation.reportError("Arm cap limit cleared " + _capDirection, false);
            }
            _capTimeout = 0;
            _capDirection = 0;
        }

        if (speed > 0 && atFrontLimit()) {
            speed = 0;
        } else if (speed < 0 && atRearLimit()) {
            speed = 0;
        }

        _motor.set((Constants.Arm.MOTOR_INVERTED ? -1 : 1) * speed);
        if (!_frontLimit.get()) {
            _pot.restTop();
            SmartDashboard.putNumber("Arm/TopPot",_pot.getRaw());
        }
        if (!_rearLimit.get()) {
            _pot.restBottem();
            SmartDashboard.putNumber("Arm/BottomPot",_pot.getRaw());
        }
    }
    public boolean isEnabled() {return getPIDController().isEnabled();}

    public double getPot() { return _pot.get(); }

    public double getAngle() {
        return getPot();
    }

    public void setTargetAngle(double angle) {
        SmartDashboard.putNumber("Arm/targetAngle", angle);
        setSetpoint(angle);
    }

    public boolean atFrontLimit() {
        return /*_atFrontLimit || */  !_frontLimit.get() ||  getAngle() >= Constants.Arm.ANGLE_MAX;
    }

    public boolean atRearLimit() {
        return /*_atRearLimit || */ !_rearLimit.get() ||  getAngle() <= Constants.Arm.ANGLE_MIN;
    }


    @Override
    protected double returnPIDInput() {
        return getPot();
    }

    @Override
    protected void usePIDOutput(double output) {
        SmartDashboard.putNumber("Arm/PID output", output);
        drive(output);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveArm(this, _robot.getOI()));
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Arm/setpoint", getPIDController().getSetpoint());
        SmartDashboard.putNumber("Arm/angleRaw", _pot.getRaw());
        SmartDashboard.putNumber("Arm/angle", _pot.get());
        SmartDashboard.putBoolean("Arm/FrontLimitSwitch", !_frontLimit.get());
        SmartDashboard.putBoolean("Arm/RearLimitSwitch", !_rearLimit.get());
    }
}
