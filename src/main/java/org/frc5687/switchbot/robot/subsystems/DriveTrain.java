package org.frc5687.switchbot.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.Robot;
import org.frc5687.switchbot.robot.RobotMap;
import org.frc5687.switchbot.robot.commands.AllDrive;
import org.frc5687.switchbot.robot.utils.Helpers;

import static org.frc5687.switchbot.robot.utils.Helpers.limit;

/**
 * Created by Ben Bernard on 6/4/2018.
 */
public class DriveTrain extends Subsystem  implements PIDSource {
    // Add the objects for the motor controllers


    CANSparkMax _leftMaster;
    CANSparkMax _leftFollower;

    CANSparkMax _rightMaster;
    CANSparkMax _rightFollower;

    CANEncoder _leftEncoder;
    CANEncoder _rightEncoder;

    private Robot _robot;
    private DriveMode _driveMode = DriveMode.CHEESY_ARCADE;
    private AnalogInput _lightSensor;
    public AHRS _imu;
    private double _rightOffset;
    private double _leftOffset;
    private double _lastLeftPosition = 0;
    private double _lastRightPosition = 0;

    public DriveTrain(Robot robot) {
        _robot = robot;
        _imu = robot.getIMU();

        _leftMaster = new CANSparkMax(RobotMap.CAN.LEFT_MASTER_SPARK, CANSparkMaxLowLevel.MotorType.kBrushless);
        _rightMaster = new CANSparkMax(RobotMap.CAN.RIGHT_MASTER_SPARK, CANSparkMaxLowLevel.MotorType.kBrushless);
        _leftFollower = new CANSparkMax(RobotMap.CAN.LEFT_FOLLOWER_SPARK, CANSparkMaxLowLevel.MotorType.kBrushless);
        _rightFollower = new CANSparkMax(RobotMap.CAN.RIGHT_FOLLOWER_SPARK, CANSparkMaxLowLevel.MotorType.kBrushless);

        // Motor Initialization

        // Setup followers to follow their master
        _leftFollower.follow(_leftMaster);
        
        _rightFollower.follow(_rightMaster);

        // Setup motors

        // _leftMaster.configPeakOutputForward(Constants.DriveTrain.HIGH_POW, 0);
        // _leftFollowerA.configPeakOutputForward(Constants.DriveTrain.HIGH_POW, 0);

//        _rightMaster.configPeakOutputForward(Constants.DriveTrain.HIGH_POW, 0);
//        _rightFollowerA.configPeakOutputForward(Constants.DriveTrain.HIGH_POW, 0);
//
//        _leftMaster.configPeakOutputReverse(Constants.DriveTrain.LOW_POW, 0);
//        _leftFollowerA.configPeakOutputReverse(Constants.DriveTrain.LOW_POW, 0);
//
//        _rightMaster.configPeakOutputReverse(Constants.DriveTrain.LOW_POW, 0);
//        _rightFollowerA.configPeakOutputReverse(Constants.DriveTrain.LOW_POW, 0);
//
//
//        _leftMaster.configNominalOutputForward(0.0, 0);
//        _leftFollowerA.configNominalOutputForward(0.0, 0);
//        _rightMaster.configNominalOutputForward(0.0, 0);
//        _rightFollowerA.configNominalOutputForward(0.0, 0);
//
//        _leftMaster.configNominalOutputReverse(0.0, 0);
//        _leftFollowerA.configNominalOutputReverse(0.0, 0);
//        _rightMaster.configNominalOutputReverse(0.0, 0);
//        _rightFollowerA.configNominalOutputReverse(0.0, 0);
//
        _leftMaster.setInverted(Constants.DriveTrain.LEFT_MOTORS_INVERTED);
        _leftFollower.setInverted(Constants.DriveTrain.LEFT_MOTORS_INVERTED);
        _rightMaster.setInverted(Constants.DriveTrain.RIGHT_MOTORS_INVERTED);
        _rightFollower.setInverted(Constants.DriveTrain.RIGHT_MOTORS_INVERTED);
//
        // Configure the encoders
        _leftEncoder = _leftMaster.getEncoder();
        _rightEncoder = _rightMaster.getEncoder();

        resetDriveEncoders();

        _lightSensor = new AnalogInput(RobotMap.Analog.LIGHT_SENSOR);


    }


    protected void initDefaultCommand() {
        setDefaultCommand(new AllDrive(this, _robot.getOI()));
    }

    public void setPower(double leftSpeed, double rightSpeed) {
        setPower(leftSpeed, rightSpeed, false);
    }
    public void setPower(double leftSpeed, double rightSpeed, boolean override) {
        try {
            _leftMaster.set(leftSpeed);
            _rightMaster.set(rightSpeed);
        } catch (Exception e) {
            DriverStation.reportError("DriveTrain.setPower exception: " + e.toString(), false);
        }
        SmartDashboard.putNumber("DriveTrain/PowerRight", rightSpeed);
        SmartDashboard.putNumber("DriveTrain/PowerLeft", leftSpeed);
    }


    public void resetDriveEncoders() {
        try {
//            _leftMaster.setSelectedSensorPosition(0,0,0);
//            _rightMaster.setSelectedSensorPosition(0, 0, 0);
        } catch (Exception e) {
            DriverStation.reportError("DriveTrain.resetDriveEncoders exception. I suppose this is really bad. : " + e.toString(), false);
        }
    }


    public void arcadeDrive(double speed, double rotation) {
        SmartDashboard.putNumber("DriveTrain/Speed", speed);
        SmartDashboard.putNumber("DriveTrain/Rotation", rotation);

        speed = limit(speed);

        rotation = limit(rotation);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        speed = Math.copySign(speed * speed, speed);
        rotation = Math.copySign(rotation * rotation, rotation);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);

        if (speed >= 0.0) {
            // First quadrant, else second quadrant
            if (rotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            } else {
                leftMotorOutput = speed + rotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (rotation >= 0.0) {
                leftMotorOutput = speed + rotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            }
        }

        setPower(leftMotorOutput, rightMotorOutput);
    }

    public void cheesyDrive(double speed, double rotation) {
        SmartDashboard.putNumber("DriveTrain/Speed", speed);
        SmartDashboard.putNumber("DriveTrain/Rotation", rotation);

        speed = limit(speed);
        Shifter.Gear gear = _robot.getShifter().getGear();

        rotation = limit(rotation);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        speed = Math.copySign(speed * speed, speed);
        rotation = Helpers.applySensitivityFactor(rotation,  gear == Shifter.Gear.HIGH ? Constants.DriveTrain.ROTATION_SENSITIVITY_HIGH_GEAR : Constants.DriveTrain.ROTATION_SENSITIVITY_LOW_GEAR);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);

        if (speed==0.0) {
            leftMotorOutput = -rotation;
            rightMotorOutput = rotation;
        } else {
            double delta = rotation * Math.abs(speed);
            leftMotorOutput = speed - delta;
            rightMotorOutput = speed + delta;
        }

        setPower(limit(leftMotorOutput), limit(rightMotorOutput));
    }


    public void tankDrive(double leftSpeed, double rightSpeed, boolean overrideCaps) {
        SmartDashboard.putNumber("DriveTrain/LeftSpeed", leftSpeed);
        SmartDashboard.putNumber("DriveTrain/RightSpeed", rightSpeed);
        double leftMotorOutput = leftSpeed;
        double rightMotorOutput = rightSpeed;



        setPower(leftMotorOutput, rightMotorOutput);
    }

    public float getYaw() {
        return _imu.getYaw();
    }

    /**
     * Get the number of ticks since the last reset
     * @return
     */
    public long getLeftTicks() {
        return (long)_leftEncoder.getPosition();
    }

    public long getRightTicks() {
        return (long)_rightEncoder.getPosition();
    }

    /**
     * The left distance in Inches since the last reset.
     * @return
     */
    public double getLeftDistance() {
        double current = _leftEncoder.getPosition();
        if (current!=0) {
            _lastLeftPosition = current;
        }
        return _lastLeftPosition * Constants.DriveTrain.LEFT_RATIO - _leftOffset;
    }

    public double getRightDistance() {
        double current = _rightEncoder.getPosition();
        if (current!=0) {
            _lastRightPosition = current;
        }
        return _lastRightPosition * Constants.DriveTrain.RIGHT_RATIO - _rightOffset;
    }

    /**
     * @return average of leftDistance and rightDistance
     */
    public double getDistance() {
        if (Math.abs(getRightTicks())<10) {
            return getLeftDistance();
        }
        if (Math.abs(getLeftTicks())<10) {
            return getRightDistance();
        }
        return (getLeftDistance() + getRightDistance()) / 2;
    }




    /*
    public void curvaturerive(double speed, double rotation, boolean isQuickTurn) {

        speed = limit(speed);

        rotation = limit(rotation);

        double angularPower;
        boolean overPower;

        if (isQuickTurn) {
            if (Math.abs(speed) < m_quickStopThreshold) {
                m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
                        + m_quickStopAlpha * limit(zRotation) * 2;
            }
            overPower = true;
            angularPower = zRotation;
        } else {
            overPower = false;
            angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

            if (m_quickStopAccumulator > 1) {
                m_quickStopAccumulator -= 1;
            } else if (m_quickStopAccumulator < -1) {
                m_quickStopAccumulator += 1;
            } else {
                m_quickStopAccumulator = 0.0;
            }
        }

        double leftMotorOutput = xSpeed + angularPower;
        double rightMotorOutput = xSpeed - angularPower;

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }

        m_leftMotor.set(leftMotorOutput * m_maxOutput);
        m_rightMotor.set(-rightMotorOutput * m_maxOutput);

        m_safetyHelper.feed();
    }
*/

    public void setCurrentLimiting(int amps) {
        _leftMaster.setSmartCurrentLimit(amps);
        _rightMaster.setSmartCurrentLimit(amps);
    }


    public double getLeftSpeed() {
        return _leftMaster.get();
    }

    public double getRightSpeed() {
        return _rightMaster.get();
    }

    public double getLeftRate() {
        return _leftEncoder.getVelocity();
    }

    public double getRightRate() {
        return _rightEncoder.getVelocity();
    }

    public DriveMode getDriveMode() { return _driveMode; }


    public boolean tapeIsDetected() {
        int value = _lightSensor.getValue();
        return value >= Constants.DriveTrain.LIGHT_SENSOR.DETECTED_VALUE;
    }


    @Override
    public double pidGet() {
        return getDistance();
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    public void setDriveMode(DriveMode driveMode) { _driveMode = driveMode; }

    public void enableBrakeMode() {
        try {
            _leftMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
            _rightMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
            _leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
            _rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);

        } catch (Exception e) {
            DriverStation.reportError("DriveTrain.enableBrakeMode exception: " + e.toString(), false);
        }
        SmartDashboard.putString("DriveTrain/neutralMode", "Brake");
    }

    public void enableCoastMode() {
        try {
            _leftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
            _rightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
            _leftFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
            _rightFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        } catch (Exception e) {
            DriverStation.reportError("DriveTrain.enableCoastMode exception: " + e.toString(), false);
        }
        SmartDashboard.putString("DriveTrain/neutralMode", "Coast");
    }



    public enum DriveMode {
        TANK(0),
        ARCADE(1),
        CHEESY_ARCADE(2);

        private int _value;

        DriveMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }

    public void updateDashboard() {
        SmartDashboard.putNumber("DriveTrain/LeftDistance", _lastLeftPosition);
        SmartDashboard.putNumber("DriveTrain/RIghtDistance", _lastRightPosition);
        SmartDashboard.putNumber("DriveTrain/LeftRate", getLeftRate());
        SmartDashboard.putNumber("DriveTrain/RightRate", getRightRate());
        SmartDashboard.putNumber("DriveTrain/LeftSpeed", getLeftSpeed());
        SmartDashboard.putNumber("DriveTrain/RightSpeed", getRightSpeed());
        SmartDashboard.putNumber("DriveTrain/Yaw", _imu.getYaw());
        SmartDashboard.putBoolean("DriveTrain/tapeIsDetected", tapeIsDetected());
        SmartDashboard.putNumber("DriveTrain/IR Tape raw", _lightSensor.getValue());

    }

}
