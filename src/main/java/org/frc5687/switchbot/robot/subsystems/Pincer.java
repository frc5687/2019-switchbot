package org.frc5687.switchbot.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.Robot;
import org.frc5687.switchbot.robot.RobotMap;
import org.frc5687.switchbot.robot.commands.HoldPincer;

/**
 * Created by Ben Bernard on 6/5/2018.
 */
public class Pincer extends Subsystem {
    private Robot _robot;
    private DoubleSolenoid _leftSolenoid;
    private DoubleSolenoid _rightSolenoid;
    private VictorSPX _leftIntakeMotor;
    private VictorSPX _rightIntakeMotor;
    private IntakeState _intakeState = IntakeState.HOLD;
    private AnalogInput _cubeSensor;

    public Pincer(Robot robot) {
        _robot = robot;
        _leftSolenoid = new DoubleSolenoid(RobotMap.PCM.LEFT_PINCER_OPEN, RobotMap.PCM.LEFT_PINCER_CLOSE);
        _rightSolenoid = new DoubleSolenoid(RobotMap.PCM.RIGHT_PINCER_OPEN, RobotMap.PCM.RIGHT_PINCER_CLOSE);
        _leftIntakeMotor = new VictorSPX(RobotMap.CAN.LEFT_INTAKE_VICTORSPX);
        _rightIntakeMotor = new VictorSPX(RobotMap.CAN.RIGHT_INTAKE_VICTORSPX);

        _leftIntakeMotor.configPeakOutputForward(Constants.Intake.HIGH_POW, 0);
        _rightIntakeMotor.configPeakOutputForward(Constants.Intake.HIGH_POW, 0);

        _leftIntakeMotor.configPeakOutputReverse(Constants.Intake.LOW_POW, 0);
        _rightIntakeMotor.configPeakOutputReverse(Constants.Intake.LOW_POW, 0);

        _leftIntakeMotor.configNominalOutputForward(0.0, 0);
        _rightIntakeMotor.configNominalOutputForward(0.0, 0);

        _leftIntakeMotor.configNominalOutputReverse(0.0, 0);
        _rightIntakeMotor.configNominalOutputReverse(0.0, 0);

        _leftIntakeMotor.setInverted(Constants.Intake.LEFT_MOTORS_INVERTED);
        _rightIntakeMotor.setInverted(Constants.Intake.RIGHT_MOTORS_INVERTED);

        _leftIntakeMotor.setNeutralMode(NeutralMode.Brake);
        _rightIntakeMotor.setNeutralMode(NeutralMode.Brake);

        _cubeSensor = new AnalogInput(RobotMap.Analog.CUBE_IR);
    }


    public void open() {
        _leftSolenoid.set(DoubleSolenoid.Value.kForward);
        _rightSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void close() {
        _leftSolenoid.set(DoubleSolenoid.Value.kReverse);
        _rightSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void runIntake(double speed) {
        switch(_intakeState) {
            case HOLD:
                run(Constants.Intake.HOLD_SPEED);
                break;
            case INTAKE:
                run(Constants.Intake.INTAKE_SPEED);
                break;
            case EJECT:
                run(speed);
                break;
            default:
                run(0);
        }
        _robot.getLEDStrip().setHasCube(isCubeDetected());
    }

    public void run(double speed) {
        _leftIntakeMotor.set(ControlMode.PercentOutput, -speed);
        _rightIntakeMotor.set(ControlMode.PercentOutput, -speed);
    }

    private boolean isCubeDetected() {
        return _cubeSensor.getValue()>= Constants.Intake.CUBE_DETECTED_THRESHOLD;
    }

    public void setIntakeState(IntakeState intakeState) {
        _intakeState = intakeState;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new HoldPincer(this));
    }


    public enum IntakeState {
        HOLD(0),
        INTAKE(1),
        EJECT(2);

        private int _value;

        IntakeState(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Pincer/IRSenson", _cubeSensor.getValue());
        SmartDashboard.putBoolean("Pincer/CubeDetected", isCubeDetected());
    }
}
