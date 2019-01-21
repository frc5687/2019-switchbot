package org.frc5687.switchbot.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.Robot;
import org.frc5687.switchbot.robot.RobotMap;
import org.frc5687.switchbot.robot.commands.DriveClimber;
import org.frc5687.switchbot.robot.utils.Helpers;

public class Climber extends Subsystem {

    private CANSparkMax _intake;
    private CANSparkMax _elevator;

    private Robot _robot;
    public Climber(Robot robot) {
        _robot = robot;

        _intake = new CANSparkMax(RobotMap.CAN.INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        _elevator = new CANSparkMax(RobotMap.CAN.ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        _intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _elevator.setIdleMode(CANSparkMax.IdleMode.kBrake);

    }

    public void setSpeeds(double fore, double aft) {
        fore = Helpers.limit(fore, Constants.CLIMBER.MAX_INTAKE_SPEED);
        aft = Helpers.limit(aft, Constants.CLIMBER.MAX_ELEVATOR_SPEED);

        DriverStation.reportError("CLIMBER: Intake at " + fore + " and aft " + aft, false);
        _intake.set(fore);
        _elevator.set(aft);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveClimber(_robot, this));
    }
}
