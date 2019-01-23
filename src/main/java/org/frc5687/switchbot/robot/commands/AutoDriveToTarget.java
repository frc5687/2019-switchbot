package org.frc5687.switchbot.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.OI;
import org.frc5687.switchbot.robot.Robot;
import org.frc5687.switchbot.robot.subsystems.DriveTrain;
import org.frc5687.switchbot.robot.utils.Limelight;

public class AutoDriveToTarget extends Command  {

    private DriveTrain _driveTrain;
    private AHRS _imu;
    private Limelight _limelight;
    private OI _oi;

    private PIDController _angleController;
    private PIDController _distanceController;

    private double _angleTarget;
    private double _distanceTarget;

    private double _distanceTolerance;

    private double speed;
    private long _timeout = 2000;

    private double _anglePIDOut;
    private double _distancePIDOut;


    private long _startTimeMillis;
    private boolean _aborted = false;


    private String _message = "";


    public AutoDriveToTarget(Robot robot, double speed, long timeout, double tolerance, String message) {
        _driveTrain = robot.getDriveTrain();
        _imu = robot.getIMU();
        _limelight = robot.get_limelight();
        _oi = robot.getOI();

        requires(_driveTrain);
        this.speed = speed;
        _timeout = timeout;
        _distanceTolerance = tolerance;
        _message = message;
    }

    @Override
    protected void initialize() {
        double kPAngle = Constants.Auto.DriveToTarget.kPAngle; // Double.parseDouble(SmartDashboard.getString("DB/String 0", ".04"));
        double kIAngle = Constants.Auto.DriveToTarget.kIAngle; // Double.parseDouble(SmartDashboard.getString("DB/String 1", ".006"));
        double kDAngle = Constants.Auto.DriveToTarget.kDAngle; //Double.parseDouble(SmartDashboard.getString("DB/String 2", ".09"));

        double kPDistance = Constants.Auto.DriveToTarget.kPDistance; // Double.parseDouble(SmartDashboard.getString("DB/String 0", ".04"));
        double kIDistance = Constants.Auto.DriveToTarget.kIDistance; // Double.parseDouble(SmartDashboard.getString("DB/String 1", ".006"));
        double kDDistance = Constants.Auto.DriveToTarget.kDDistance; //Double.parseDouble(SmartDashboard.getString("DB/String 2", ".09"));

        // 1: Read current target _angleTarget from limelight
        // 2: Read current yaw from navX
        // 3: Set _angleController._angleTarget to sum

        double limeLightAngle = _limelight.getHorizontalAngle();
        double yawAngle = _imu.getAngle();
        _angleTarget = limeLightAngle + yawAngle;

        SmartDashboard.putNumber("AutoAlignToTarget/startoffset", limeLightAngle);
        SmartDashboard.putNumber("AutoAlignToTarget/startyaw", yawAngle);
        SmartDashboard.putNumber("AutoAlignToTarget/angletarget", _angleTarget);

        _angleController = new PIDController(kPAngle, kIAngle, kDAngle, _imu, new AngleListener(), 0.01);
        _angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        _angleController.setOutputRange(-Constants.Auto.DriveToTarget.TURN_SPEED, Constants.Auto.DriveToTarget.TURN_SPEED);
        _angleController.setAbsoluteTolerance(Constants.Auto.DriveToTarget.ANGLE_TOLERANCE);
        _angleController.setContinuous();
        _angleController.setSetpoint(_angleTarget);
        _angleController.enable();


        _distanceController = new PIDController(kPDistance, kIDistance, kDDistance, _driveTrain.getIRDistanceSensor(), new DistanceListener(), 0.1);
        _distanceController.setInputRange(0, 80);
        _distanceController.setOutputRange(-speed, speed);
        _distanceController.setAbsoluteTolerance(_distanceTolerance);
        _distanceController.setContinuous(false);
        _distanceController.setSetpoint(_distanceTarget);
        _distanceController.enable();

        _startTimeMillis = System.currentTimeMillis();

    }

    @Override
    protected void execute() {
        double limeLightAngle = _limelight.getHorizontalAngle();
        double yawAngle = _imu.getAngle();
        _angleTarget = limeLightAngle + yawAngle;

        SmartDashboard.putNumber("AutoAlignToTarget/startoffset", limeLightAngle);
        SmartDashboard.putNumber("AutoAlignToTarget/startyaw", yawAngle);
        SmartDashboard.putNumber("AutoAlignToTarget/target", _angleTarget);

        if (Math.abs(_angleTarget - _angleController.getSetpoint()) > Constants.Auto.DriveToTarget.ANGLE_TOLERANCE) {
            _angleController.setSetpoint(_angleTarget);
            SmartDashboard.putNumber("AutoAlignToTarget/setpoint", _angleTarget);
        }

        SmartDashboard.putBoolean("AutoAlignToTarget/onTarget", _angleController.onTarget());
        SmartDashboard.putNumber("AutoAlignToTarget/yaw", _imu.getYaw());

        SmartDashboard.putNumber("AutoAlignToTarget/_anglePIDOut", _anglePIDOut);

        _driveTrain.setPower(_distancePIDOut + _anglePIDOut, _distancePIDOut  -_anglePIDOut, true); // positive output is clockwise
    }


    @Override
    protected boolean isFinished() {
        if (_aborted) { return true; }

        if (_distanceController.onTarget()) {
            return true;
        }

        return false;
    }

    @Override
    protected void end() {
        _driveTrain.setPower(0,0, true);
        DriverStation.reportError("AutoAlign finished: _angleTarget = " + _imu.getYaw() + ", time = " + (System.currentTimeMillis() - _startTimeMillis), false);
        _angleController.disable();
        DriverStation.reportError("AutoAlign.end() _angleController disabled", false);
    }

    private class AngleListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _anglePIDOut = output;
            }
        }
    }

    private class DistanceListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _distancePIDOut = output;
            }
        }
    }



}