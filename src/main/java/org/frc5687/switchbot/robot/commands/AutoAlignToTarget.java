package org.frc5687.switchbot.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.OI;
import org.frc5687.switchbot.robot.Robot;
import org.frc5687.switchbot.robot.subsystems.DriveTrain;

public class AutoAlignToTarget extends Command implements PIDOutput {

    private PIDController controller;
    private double endTime;
    private double angle;
    private double speed;
    private long _timeout = 2000;

    private double pidOut;

    private long _onTargetSince;
    private long startTimeMillis;
    private long _endTimeMillis;
    private boolean _aborted = false;

    private DriveTrain driveTrain;
    private AHRS imu;

    private String _message = "";

    NetworkTable _table;

    private OI _oi;

    private DriveTrainBehavior _driveTrainBehavior = DriveTrainBehavior.bothSides;

    private double _tolerance;

    public AutoAlignToTarget(Robot robot ) {
        this(robot, Constants.Auto.Align.SPEED);
    }

    public AutoAlignToTarget(Robot robot, double speed) {
        this(robot.getDriveTrain(), robot.getIMU(), speed);
    }

    public AutoAlignToTarget(DriveTrain driveTrain, AHRS imu, double speed) {
        this(driveTrain, imu, speed, 2000);
    }

    public AutoAlignToTarget(DriveTrain driveTrain, AHRS imu,  double speed, long timeout) {
        this(driveTrain, imu, speed, timeout, Constants.Auto.Align.TOLERANCE, "");
    }

    public AutoAlignToTarget(Robot robot, long timeout, double tolerance) {
        this(robot.getDriveTrain(), robot.getIMU(), Constants.Auto.Align.SPEED, timeout, tolerance, "");
    }

    public AutoAlignToTarget(Robot robot, double speed, long timeout, double tolerance) {
        this(robot.getDriveTrain(), robot.getIMU(), speed, timeout, tolerance, "");
        _oi = robot.getOI();
    }

    public AutoAlignToTarget(DriveTrain driveTrain, AHRS imu, double speed, long timeout, double tolerance, String message) {
        this(driveTrain, imu, speed, timeout, tolerance, DriveTrainBehavior.bothSides, message);
    }

    public AutoAlignToTarget(Robot robot, double speed, long timeout, double tolerance, DriveTrainBehavior driveTrainBehavior, String message) {
        this(robot.getDriveTrain(), robot.getIMU(), speed, timeout, tolerance, driveTrainBehavior, message);
        _oi = robot.getOI();
    }

    public AutoAlignToTarget(DriveTrain driveTrain, AHRS imu, double speed, long timeout, double tolerance, DriveTrainBehavior driveTrainBehavior, String message) {
        requires(driveTrain);
        this.angle = angle;
        this.speed = speed;
        this.driveTrain = driveTrain;
        this.imu = imu;
        _timeout = timeout;
        _tolerance = tolerance;
        _driveTrainBehavior = driveTrainBehavior;
        _message = message;
    }

    @Override
    protected void initialize() {
        double kP = Constants.Auto.Align.kP; // Double.parseDouble(SmartDashboard.getString("DB/String 0", ".04"));
        double kI = Constants.Auto.Align.kI; // Double.parseDouble(SmartDashboard.getString("DB/String 1", ".006"));
        double kD = Constants.Auto.Align.kD; //Double.parseDouble(SmartDashboard.getString("DB/String 2", ".09"));

        // 1: Read current target angle from limelight
        // 2: Read current yaw from navX
        // 3: Set controller.angle to sum
        _table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = _table.getEntry("tx");

        double limeLightAngle = tx.getDouble(0.0);
        double yawAngle = imu.getAngle();
        angle = limeLightAngle + yawAngle;

        SmartDashboard.putNumber("AutoAlignToTarget/startoffset", limeLightAngle);
        SmartDashboard.putNumber("AutoAlignToTarget/startyaw", yawAngle);
        SmartDashboard.putNumber("AutoAlignToTarget/target", angle);

        controller = new PIDController(kP, kI, kD, imu, this, 0.01);
        controller.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        controller.setOutputRange(-speed, speed);
        controller.setAbsoluteTolerance(_tolerance);
        controller.setContinuous();
        controller.setSetpoint(angle);
        controller.enable();
        SmartDashboard.putNumber("AutoAlignToTarget/setpoint", angle);
        DriverStation.reportError("AutoAlign " + _message + " initialized to " + angle + " at " + speed, false);
        DriverStation.reportError("kP="+kP+" , kI="+kI+", kD="+kD + ",T="+ Constants.Auto.Align.TOLERANCE, false);
        startTimeMillis = System.currentTimeMillis();
        _endTimeMillis = startTimeMillis + _timeout;

    }

    @Override
    protected void execute() {
        NetworkTableEntry tx = _table.getEntry("tx");
        double limeLightAngle = tx.getDouble(0.0);
        double yawAngle = imu.getAngle();
        angle = limeLightAngle + yawAngle;

        SmartDashboard.putNumber("AutoAlignToTarget/startoffset", limeLightAngle);
        SmartDashboard.putNumber("AutoAlignToTarget/startyaw", yawAngle);
        SmartDashboard.putNumber("AutoAlignToTarget/target", angle);

        if (Math.abs(angle - controller.getSetpoint()) > _tolerance) {
            controller.setSetpoint(angle);
            SmartDashboard.putNumber("AutoAlignToTarget/setpoint", angle);
        }




        SmartDashboard.putBoolean("AutoAlignToTarget/onTarget", controller.onTarget());
        SmartDashboard.putNumber("AutoAlignToTarget/yaw", imu.getYaw());

        actOnPidOut();
        //SmartDashboard.putData("AutoAlignToTarget/pid", controller);

    }

    private void actOnPidOut() {
        if (pidOut > 0 && pidOut < Constants.Auto.Align.MINIMUM_SPEED) {
            pidOut = Constants.Auto.Align.MINIMUM_SPEED;
        }
        if (pidOut < 0 && pidOut > -Constants.Auto.Align.MINIMUM_SPEED) {
            pidOut = -Constants.Auto.Align.MINIMUM_SPEED;
        }
        SmartDashboard.putNumber("AutoAlignToTarget/pidOut", pidOut);
        if (_driveTrainBehavior == DriveTrainBehavior.bothSides) {
            driveTrain.setPower(pidOut, -pidOut, true); // positive output is clockwise
        } else if (_driveTrainBehavior == DriveTrainBehavior.rightOnly) {
            driveTrain.setPower(0, -pidOut);
        } else if (_driveTrainBehavior == DriveTrainBehavior.leftOnly) {
            driveTrain.setPower(pidOut, 0);
        }
    }

    @Override
    protected boolean isFinished() {
        if (_aborted) { return true; }
        if (!controller.onTarget()) {
            _onTargetSince = 0;
        }

        if((_oi!=null && !_oi.isAutoTargetPressed()) && System.currentTimeMillis() >= _endTimeMillis){
            DriverStation.reportError("AutoAlignToTarget timed out after " + _timeout + "ms at " + imu.getYaw(), false);
            return true;
        }

        if (controller.onTarget()) {
            if (_onTargetSince == 0) {
                DriverStation.reportError("AutoAlignToTarget reached target " + imu.getYaw(), false);
                _onTargetSince = System.currentTimeMillis();
            }

            if ((_oi!=null && !_oi.isAutoTargetPressed()) && System.currentTimeMillis() > _onTargetSince + Constants.Auto.Align.STEADY_TIME) {
                DriverStation.reportError("AutoAlignToTarget complete after " + Constants.Auto.Align.STEADY_TIME + " at " + imu.getYaw(), false);
                return  true;
            }
        }

        return false;
    }

    @Override
    protected void end() {
        driveTrain.setPower(0,0, true);
        DriverStation.reportError("AutoAlign finished: angle = " + imu.getYaw() + ", time = " + (System.currentTimeMillis() - startTimeMillis), false);
        controller.disable();
        DriverStation.reportError("AutoAlign.end() controller disabled", false);
    }

    @Override
    public void pidWrite(double output) {
        pidOut = output;
//        actOnPidOut();
        //SmartDashboard.putNumber("AutoAlign/pidOut", pidOut);
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    /*
    Used in AutoAlign to select which types of turns to do.
    bothSides will turn normally, or "in place" (hah!)
    leftOnly will use the pidOut to drive the left side of the drivetrain, but the right side will be in 0in/s talon velocity pid mode.
    rightOnly will use the pidOut to drive the right side of the drivetrain, but the left side will be in 0in/s talon velocity pid mode.
     */
    public enum DriveTrainBehavior {
        bothSides,
        leftOnly,
        rightOnly
    }





}