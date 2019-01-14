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
import org.frc5687.switchbot.robot.Constants.Auto.Drive.toTarget;

public class AutoDriveToTarget extends Command {
    private double distance;
    private double speed;
    private PIDController distanceController;
    private PIDController angleController;
    private PIDListener distancePID;
    private PIDListener anglePID;
    private long endMillis;
    private long maxMillis;

    private long settleTime = 40;
    private long settleEnd = 0;
    private boolean stopOnFinish;
    private double angle;

    private String debug;

    private DriveTrain driveTrain;
    private AHRS imu;
    private OI _oi;
    NetworkTable _table;

    private double kPdistance = 0.05; // .05;
    private double kIdistance = 0.000; // .001;
    private double kDdistance = 0.1; //.1;
    private double kTdistance = 0.5;

    private double kPangle = .001;
    private double kIangle = .0001;
    private double kDangle = .001;
    private double kTangle;



    public AutoDriveToTarget(Robot robot, double distance, double speed, long maxMillis, String debug) {
        this(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), distance, speed, true, true, 1000, maxMillis, debug);
    }


    /***
     * Drives for a set distance at a set speed.
     * @param distance Distance to drive
     * @param speed Speed to drive
     * @param usePID Whether to use pid or not
     * @param stopOnFinish Whether to stop the motors when we are done
     * @param angle The angle to drive, in degrees.  Pass 1000 to maintain robot's hading.
     * @param maxMillis Maximum time in millis to allow the command to run
     */
    public AutoDriveToTarget(DriveTrain driveTrain, AHRS imu, OI oi, double distance, double speed, boolean usePID, boolean stopOnFinish, double angle, long maxMillis, String debug) {
        requires(driveTrain);
        this.speed = speed;
        this.distance = distance;
        this.stopOnFinish = stopOnFinish;
        this.angle = angle;
        this.maxMillis = maxMillis;
        this.debug = debug;
        this.driveTrain = driveTrain;
        this.imu = imu;
        this._oi = oi;
    }

    @Override
    protected void initialize() {

        _table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = _table.getEntry("ty");

        double limeLightYAngle = ty.getDouble(0.0);
        double fixedAngle = 0.0;
        double angleY = fixedAngle + limeLightYAngle;
        double tanY = Math.tan(angleY * (Math.PI / 180));
        double currentTargetDistance = (toTarget.TARGET_HEIGHT - toTarget.CAMERA_HEIGHT)/tanY;
        double setpointDistance = driveTrain.getDistance() + (currentTargetDistance - distance);

        this.endMillis = maxMillis == 0 ? Long.MAX_VALUE : System.currentTimeMillis() + maxMillis;
        driveTrain.enableBrakeMode();
        distancePID = new PIDListener();
        SmartDashboard.putNumber("AutoDriveToTarget/kP", kPdistance);
        SmartDashboard.putNumber("AutoDriveToTarget/kI", kIdistance);
        SmartDashboard.putNumber("AutoDriveToTarget/kD", kDdistance);
        SmartDashboard.putNumber("AutoDriveToTarget/kT", kTdistance);

        distanceController = new PIDController(kPdistance, kIdistance, kDdistance, speed, driveTrain, distancePID, 0.01);
        distanceController.setAbsoluteTolerance(kTdistance);
        distanceController.setOutputRange(-speed, speed);
        distanceController.setSetpoint(setpointDistance);
        distanceController.enable();

        NetworkTableEntry tx = _table.getEntry("tx");

        double limeLightAngle = tx.getDouble(0.0);
        double yawAngle = imu.getAngle();
        angle = limeLightAngle + yawAngle;

        anglePID = new PIDListener();
        angleController = new PIDController(kPangle, kIangle, kDangle, imu, anglePID, 0.01);
        angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        double maxSpeed = speed * Constants.Auto.Drive.AnglePID.MAX_DIFFERENCE;
        SmartDashboard.putNumber("AutoDriveToTarget/angleMaxSpeed", maxSpeed);
        SmartDashboard.putNumber("AutoDriveToTarget/setPoint", driveTrain.getYaw());
        angleController.setOutputRange(-maxSpeed, maxSpeed);
        angleController.setContinuous();

        // If an angle is supplied, use that as our setpoint.  Otherwise get the current heading and stick to it!
        angleController.setSetpoint(angle);
        angleController.enable();

        DriverStation.reportError("Auto Drive initialized: " + (debug==null?"":debug), false);
        SmartDashboard.putNumber("AutoDriveToTarget/AngleY", angleY);
        SmartDashboard.putNumber("AutoDriveToTarget/TargetDistance", currentTargetDistance);
        SmartDashboard.putNumber("AutoDriveToTarget/SetpointDistance", setpointDistance);
        SmartDashboard.putNumber("AutoDriveToTarget/startoffset", limeLightAngle);
        SmartDashboard.putNumber("AutoDriveToTarget/startyaw", yawAngle);
        SmartDashboard.putNumber("AutoDriveToTarget/target", angle);
    }

    @Override
    protected void execute() {

        _table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = _table.getEntry("ty");

        double limeLightYAngle = ty.getDouble(0.0);
        double fixedAngle = 0.0;
        double angleY = fixedAngle + limeLightYAngle;
        double tanY = Math.tan(angleY * (Math.PI / 180));
        double currentTargetDistance = (toTarget.TARGET_HEIGHT + toTarget.CAMERA_HEIGHT)/tanY;
        double setpointDistance = driveTrain.getDistance() + (currentTargetDistance - distance);

        double distanceFactor = 0;
        double angleFactor = 0;
        if (Math.abs(setpointDistance - distanceController.getSetpoint()) > 6) {
            distanceController.setSetpoint(setpointDistance);
            SmartDashboard.putNumber("AutoDriveToTarget/SetpointDistance", setpointDistance);
        }

        distanceFactor = distancePID.get();
        if (distanceFactor < 0) { distanceFactor = Math.min(distanceFactor, -0.3); }
        if (distanceFactor > 0) { distanceFactor = Math.max(distanceFactor, 0.3); }
        _table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = _table.getEntry("tx");

        double limeLightAngle = tx.getDouble(0.0);
        double yawAngle = imu.getAngle();
        angle = limeLightAngle + yawAngle;

        angleFactor = anglePID.get();

        SmartDashboard.putNumber("AutoDriveToTarget/distanceFactor", distanceFactor);
        SmartDashboard.putNumber("AutoDriveToTarget/angleFactor", angleFactor);

        driveTrain.setPower(distanceFactor + angleFactor, distanceFactor - angleFactor, true);

        SmartDashboard.putBoolean("AutoDriveToTarget/onTarget", distanceController == null ? false : distanceController.onTarget());
        SmartDashboard.putNumber("AutoDriveToTarget/imu", driveTrain.getYaw());
        SmartDashboard.putNumber("AutoDriveToTarget/distance", driveTrain.pidGet());
        SmartDashboard.putNumber("AutoDriveToTarget/turnPID", anglePID.get());
        if (Math.abs(angle - angleController.getSetpoint()) > 2) {
            angleController.setSetpoint(angle);
            SmartDashboard.putNumber("AutoDriveToTarget/AngleSetpoint", angle);

        }
    }

    @Override
    protected boolean isFinished() {
        if (!_oi.isAutoDrivePressed() && maxMillis>0 && endMillis!=Long.MAX_VALUE && System.currentTimeMillis() > endMillis) {
            DriverStation.reportError("AutoDriveToTarget for " + maxMillis + " timed out.", false);
            return true;
        }
        if (distanceController.onTarget()) {
            if (settleTime == 0) {
                DriverStation.reportError("AutoDriveToTarget nosettle complete at " + driveTrain.getDistance() + " inches", false);
                return true;
            }
            if (settleEnd > 0) {
                if (System.currentTimeMillis() > settleEnd) {
                    DriverStation.reportError("AutoDriveToTarget settled at " + driveTrain.getDistance() + " inches", false);
                    return true;
                }
            } else {
                DriverStation.reportError("AutoDriveToTarget settling for " + settleTime + "ms", false);
                settleEnd = System.currentTimeMillis() + settleTime;
            }
        } else {
            if (settleEnd > 0) {
                DriverStation.reportError("AutoDriveToTarget unsettled at " + driveTrain.getDistance() + " inches", false);
                settleEnd = 0;
            }
        }
        return false;
    }



    @Override
    protected void end() {
        DriverStation.reportError("AutoDriveToTarget Finished (" + driveTrain.getDistance() + ", " + (driveTrain.getYaw() - angleController.getSetpoint()) + ") " + (debug==null?"":debug), false);
        driveTrain.enableCoastMode();
        angleController.disable();
        if (distanceController!=null) {
            distanceController.disable();
        }
        if (stopOnFinish) {
            DriverStation.reportError("Stopping at ." + driveTrain.getDistance(), false);
            driveTrain.enableBrakeMode();
            driveTrain.setPower(0, 0, true);
        }
    }


    private class PIDListener implements PIDOutput {

        private double value;

        public double get() {
            return value;
        }

        @Override
        public void pidWrite(double output) {
            value = output;
        }

    }

}
