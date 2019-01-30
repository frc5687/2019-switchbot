package org.frc5687.switchbot.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.followers.EncoderFollower;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.subsystems.DriveTrain;
import org.frc5687.switchbot.robot.utils.RioLogger;

public class AutoDrivePath extends Command {
    private double _distance;
    private double _speed;
    private Trajectory _leftTrajectory;
    private Trajectory _rightTrajectory;
    private DistanceFollower _leftFollower;
    private DistanceFollower _rightFollower;

    private DriveTrain _driveTrain;
    private AHRS _imu;

    private PIDController _angleController;
    private PIDListener _anglePID;
    private int _index = 0;

    private double _angleFactor;

    public AutoDrivePath(DriveTrain driveTrain, AHRS imu, double distance, double speed) {

        _driveTrain = driveTrain;
        _speed = speed;
        _imu = imu;
        Waypoint[] points = new Waypoint[] {
                new Waypoint(0, 0, 0),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
                new Waypoint(distance, 0, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
        };
        RioLogger.info(this.getClass().getSimpleName(), "Generating trajectory from 0,0,0 to " + distance + ",0,0 with dt=" + (1.0/Constants.CYCLES_PER_SECOND) + ",  ");
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 1.0 / Constants.CYCLES_PER_SECOND, Constants.DriveTrain.CAP_SPEED_IPS, Constants.DriveTrain.MAX_ACCELERATION_IPSS, Constants.DriveTrain.MAX_JERK_IPSSS);
        _leftTrajectory = Pathfinder.generate(points, config);
        _rightTrajectory = Pathfinder.generate(points, config);

        RioLogger.info(this.getClass().getSimpleName(), _leftTrajectory.length() + " segments.");
/*        for (int i = 0; i < _trajectory.length(); i++) {
            Trajectory.Segment s= _trajectory.get(i);
            DriverStation.reportError("Seg " + i + " x=" + s.x + ", pos=" + s.position + ", vel=" + s.velocity + ", acc="+s.acceleration,false);
        }
*/
    }

    public AutoDrivePath(DriveTrain driveTrain, AHRS imu, String pathName) {
        _leftTrajectory = PathfinderFRC.getTrajectory(pathName + ".right");
        _rightTrajectory = PathfinderFRC.getTrajectory(pathName + ".left");

        RioLogger.info(this.getClass().getSimpleName(), _leftTrajectory.length() + " segments.");

    }

    @Override
    protected void initialize() {
        _driveTrain.resetDriveEncoders();
        _leftFollower = new DistanceFollower(_leftTrajectory);
        _rightFollower = new DistanceFollower(_rightTrajectory);
        _leftFollower.configurePIDVA(0.1, 0.0, 0.001, 1 / Constants.DriveTrain.MAX_SPEED_IPS, 0);
        _rightFollower.configurePIDVA(0.1, 0.0, 0.001, 1 / Constants.DriveTrain.MAX_SPEED_IPS, 0);

        _anglePID = new PIDListener();
        _angleController = new PIDController(Constants.AutoDrivePath.kPangle, Constants.AutoDrivePath.kIangle, Constants.AutoDrivePath.kDangle, _imu, _anglePID, 0.05);
        _angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        _angleController.setAbsoluteTolerance(Constants.AutoDrivePath.ANGLE_TOLERANCE);
        double maxSpeed = _speed * Constants.Auto.Drive.AnglePID.MAX_DIFFERENCE;
        SmartDashboard.putNumber("AutoDrive/angleMaxSpeed", maxSpeed);
        SmartDashboard.putNumber("AutoDrive/setPoint", _driveTrain.getYaw());
        _angleController.setOutputRange(-maxSpeed, maxSpeed);
        _angleController.setContinuous();

        // If an angle is supplied, use that as our setpoint.  Otherwise get the current heading and stick to it!
        _angleController.setSetpoint(_driveTrain.getYaw());
        _angleController.enable();
        _index = 0;
    }

    @Override
    protected void execute() {
        double leftDistance = _driveTrain.getLeftDistance();
        double rightDistance = _driveTrain.getRightDistance();
        _index++;
        RioLogger.info(this.getClass().getSimpleName(), "Left Segment " + _index + " target: " + _leftFollower.getSegment().x + " actual " + leftDistance + " vel=" + _leftFollower.getSegment().velocity);
        RioLogger.info(this.getClass().getSimpleName(), "Right Segment " + _index + " target: " + _rightFollower.getSegment().x + " actual " + rightDistance + " vel=" + _rightFollower.getSegment().velocity);
        double leftSpeed = _leftFollower.calculate(leftDistance);
        double rightSpeed = _rightFollower.calculate(rightDistance);

        double trajectoryHeading = Pathfinder.r2d(_leftFollower.getHeading());
        if (Math.abs(trajectoryHeading-_angleController.getSetpoint())>Constants.AutoDrivePath.ANGLE_TOLERANCE) {
            _angleController.setSetpoint(trajectoryHeading);
        }

        RioLogger.info(this.getClass().getSimpleName(), "Calculated speed: " + leftSpeed + "," + rightSpeed + " angleFactor " + _angleFactor);

        _driveTrain.setPower(leftSpeed -_angleFactor , rightSpeed + _angleFactor, true);
    }

    @Override
    protected boolean isFinished() {
        return _leftFollower.isFinished() || _rightFollower.isFinished();
    }


    private class PIDListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            _angleFactor = output;
        }

    }

}
