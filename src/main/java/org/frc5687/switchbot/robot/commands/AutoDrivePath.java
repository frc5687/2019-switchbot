package org.frc5687.switchbot.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.subsystems.DriveTrain;

public class AutoDrivePath extends Command {
    private double _distance;
    private double _speed;
    private Trajectory _trajectory;
    private DistanceFollower _follower;

    private DriveTrain _driveTrain;
    private AHRS _imu;

    private PIDController _angleController;
    private PIDListener _anglePID;
    private double kPangle = .001;
    private double kIangle = .0001;
    private double kDangle = .001;
    private int _index = 0;

    public AutoDrivePath(DriveTrain driveTrain, AHRS imu, double distance, double speed) {

        _driveTrain = driveTrain;
        _speed = speed;
        _imu = imu;
        Waypoint[] points = new Waypoint[] {
                new Waypoint(0, 0, 0),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
                new Waypoint(distance, 0, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
        };
        DriverStation.reportError("Generating trajctory from 0,0,0 to " + distance + ",0,0 with dt=" + (1.0/Constants.CYCLES_PER_SECOND) + ",  ", false);
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 1.0 / Constants.CYCLES_PER_SECOND, Constants.DriveTrain.CAP_SPEED_IPS, Constants.DriveTrain.MAX_ACCELERATION_IPSS, Constants.DriveTrain.MAX_JERK_IPSSS);
        _trajectory = Pathfinder.generate(points, config);

        DriverStation.reportError("Generated " + _trajectory.length() + " segments.", false);
/*        for (int i = 0; i < _trajectory.length(); i++) {
            Trajectory.Segment s= _trajectory.get(i);
            DriverStation.reportError("Seg " + i + " x=" + s.x + ", pos=" + s.position + ", vel=" + s.velocity + ", acc="+s.acceleration,false);
        }
*/
    }

    @Override
    protected void initialize() {
        _driveTrain.resetDriveEncoders();
        _follower = new DistanceFollower(_trajectory);
        _follower.configurePIDVA(0.1, 0.0, 0.001, 1 / Constants.DriveTrain.MAX_SPEED_IPS, 0);

        _anglePID = new PIDListener();
        _angleController = new PIDController(kPangle, kIangle, kDangle, _imu, _anglePID, 0.05);
        _angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
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
        double distance = _driveTrain.getDistance();
        _index++;
        DriverStation.reportError("Segment " + _index + " target: " + _follower.getSegment().x + " actual " + distance + " vel=" + _follower.getSegment().velocity, false);
        double speed = _follower.calculate(distance);
        double angleFactor = _anglePID.get();

        DriverStation.reportError("Calculated speed: " + speed + " anglFactor " +angleFactor, false);

        _driveTrain.setPower(speed , speed, true);
    }

    @Override
    protected boolean isFinished() {
        return _follower.isFinished();
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
