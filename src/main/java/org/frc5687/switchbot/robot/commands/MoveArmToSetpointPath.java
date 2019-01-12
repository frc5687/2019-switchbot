package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.OI;
import org.frc5687.switchbot.robot.subsystems.Arm;

/**
 * Created by Ben Bernard on 6/8/2018.
 */
public class MoveArmToSetpointPath extends Command {
    private double _target;
    private Arm _arm;
    private long _timeout = 10000;
    private long _endMillis;
    private OI _oi;
    Trajectory.Config _config;
    Trajectory _trajectory;
    DistanceFollower _follower;

    public MoveArmToSetpointPath(Arm arm, OI oi, double target, long timeout) {
        this.setInterruptible(true);
        requires(arm);
        _arm = arm;
        _target = target;
        _timeout = timeout;
        _oi = oi;
        _config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, Constants.Arm.CAP_SPEED_DPS, 2.0, 60.0);
    }

    @Override
    protected void initialize() {
        super.initialize();
        _arm.disable();
        double _start = _arm.getAngle();
        _endMillis = System.currentTimeMillis() + _timeout;
        DriverStation.reportError("Starting MoveArmToSetpointPath from " + _start + " to " + _target + " for max " + _timeout + "ms", false);

        Waypoint[] points = new Waypoint[] {
                new Waypoint(0, _start, 0),
                new Waypoint(0, _target, 0),
        };

        _trajectory = Pathfinder.generate(points, _config);
        _follower = new DistanceFollower(_trajectory);
        _follower.configurePIDVA(1.0, 0.0, 0.0, 1 / Constants.Arm.MAX_SPEED_DPS, 0);
    }

    @Override
    protected void execute() {
        double speed = _follower.calculate(_arm.getAngle());
        _arm.drive(speed);
    }

    @Override
    protected boolean isFinished() {
        return _follower.isFinished();
    }


    @Override
    protected void end() {
        DriverStation.reportError("MoveArmToSetpointPID Ending", false);
        if (!DriverStation.getInstance().isAutonomous()) {
            _arm.disable();
            _arm.drive(_oi.getArmSpeed());
        } else {
            _arm.setSetpoint(_target);
            _arm.setAbsoluteTolerance(1.0);
            _arm.enable();
        }
    }




}
