package org.frc5687.switchbot.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.Robot;
import org.frc5687.switchbot.robot.subsystems.Shifter;


/**
 * Created by Ben Bernard on 2/2/2018.
 */
public class AutoGroup extends CommandGroup {
    public AutoGroup(int position, int switchSide, Robot robot) {
        super();
        int switchFactor = switchSide * (position );

        addSequential(new Shift(robot.getDriveTrain(), robot.getShifter(), Shifter.Gear.LOW, true));
        addSequential(new ClosePincer(robot.getPincer()));
        addParallel(new HoldPincer(robot.getPincer()));

        SmartDashboard.putString("Auto/Mode", "Switch Only");
        switch(switchFactor) {
            case -Constants.AutoChooser.Position.FAR_LEFT: // Position 1, left side
                buildFarLeftCube(robot);
                break;
            case Constants.AutoChooser.Position.FAR_LEFT:  // Position 1, right side
                buildAutoCross(robot);
                break;
            case -Constants.AutoChooser.Position.MID_LEFT: // Position 2, left side:
                straightSwitch(robot);
                break;
            case Constants.AutoChooser.Position.MID_LEFT: // Position 2, right side
                buildAutoCross(robot);
                break;
            case -Constants.AutoChooser.Position.CENTER_LEFT: // Position 3, left side
                buildCenterLeftCube(robot);
                break;
            case Constants.AutoChooser.Position.CENTER_LEFT: // Position 3, right side
                buildCenterRightCube(robot);
                break;
            case -Constants.AutoChooser.Position.CENTER_RIGHT: // Position 4, left side
                buildAutoCross(robot);
                break;
            case Constants.AutoChooser.Position.CENTER_RIGHT: // Position 4, right side
                straightSwitch(robot);
                break;
            case -Constants.AutoChooser.Position.NEAR_RIGHT: // Position 5, left side
            case Constants.AutoChooser.Position.NEAR_RIGHT: // Position 5, right side
                buildAutoCross(robot);
                break;
            case -Constants.AutoChooser.Position.FAR_RIGHT: // Position 6, left side
                buildAutoCross(robot);
                break;
            case Constants.AutoChooser.Position.FAR_RIGHT: // Position 6, left side
                buildFarRightCube(robot);
                break;
            case 7:
            case -7:
                addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 72, 0.6, true, true, 10000, "AutoCross"));
                break;
            case 8:
            case -8:
                addSequential(new AutoDrivePath(robot.getDriveTrain(), robot.getIMU(), 72, 1.0));
                break;
            case -12: // Position 6, left side
                this.addSequential(new AutoAlign(robot, -90, 1.0));
                break;
            case 12: // Position 6, left side
                this.addSequential(new AutoAlign(robot, 90, 1.0));
                break;
            default:
                buildAutoCross(robot);
                break;
        }
    }

    private void buildAutoCross(Robot robot) {
        SmartDashboard.putString("AAutoRun", "AutoCross");

        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 160, .60, true, true, 5000, "AutoCross"));
        return;
    }

    private void buildCenterLeftCube(Robot robot) {
        SmartDashboard.putString("AAutoRun", "CenterLeftSwitch");

        // First attack - delivering starter cube to left side
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.UP));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 8, 0.4, true, true, 1000, "Attack1"));
        addSequential(new AutoAlign(robot, -24, 1.0, 5000, 1.0, AutoAlign.DriveTrainBehavior.bothSides, "left 22deg"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 110, 0.6, true, true, 5000, "Attack2"));
        // addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 0, 0, true, true, 5000, "Stop"));
        addSequential(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.FRONT_SWITCH));
        addSequential(new AutoAlign(robot, 0, 1.0, 2000, 1.0, AutoAlign.DriveTrainBehavior.bothSides, "face switch"));

        addSequential(new Eject(robot.getPincer(), null));

/*        // Retreat to centerline of cube zone
        addSequential(new AutoAlign(robot, -25, .60, 500, 1.0, AutoAlign.DriveTrainBehavior.leftOnly, "realign 2"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), -60, .75, true, true, 5000, "Retreat 2"));
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.FRONT_FLAT));
        addSequential(new AutoAlign(robot, 0.0, .60, 500, 1.0, AutoAlign.DriveTrainBehavior.bothSides, "target 1"));

        // Approach the center cube
        addParallel(new OpenPincer(robot.getPincer()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 20, .75, true, true, 5000, "Approach1"));
        addSequential(new ClosePincer(robot.getPincer()));
        addParallel(new HoldPincer(robot.getPincer()));

        // Retrieve
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 7, .6, true, true, 5000, "Retrieve"));

        // Turn and attack with second cube
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.FRONT_SWITCH));
        addSequential(new AutoAlign(robot, -70, .60, 500, 1.0, AutoAlign.DriveTrainBehavior.bothSides, "realign 3"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 50, .6, true, true, 5000, "Attack2"));
        addSequential(new AutoAlign(robot, 0, .40, 500, 1.0, AutoAlign.DriveTrainBehavior.bothSides, "realign 4"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 16, .4, true, true, 5000, "Attack3"));
        addSequential(new Eject(robot.getPincer(), null));

        // Align and retreat to oblique angle for 3d cube
        addSequential(new AutoAlign(robot, 0, .60, 500, 1.0, AutoAlign.DriveTrainBehavior.bothSides, ""));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), -24, .75, true, true, 5000, "Retreat2"));
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.FRONT_FLAT));

        // Align to 3rd cube and approach
        addSequential(new AutoAlign(robot, 45, .60, 500, 1.0, AutoAlign.DriveTrainBehavior.bothSides, ""));
        addParallel(new OpenPincer(robot.getPincer()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 28, .5, true, true, 5000, "Approach2"));

        // Hold onto 3rd cube!
        addSequential(new ClosePincer(robot.getPincer()));
        addParallel(new HoldPincer(robot.getPincer()));

       // Retrieve and align
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), -20 , .75, true, true, 5000, "Retrieve2"));
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.FRONT_SWITCH));
        addSequential(new AutoAlign(robot, 0, .60, 500, 1.0, AutoAlign.DriveTrainBehavior.bothSides, ""));

        // Attack with 3rd cube
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 24, .75, true, true, 5000, "Attack3"));
        addSequential(new Eject(robot.getPincer()));
*/
        return;
    }

    private void buildCenterRightCube(Robot robot) {
        SmartDashboard.putString("AAutoRun", "CenterRightSwitch");

        // First attack - delivering starter cube to right side
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.UP));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 8, 0.4, true, true, 1000, "Attack1"));
        addSequential(new AutoAlign(robot, 24, 1.0, 5000, 1.0, AutoAlign.DriveTrainBehavior.bothSides, "left 22deg"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 110, 0.5, true, true, 5000, "Attack2"));
        // addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 0, 0, true, true, 5000, "Stop"));
        addSequential(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.FRONT_SWITCH));
        addSequential(new AutoAlign(robot, 0, 1.0, 2000, 1.0, AutoAlign.DriveTrainBehavior.bothSides, "face switch"));

        addSequential(new Eject(robot.getPincer(), null));

/*        // Retreat to centerline of cube zone
        addSequential(new AutoAlign(robot, 35, .60, 500, 1.0, AutoAlign.DriveTrainBehavior.leftOnly, "realign 2"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), -60, .75, true, true, 5000, "Retreat 2"));
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.FRONT_FLAT));
        addSequential(new AutoAlign(robot, 0.0, .60, 500, 1.0, AutoAlign.DriveTrainBehavior.bothSides, "target 1"));

        // Approach the center cube
        addParallel(new OpenPincer(robot.getPincer()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 20, .75, true, true, 5000, "Approach1"));
        addSequential(new ClosePincer(robot.getPincer()));
        addParallel(new HoldPincer(robot.getPincer()));

        // Retrieve
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 7, .6, true, true, 5000, "Retrieve"));

        // Turn and attack with second cube
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.FRONT_SWITCH));
        addSequential(new AutoAlign(robot, 70, .60, 500, 1.0, AutoAlign.DriveTrainBehavior.bothSides, "realign 3"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 50, .6, true, true, 5000, "Attack2"));
        addSequential(new AutoAlign(robot, 0, .40, 500, 1.0, AutoAlign.DriveTrainBehavior.bothSides, "realign 4"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 10, .4, true, true, 5000, "Attack3"));
        addSequential(new Eject(robot.getPincer(), null));

        // Align and retreat to oblique angle for 3d cube
        addSequential(new AutoAlign(robot, 0, .60, 500, 1.0, AutoAlign.DriveTrainBehavior.bothSides, ""));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), -24, .75, true, true, 5000, "Retreat2"));
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.FRONT_FLAT));
*/
        return;

    }

    private void buildFarLeftCube(Robot robot) {
        SmartDashboard.putString("AAutoRun", "FarLeftSwitch");

        // First attack - delivering starter cube to left side
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.UP));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 160, 0.6, true, true, 5000, "Attack1"));
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.FRONT_SWITCH));
        addSequential(new AutoAlign(robot, 90, 1.0, 2000, 1.0, AutoAlign.DriveTrainBehavior.bothSides, "right 90deg"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 24, 1.0, true, true, 3000, "Attack2"));
        addSequential(new Eject(robot.getPincer(), null));
    }

    private void buildFarRightCube(Robot robot) {
        SmartDashboard.putString("AAutoRun", "FarRightSwitch");
        // First attack - delivering starter cube to left side
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.UP));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 160, 0.6, true, true, 5000, "Attack1"));
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.FRONT_SWITCH));
        addSequential(new AutoAlign(robot, -90, 1.0, 2000, 1.0, AutoAlign.DriveTrainBehavior.bothSides, "left 90deg"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 24, 1.0, true, true, 3000, "Attack2"));
        addSequential(new Eject(robot.getPincer(), null));
    }



    private void straightSwitch(Robot robot) {
        SmartDashboard.putString("AAutoRun", "StraightSwitch");
        addParallel(new MoveArmToSetpoint(robot.getArm(), null, Constants.Arm.FRONT_SWITCH));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), 110, .60, true, true, 5000, "StraightSwitch"));
        addSequential(new Eject(robot.getPincer(), null));
        return;
    }


}