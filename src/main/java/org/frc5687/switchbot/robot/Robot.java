package org.frc5687.switchbot.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.switchbot.robot.commands.AutoAlignToTarget;
import org.frc5687.switchbot.robot.commands.AutoGroup;
import org.frc5687.switchbot.robot.subsystems.*;
import org.frc5687.switchbot.robot.utils.AutoChooser;
import org.frc5687.switchbot.robot.utils.Limelight;
import org.frc5687.switchbot.robot.utils.PDP;
import org.frc5687.switchbot.robot.utils.Version;
import org.frc5687.switchbot.robot.utils.RioLogger;

public class Robot extends TimedRobot {

    private static Robot _instance;

    private LEDStrip _ledStrip;
    private DriveTrain _drivetrain;
    private Pincer _pincer;
    private Arm _arm;
    private Shifter _shifter;
    private Limelight _limelight;


    private OI _oi;
    private PDP _pdp;
    private AHRS _imu;
    private UsbCamera _camera0;
    private UsbCamera _camera1;

    private AutoChooser _autoChooser;

    private Command _autoCommand;



    public Robot() {
    }

    @Override
    public void startCompetition() {
        super.startCompetition();
    }

    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        RioLogger.getInstance().init();
        RioLogger.info(this, "Starting " + this.getClass().getCanonicalName() + " from branch " + Version.BRANCH);
        _instance = this;
        // setPeriod(1 / Constants.CYCLES_PER_SECOND);
        _imu = new AHRS(SPI.Port.kMXP, (byte) 100);
        _limelight = new Limelight();

        _pdp = new PDP();
        _oi = new OI();
        _ledStrip = new LEDStrip();
        _drivetrain = new DriveTrain(_instance);
        _pincer = new Pincer(_instance);
        _arm = new Arm(_instance);
        _shifter = new Shifter(_instance);

        _oi.initializeButtons(_instance);
        _autoChooser = new AutoChooser();

        try {
            // _camera0 = CameraServer.getInstance().startAutomaticCapture(0);
            // _camera0.setResolution(320, 240);
            // _camera0.setFPS(10);
        } catch (Exception e) {
            RioLogger.error(this.getClass().getSimpleName(),  e.getMessage());
        }

        try {
            // _camera1 = CameraServer.getInstance().startAutomaticCapture(1);
            // _camera1.setResolution(320, 240);
            // _camera1.setFPS(30);
        } catch (Exception e) {
            RioLogger.error(this.getClass().getSimpleName(), (e.getMessage()));
        }

    }

    @Override
    protected void loopFunc() {
        try {
            super.loopFunc();
        } catch (Throwable throwable) {
            RioLogger.error(this.getClass().getSimpleName(), "Unhandled exception: " + throwable.toString());
            System.exit(1);
        }
    }

    @Override
    public void disabledPeriodic() {
        updateDashboard();
    }

    @Override
    public void autonomousInit() {
        _imu.reset();
        _drivetrain.resetDriveEncoders();
        _drivetrain.enableBrakeMode();
        _drivetrain.setCurrentLimiting(40);
        RioLogger.getInstance().init();
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData==null) { gameData = ""; }
        int retries = 100;
        gameData="LLL";
        SmartDashboard.putString("Auto/gameData", gameData);
        RioLogger.debug(this.getClass().getSimpleName(),  "gameData before parse: " + gameData);
        int switchSide = 0;
        if (gameData.length()>0) {
            switchSide = gameData.charAt(0)=='L' ? Constants.AutoChooser.LEFT : Constants.AutoChooser.RIGHT;
        }
        int autoPosition = _autoChooser.positionSwitchValue();
        SmartDashboard.putNumber("Auto/SwitchSide", switchSide);
        SmartDashboard.putNumber("Auto/Position", autoPosition);
        RioLogger.info(this.getClass().getSimpleName(), "Running AutoGroup with position: " + autoPosition + ",  switchSide: " + switchSide);
        _autoCommand = new AutoGroup(autoPosition, switchSide, this);
        _autoCommand.start();
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        updateDashboard();
    }

    @Override
    public void disabledInit(){
        _drivetrain.enableCoastMode();
        RioLogger.getInstance().forceSync();
        RioLogger.getInstance().close();
    }

    @Override
    public void teleopInit() {
        if (_autoCommand != null) {
            _autoCommand.cancel();
        }
        _drivetrain.enableBrakeMode();
        _drivetrain.setCurrentLimiting(40);
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        _oi.poll();
        _ledStrip.poll();
        updateDashboard();
        // set the motor output based on jostick position

    }

    public void updateDashboard() {
        _pdp.updateDashboard();
        _arm.updateDashboard();
        _autoChooser.updateDashboard();
        _drivetrain.updateDashboard();
        _pincer.updateDashboard();
    }

    public OI getOI() { return _oi; }
    public DriveTrain getDriveTrain() { return _drivetrain; }
    public Pincer getPincer() { return _pincer; }
    public PDP getPDP() { return _pdp; }
    public Arm getArm() { return _arm; }
    public Shifter getShifter() { return _shifter; }
    public AHRS getIMU() { return _imu; }
    public LEDStrip getLEDStrip() { return _ledStrip; }
    public Limelight get_limelight() { return _limelight; }
}
