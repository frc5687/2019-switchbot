package org.frc5687.switchbot.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.switchbot.robot.LEDColors;
import org.frc5687.switchbot.robot.RobotMap;
import org.frc5687.switchbot.robot.utils.Color;
import org.frc5687.switchbot.robot.utils.LEDController;

/**
 * Subsystem to control lights for vision tracking and shooter aid
 *
 * @author wil
 */
public class LEDStrip extends Subsystem {

    private LEDController redStrip;
    private LEDController greenStrip;
    private LEDController blueStrip;

    private Shifter.Gear _gear;
    private boolean _hasCube;

    private long _overrideMillis = Long.MIN_VALUE;

    public LEDStrip() {
        redStrip = new LEDController(RobotMap.PWM.RED_STRIP);
        greenStrip = new LEDController(RobotMap.PWM.GREEN_STRIP);
        blueStrip = new LEDController(RobotMap.PWM.BLUE_STRIP);
    }

    @Override
    protected void initDefaultCommand() {
    }


    public void poll() {
        if (System.currentTimeMillis() > _overrideMillis) {
            setStripColor(pickColor());
        }
    }

    public Color pickColor() {
        if (_hasCube) { return LEDColors.HAS_CUBE; }
        if (_gear== Shifter.Gear.HIGH) { return LEDColors.HIGH_GEAR; }

        return LEDColors.LOW_GEAR;
    }

    public void setStripColor(int red, int green, int blue) {
        redStrip.setRaw(red);
        greenStrip.setRaw(green);
        blueStrip.setRaw(blue);
        updateDashboard();
    }

    public void setStripColor(Color color) {
        setStripColor(color, false);
    }

    public void setStripColor(Color color, boolean override) {
        redStrip.setRaw(color.getRed());
        greenStrip.setRaw(color.getGreen());
        blueStrip.setRaw(color.getBlue());
        if (override) {
            _overrideMillis = System.currentTimeMillis() + 1000;
        }
        updateDashboard();
    }

    public void clearOverride() {
        _overrideMillis = Long.MIN_VALUE;
    }

    public Color getStripColor() {
        return new Color(redStrip.getRaw(), greenStrip.getRaw(), blueStrip.getRaw());
    }

    public void updateDashboard() {
        try {
            SmartDashboard.putNumber("ledstrip/red", redStrip.getRaw());
            SmartDashboard.putNumber("ledstrip/green", greenStrip.getRaw());
            SmartDashboard.putNumber("ledstrip/blue", blueStrip.getRaw());
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), true);
        }
    }

    public void setGear(Shifter.Gear gear) {
        _gear = gear;
    }


    public void setHasCube(boolean hasCube) {
        _hasCube = hasCube;
    }
}