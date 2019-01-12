package org.frc5687.switchbot.robot.utils;

/**
 * Created by Ben Bernard on 6/9/2018.
 */

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.switchbot.robot.Constants;
import org.frc5687.switchbot.robot.RobotMap;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by Ben Bernard on 2/2/2018.
 */
public class AutoChooser {
    private RotarySwitch positionSwitch;
    private Map<Integer, String> positionLabels;

    public AutoChooser() {
        positionLabels = new HashMap<Integer, String>();
        positionLabels.put(1, "Far Left");
        positionLabels.put(2, "Near Left");
        positionLabels.put(3, "Center Left");
        positionLabels.put(4, "Center Right");
        positionLabels.put(5, "Near Right");
        positionLabels.put(6, "Far Right");

        positionSwitch = new RotarySwitch(RobotMap.Analog.AUTO_SELECTOR, Constants.RotarySwitch.TOLERANCE, 0.07692, 0.15384, 0.23076, 0.30768, 0.3846, 0.46152, 0.53844, 0.61536, 0.69228, 0.7692, 0.84612, 0.92304);
    }


    public int positionSwitchValue() {
        return positionSwitch.get() + 1;
    }

    private String getPositionLabel() {
        int position = positionSwitchValue();
        return positionLabels.getOrDefault(position, "Unused");
    }

    private String getPositionLabel(int position) {
        return positionLabels.getOrDefault(position, "Unused");
    }


    public void updateDashboard() {
        SmartDashboard.putString("AutoChooser/Label/Position", getPositionLabel());
        SmartDashboard.putNumber("AutoChooser/Raw/Position", positionSwitch.getRaw());
        SmartDashboard.putNumber("AutoChooser/Numeric/Position", positionSwitchValue());
    }
}