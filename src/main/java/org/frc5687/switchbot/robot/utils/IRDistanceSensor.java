package org.frc5687.switchbot.robot.utils;

import edu.wpi.first.wpilibj.AnalogInput;

public class IRDistanceSensor extends AnalogInput {
    private double _coefficient;
    private double _power;

    public IRDistanceSensor(int channel, Type type) {
        super(channel);
        _coefficient = _coefficients[type.getValue()];
        _power = _powers[type.getValue()];
    }


    public double getRaw() {
        return super.getVoltage();
    }

    public double getDistance() {
        return _coefficient * Math.pow(getRaw(), _power) / 2.54;

    }
    @Override
    public double pidGet() {
        return getDistance();
    }



    public enum Type {
        SHORT(0), // 4-30cm
        MEDIUM(1), // 4-30cm
        LONG(2), // 4-30cm
        ULTRALONG(3);

        private int _value;

        Type(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }

    }

    /**
     * a in the voltage-to-distance equation distance = a * voltage ^ b
     */
    private static final double[] _coefficients = {
            27.385,
            27.385,
            27.385,
            27.385};

    /**
     * b in the voltage-to-distance equation distance = a * voltage ^ b
     */
    private static final double[] _powers = {
            -1.203,
            -1.203,
            -1.203,
            -1.203};

}