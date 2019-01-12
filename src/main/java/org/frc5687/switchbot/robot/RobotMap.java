package org.frc5687.switchbot.robot;

public class RobotMap {

    public static class CAN {
        public static final int LEFT_MASTER_TALON = 0;
        public static final int RIGHT_MASTER_TALON = 1;
        public static final int LEFT_FOLLOWER_VICTORSPX = 3;
        public static final int RIGHT_FOLLOWER_VICTORSPX = 2;
        public static final int LEFT_INTAKE_VICTORSPX = 0;
        public static final int RIGHT_INTAKE_VICTORSPX = 1;

    }

    public static class PWM {
        public static final int ARM_VICTORSP = 0;
        public static final int RED_STRIP = 10;
        public static final int GREEN_STRIP = 11;
        public static final int BLUE_STRIP = 13;
    }

    public static class PCM {
        public static final int LEFT_PINCER_OPEN = 5;
        public static final int LEFT_PINCER_CLOSE = 1;
        public static final int RIGHT_PINCER_OPEN = 2;
        public static final int RIGHT_PINCER_CLOSE = 3;
        public static final int SHIFTER_HIGH = 4;
        public static final int SHIFTER_LOW = 0;
    }

    public static class PDP {
        public static final int ARM_VICTORSP = 0;
    }

    public static class Analog {
        public static final int ARM_POTENTIOMETER = 0;
        public static final int AUTO_SELECTOR = 1;
        public static final int CUBE_IR = 2;
        public static final int TAPE_IR = 3;
    }

    public static class DIO {
        public static final int ARM_FRONT_LIMIT = 0;
        public static final int ARM_REAR_LIMIT = 1;
    }

}
