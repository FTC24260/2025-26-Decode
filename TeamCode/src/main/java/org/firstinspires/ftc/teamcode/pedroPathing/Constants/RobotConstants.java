package org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public final class RobotConstants {

    // ----------------------
    // Device Names
    // ----------------------
    public static final class Hardware {
        public static final String LEFT_FRONT = "leftFront";
        public static final String LEFT_REAR = "leftRear";
        public static final String RIGHT_FRONT = "rightFront";
        public static final String RIGHT_REAR = "rightRear";
        public static final String INTAKE_MOTOR      = "intake";
        public static final String SPINDEX_RIGHT     = "spindexRight";
        public static final String SPINDEX_LEFT      = "spindexLeft";
        public static final String COLOR_SENSOR      = "colorSensor";
        public static final String TURRET_MOTOR      = "turet";
        public static final String SHOOTER_L     = "ShooterL";
        public static final String SHOOTER_R     = "ShooterR";

        public static final String HOOD_LEFT         = "hoodLeft";
        public static final String HOOD_RIGHT        = "hoodRight";
        public static final String KICKER            = "kicker";
        public static final String LIMELIGHT_NAME    = "limelight";
    }

    // ----------------------
    // Spindex
    // ----------------------
    public static final class Spindex {
        public static final double INTAKE_1 = 0.55;
        public static final double INTAKE_2 = 0.813;
        public static final double INTAKE_3 = 0.3;
        public static final double SHOOT_1 = 0.68;
        public static final double SHOOT_2 = 0.42;
        public static final double SHOOT_3 = 0.17;
        public static final int SENSOR_IGNORE = 1000;
        public static final int INITIAL_IGNORE = 1500;
    }

    // ----------------------
    // Ball Colors
    // ----------------------
    public static final class Ball {
        public enum Color { PURPLE, GREEN, UNKNOWN }

        public static final double PURPLE_THRESHOLD = 1.2;
        public static final double GREEN_THRESHOLD  = 1.2;
        public static final double CLEAR_THRESHOLD  = 0.08;
    }

    // ----------------------
    // Turret
    // ----------------------
    public static final class Turret {
        public static final double MAX_RPM = 1150;
        public static final double KP      = 0.012;
        public static final double KI      = 0.0;
        public static final double KD      = 0.0004;
        public static final boolean IF_REVERSED = false;
    }

    // ----------------------
    // Shooter
    // ----------------------
    public static final class Shooter {
        public static final double MAX_RPM = 6000.0;
        public static final double KV      = 0.0005;
        public static final double KS      = 0.4;

        // Distance (inches) → power
        public static final double[][] POWER_TABLE = {
                {12, 0.45},
                {24, 0.57},
                {36, 0.66},
                {48, 0.73},
                {60, 0.79}
        };
    }

    // ----------------------
    // Kicker
    // ----------------------
    public static final class Kicker {
        public static final double REST_POS = 0.7;
        public static final double FIRE_POS = 0.5;
    }

    // ----------------------
    // Limelight
    // ----------------------
    public static final class Limelight {
        public static final double CAMERA_HEIGHT = 9.0;      // inches
        public static final double TARGET_HEIGHT = 38.0;     // inches
        public static final double MOUNT_ANGLE   = 22.0;     // degrees
        public static final int GOAL_TAG_ID      = 21;
        public static final double FILTER        = 0.8;
    }
}
