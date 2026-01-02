package org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public final class RobotConstants {

    // ----------------------
    // Device Names
    // ----------------------
    public static final class Hardware {
        public static final String INTAKE_MOTOR      = "intake";
        public static final String SPINDEX_RIGHT     = "spindexRight";
        public static final String SPINDEX_LEFT      = "spindexLeft";
        public static final String COLOR_SENSOR      = "colorSensor";
        public static final String TURRET_MOTOR      = "turet";
        public static final String SHOOTER_MOTOR     = "shooter";
        public static final String HOOD_LEFT         = "hoodLeft";
        public static final String HOOD_RIGHT        = "hoodRight";
        public static final String KICKER            = "kicker";
        public static final String LIMELIGHT_NAME    = "limelight";
    }

    // ----------------------
    // Spindex
    // ----------------------
    public static final class Spindex {
        public static final double SLOT_ANGLE_DEG = 120.0;
        public static final double HOME_POS       = 0.0;
        public static final double RIGHT_SCALE    = 1.0;
        public static final double LEFT_SCALE     = -1.0;
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
        public static final boolean REVERSED = false;
    }

    // ----------------------
    // Shooter
    // ----------------------
    public static final class Shooter {
        public static final double MAX_RPM = 6000.0;
        public static final double KP      = 0.0007;
        public static final double KI      = 0.0;
        public static final double KD      = 0.00005;

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
    // Hood
    // ----------------------
    public static final class Hood {
        public static final double MIN_POS = 0.05;
        public static final double MAX_POS = 0.95;
        public static final boolean RIGHT_MIRRORED = true;

        // Distance (inches) → hood angle (degrees)
        public static final double[][] ANGLE_TABLE = {
                {12, 25},
                {24, 32},
                {36, 38},
                {48, 43},
                {60, 47}
        };
    }

    // ----------------------
    // Kicker
    // ----------------------
    public static final class Kicker {
        public static final double REST_POS = 0.15;
        public static final double FIRE_POS = 0.55;
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
