package frc.robot;

public final class Constants {
    public static final class Limelight {

        // distance calculation
        public static final double GOAL_HEIGHT_INCHES = 6.010101;
        public static final double CAMERA_HEIGHT_INCHES = 0;// height of cam from floor
        public static final double CAMERA_MOUNT_ANGLE_DEG = 0;// angle from center of camera to the floor
        public static final double CAMERA_TO_EDGE_OF_ROBOT_INCHES = 0;

        // driving adjust
        public static final double Kp_DISTANCE = -0.1;// proportional control for distance

    }

    public static final int CLIMBER_MOTOR_ID = 20;
    public static final int CORAL_GRABBER_MOTOR_ID = 8;
    public static double ClimberInitialSpeed = 0.4;
}
