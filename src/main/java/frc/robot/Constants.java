package frc.robot;

public final class Constants {

    public static double ClimberInitialSpeed = 0.4;
    public static final int CLIMBER_MOTOR_ID = 20;

    public static final class LimelightConstants {
        // distance calculation
        public static final double GOAL_HEIGHT_INCHES = 6.010101;
        public static final double CAMERA_HEIGHT_INCHES = 0;// height of cam from floor
        public static final double CAMERA_MOUNT_ANGLE_DEG = 0;// angle from center of camera to the floor
        public static final double CAMERA_TO_EDGE_OF_ROBOT_INCHES = 0;
    }

    // driving adjust
    public static final double Kp_DISTANCE = -0.1;// proportional control for distance

    public static final class CoralGrabberConstants {
        public static final int CORAL_ADJUST_MOTOR_ID = 8;
        public static final int CORAL_MOTOR_RIGHT_ID = 0;
        public static final int CORAL_MOTOR_LEFT_ID = 0;
    }
}
