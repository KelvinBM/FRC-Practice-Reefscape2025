package frc.robot;


public final class Constants {

    public static double ClimberInitialSpeed = 0.4;
    public static final int CLIMBER_ADJUSTER_MOTOR_ID = 41;
    public static final int CLIMBER_ROPE_PULLER_MOTOR_ID = 42;// winch


    // driving adjust
    public static final double Kp_DISTANCE = -0.1;// proportional control for distance

    public static final class LimelightConstants {
        // distance calculation
        public static final double GOAL_HEIGHT_INCHES = 6.010101;
        public static final double CAMERA_HEIGHT_INCHES = 0;// height of cam from floor
        public static final double CAMERA_MOUNT_ANGLE_DEG = 0;// angle from center of camera to the floor
        public static final double CAMERA_TO_EDGE_OF_ROBOT_INCHES = 0;
        public static final double kDesiredDistanceOffset_Inches = 20;
    }

    public static final class AlgaeCollectorConstants {
        public static final int ALGAE_ADJUSTER_MOTOR_ID = 51;
        public static final int ALGAE_COLLECTOR_MOTOR_ID = 52;

        public static final int ALGAE_COLLECTOR_LIMIT_SWITCH_PORT = 1;

        public static final int ALGEA_BEAM_BREAKER_PORT = 0;
    }

    public static final class CoralGrabberConstants {
        // public static final int CORAL_ADJUST_MOTOR_ID = 8;
        public static final int CORAL_MOTOR_RIGHT_ID = 21;
        public static final int CORAL_MOTOR_LEFT_ID = 22;

        public static final int BEAM_BREAKER_PORT = 23;
    }

    public static final class ElevatorConstants {
        public static final int ELEVATOR_TOP_MOTOR_ID = 31;
        public static final int ELEVATOR_BOTTOM_MOTOR_ID = 32;
        public static final int ELEVATOR_LIMIT_SWITCH_PORT = 3;// DIO port 3
    }

    public static final class OperatorConstants {
        // controllers
        public static final int BUTTON_BOARD_PORT = 1;


        // button board buttons
        public static final int PORT_1 = 1;
        public static final int PORT_2 = 2;
        public static final int PORT_3 = 3;
        public static final int PORT_4 = 4;
        public static final int PORT_5 = 5;
        public static final int PORT_6 = 6;
        public static final int PORT_7 = 7;
        public static final int PORT_8 = 8;
        public static final int PORT_9 = 9;
        public static final int PORT_10 = 10;
    }
}
