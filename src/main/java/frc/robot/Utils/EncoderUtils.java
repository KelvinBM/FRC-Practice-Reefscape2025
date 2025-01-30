package frc.robot.Utils;

import frc.robot.generated.TunerConstants;

public class EncoderUtils {
    public static double ticks2Feet(double encoderTicks) {
        double wheelRadius = TunerConstants.BackLeft.WheelRadius;
        return (encoderTicks * (wheelRadius * Math.PI)) / TunerConstants.BackLeft.DriveMotorGearRatio;
    }

    public static double ticks2Inches(double encoderTicks) {
        return ticks2Feet(encoderTicks) * 12;
    }
}
