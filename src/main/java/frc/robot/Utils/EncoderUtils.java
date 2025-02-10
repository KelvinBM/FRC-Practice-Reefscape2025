package frc.robot.Utils;

public class EncoderUtils {
    /**
     * 
     * Using neo motor rotations(encoder.getPosition). Verify if value returned is accurate
     * @param encoderTicks as rotational value of encoder
     * @return encoder ticks converted to feet
     */
    public static double ticks2Feet(double encoderTicks) {
        return (encoderTicks * (6 * Math.PI)) / 12;
    }

    /**
     * 
     * @param encoderTicks
     * @return encoder ticks converted to inches
     */
    public static double ticks2Inches(double encoderTicks) {
        return ticks2Feet(encoderTicks) * 12;
    }
}
