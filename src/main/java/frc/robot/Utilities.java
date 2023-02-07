package frc.robot;

public class Utilities {

    /**
     * @param value The input that you want to apply a deadband to
     * @param deadband The width of the deadband
     * @return 0 if value is less than deadband, scaled value otherwise
     */
    public static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * @param value The value you want to modify
     * @return Applies a deadband to the input value, and returns the square with the same sign
     */
    public static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
    
    /**
     * @param value The value you want to clamp
     * @param lower The lower limit of the clamp
     * @param upper The upper limit of the clamp
     * @return Clamps the input value between an input range
     */
    public static double clamp(double value, double lower, double upper) {
        if (value < lower) return lower;
        else if (value > upper) return upper;
        else return value;
    }
}
