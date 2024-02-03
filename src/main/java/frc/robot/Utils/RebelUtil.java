package frc.robot.Utils;

public class RebelUtil {
    public static final double EPSILON = 1e-12;

    public static double constrain (double toConstrain, double min, double max) {
        if (toConstrain > max) {
            return max;
        }
        if (toConstrain < min) {
            return min;
        }
        return toConstrain;
    }
}