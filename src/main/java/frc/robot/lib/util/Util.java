package frc.robot.lib.util;

public class Util {
    public static double limit(double value, double minValue, double maxValue) {
        return Math.min(Math.max(value, minValue), maxValue);
    }    

    public static double fmodulo(double value, double period) {
        return Math.IEEEremainder(value, period);
    }
}
