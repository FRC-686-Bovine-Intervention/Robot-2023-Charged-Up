package frc.robot.lib.util;

public class Unwrapper 
{
    private double initialValue;
    private double prevIn;
    private double prevOut;
    private double period;

    public Unwrapper(double initialValue, double period) {
        this.initialValue = initialValue;
        this.prevIn = initialValue;
        this.prevOut = 0;
        this.period = period;
    }

    public void reset()
    {
        prevIn = initialValue;
        prevOut = 0;
    }

    public double unwrap(double in)
    {
        double deltaIn = in - prevIn;

        // unwrap jumps larger than 1/2 period
        if (deltaIn > period/2.0) {
            deltaIn -= period;
        } else if (deltaIn < -period/2.0) {
            deltaIn += period;
        }

        double out = prevOut + deltaIn;

        // store values for next iteration
        prevIn = in;
        prevOut = out;

        return out;
    }
}