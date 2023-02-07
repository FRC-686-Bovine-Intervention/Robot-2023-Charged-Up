package frc.robot.lib.sensorCalibration;

public class PotAndEncoderReading {
    public double potAngleDeg;
    public double absAngleDeg;
    public double relAngleDeg;

    public PotAndEncoderReading(double potAngleDeg, double absAngleDeg, double relAngleDeg)
    {
        this.potAngleDeg = potAngleDeg;
        this.absAngleDeg = absAngleDeg;
        this.relAngleDeg = relAngleDeg;        
    }
}
