package frc.robot.lib.sensorCalibration;

public class PotAndEncoderStatus {

    private PotAndEncoderHAL hal;
    private PotAndEncoderCalilbration potAndEncoderCalibration;

    private PotAndEncoderReading reading;
    public PotAndEncoderReading getReading() {return reading;}

    private boolean moving;
    public boolean isMoving() {return moving;}

    private boolean calibrated;
    public boolean isCalibrated() {return calibrated;}
 
    private double positionDeg;
    public double getPositionDeg() {return positionDeg;}

    public PotAndEncoderStatus(PotAndEncoderHAL hal, PotAndEncoderConfig potAndEncoderConfig)
    {
        this.hal = hal;
        this.potAndEncoderCalibration = new PotAndEncoderCalilbration(potAndEncoderConfig);
        this.moving = true;
        this.calibrated = false;
    }

    public void update()
    {
        reading = new PotAndEncoderReading(hal.getPotentiometerReadingDeg(),
                                           hal.getAbsoluteEncoderReadingDeg(),
                                           hal.getRelativeEncoderReadingDeg());

        positionDeg = potAndEncoderCalibration.update(reading);
        moving = potAndEncoderCalibration.isMoving();
        calibrated = potAndEncoderCalibration.isCalibrated();
    }

}
