package frc.robot.lib.sensorCalibration;

public class PotAndEncoderConfig {
    public double potentiometerGearRatio;               // gear ratio: potentiometer rotations / output rotations
    public double encoderGearRatio;                     // gear ratio: encoder rotations / output rotations
    public double potentiometerNTurns;                  // number of turns in potentiometers full range of motion
    public double outputAngleDegAtCalibration;          // angle of output at calibration position, in degrees
    public double potentiometerAngleDegAtCalib;         // potentiometer reading at calibration position, in degrees  
    public double absoluteEncoderAngleDegAtCalib;       // absolute endoder reading at calibration position, in degrees
     

    public PotAndEncoderConfig(double potentiometerGearRatio, double encoderGearRatio, double potentiometerNTurns, 
            double outputAngleDegAtCalibration, double potentiometerAngleDegAtCalib, double absoluteEncoderAngleDegAtCalib) {
        this.potentiometerGearRatio = potentiometerGearRatio;
        this.encoderGearRatio = encoderGearRatio;
        this.outputAngleDegAtCalibration = outputAngleDegAtCalibration;
        this.potentiometerAngleDegAtCalib = potentiometerAngleDegAtCalib;
        this.absoluteEncoderAngleDegAtCalib = absoluteEncoderAngleDegAtCalib;
    }
  
}
