package frc.robot.subsystems.arm;

import frc.robot.lib.sensorCalibration.PotAndEncoderConfig;
import frc.robot.subsystems.framework.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm instance;
    public static Arm getInstance(){if(instance == null){instance = new Arm();}return instance;}

    private double shoulderPotentiometerGearRatio           = 72.0/16.0;
    private double shoulderEncoderGearRatio                 = 72.0/16.0;
    private double shoulderPotentiometerNTurns              = 3.0;    
    private double shoulderAngleAtCalibration               = 0.0;      // TODO: update at calibration
    private double shoulderPotentiometerAngleDegAtCalib     = 30.0;     // TODO: update at calibration
    private double shoulderAbsoluteEncoderAngleDegAtCalib   = 30.0;     // TODO: update at calibration

    private double elbowPotentiometerGearRatio              = 64.0/16.0;
    private double elbowEncoderGearRatio                    = 64.0/16.0;
    private double elbowPotentiometerNTurns                 = 3.0;
    private double elbowAngleAtCalibration                  = 0.0;      // TODO: update at calibration
    private double elbowPotentiometerAngleDegAtCalib        = 30.0;     // TODO: update at calibration
    private double elbowAbsoluteEncoderAngleDegAtCalib      = 30.0;     // TODO: update at calibration

    private PotAndEncoderConfig shoulderPotAndEncoderConfig = new PotAndEncoderConfig(shoulderPotentiometerGearRatio, shoulderEncoderGearRatio, 
            shoulderPotentiometerNTurns, shoulderAngleAtCalibration, shoulderPotentiometerAngleDegAtCalib, shoulderAbsoluteEncoderAngleDegAtCalib);

            
    private PotAndEncoderConfig elbowPotAndEncoderConfig = new PotAndEncoderConfig(elbowPotentiometerGearRatio, elbowEncoderGearRatio, 
            elbowPotentiometerNTurns, elbowAngleAtCalibration, elbowPotentiometerAngleDegAtCalib, elbowAbsoluteEncoderAngleDegAtCalib);
    
    public PotAndEncoderConfig getShoulderPotAndEncoderConfig() {
        return shoulderPotAndEncoderConfig;
    }

    public PotAndEncoderConfig getElbowPotAndEncoderConfig() {
        return elbowPotAndEncoderConfig;
    }
    
    @Override
    public void init() {
        Loop = ArmLoop.getInstance();
        Status = ArmStatus.getInstance();
    }
}
