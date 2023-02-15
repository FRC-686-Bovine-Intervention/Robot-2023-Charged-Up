package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.lib.sensorCalibration.PotAndEncoder;

public class ArmHAL {

    private static ArmHAL instance;
    public static ArmHAL getInstance() {if(instance == null){instance = new ArmHAL();}return instance;}

    private final static double kShoulderPotentiometerGearRatio           = 72.0/16.0;
    private final static double kShoulderEncoderGearRatio                 = 72.0/16.0;
    private final static double kShoulderPotentiometerNTurns              = 3.0;    
    private final static double kShoulderAngleAtCalibration               = 0.0;      // TODO: update at calibration
    private final static double kShoulderPotentiometerAngleDegAtCalib     = 30.0;     // TODO: update at calibration
    private final static double kShoulderAbsoluteEncoderAngleDegAtCalib   = 30.0;     // TODO: update at calibration

    private final PotAndEncoder shoulderPotEncoder;
    private final PotAndEncoder.Config shoulderPotAndEncoderConfig;
    private final PotAndEncoder.HAL shoulderPotAndEncoderHAL;

    private final static double kElbowPotentiometerGearRatio              = 64.0/16.0;
    private final static double kElbowEncoderGearRatio                    = 64.0/16.0;
    private final static double kElbowPotentiometerNTurns                 = 3.0;
    private final static double kElbowAngleAtCalibration                  = 0.0;      // TODO: update at calibration
    private final static double kElbowPotentiometerAngleDegAtCalib        = 30.0;     // TODO: update at calibration
    private final static double kElbowAbsoluteEncoderAngleDegAtCalib      = 30.0;     // TODO: update at calibration

    private final PotAndEncoder elbowPotEncoder;
    private final PotAndEncoder.Config elbowPotAndEncoderConfig;
    private final PotAndEncoder.HAL elbowPotAndEncoderHAL;

    private ArmHAL()
    {
        if(RobotBase.isReal())
        {
            shoulderPotAndEncoderHAL = new PotAndEncoder.HAL(Constants.kShoulderAnalogInputPort, Constants.kShoulderEncoderId, kShoulderPotentiometerNTurns, kShoulderPotentiometerAngleDegAtCalib, kShoulderAngleAtCalibration);            
            elbowPotAndEncoderHAL    = new PotAndEncoder.HAL(Constants.kElbowAnalogInputPort, Constants.kElbowEncoderId, kElbowPotentiometerNTurns, kElbowPotentiometerAngleDegAtCalib, kElbowAngleAtCalibration); 
        }
        else
        {
            shoulderPotAndEncoderHAL = null;
            elbowPotAndEncoderHAL = null;
        }
        shoulderPotAndEncoderConfig = new PotAndEncoder.Config(kShoulderPotentiometerGearRatio, kShoulderEncoderGearRatio, kShoulderPotentiometerNTurns, kShoulderAngleAtCalibration, kShoulderPotentiometerAngleDegAtCalib, kShoulderAbsoluteEncoderAngleDegAtCalib, shoulderPotAndEncoderHAL);
        elbowPotAndEncoderConfig = new PotAndEncoder.Config(kElbowPotentiometerGearRatio, kElbowEncoderGearRatio, kElbowPotentiometerNTurns, kElbowAngleAtCalibration, kElbowPotentiometerAngleDegAtCalib, kElbowAbsoluteEncoderAngleDegAtCalib, elbowPotAndEncoderHAL);
        shoulderPotEncoder = new PotAndEncoder(shoulderPotAndEncoderConfig);
        elbowPotEncoder = new PotAndEncoder(elbowPotAndEncoderConfig);
    }

    public PotAndEncoder getShoulderPotEncoder() {return shoulderPotEncoder;}
    public PotAndEncoder getElbowPotEncoder() {return elbowPotEncoder;}
}
