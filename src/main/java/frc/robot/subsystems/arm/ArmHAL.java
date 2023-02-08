package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.lib.sensorCalibration.PotAndEncoderConfig;
import frc.robot.lib.sensorCalibration.PotAndEncoderHAL;
import frc.robot.lib.sensorCalibration.PotAndEncoderStatus;

public class ArmHAL {

    private static ArmHAL instance;
    public static ArmHAL getInstance() {if(instance == null){instance = new ArmHAL();}return instance;}

    private final static double kShoulderPotentiometerGearRatio           = 72.0/16.0;
    private final static double kShoulderEncoderGearRatio                 = 72.0/16.0;
    private final static double kShoulderPotentiometerNTurns              = 3.0;    
    private final static double kShoulderAngleAtCalibration               = 0.0;      // TODO: update at calibration
    private final static double kShoulderPotentiometerAngleDegAtCalib     = 30.0;     // TODO: update at calibration
    private final static double kShoulderAbsoluteEncoderAngleDegAtCalib   = 30.0;     // TODO: update at calibration

    private final static double kElbowPotentiometerGearRatio              = 64.0/16.0;
    private final static double kElbowEncoderGearRatio                    = 64.0/16.0;
    private final static double kElbowPotentiometerNTurns                 = 3.0;
    private final static double kElbowAngleAtCalibration                  = 0.0;      // TODO: update at calibration
    private final static double kElbowPotentiometerAngleDegAtCalib        = 30.0;     // TODO: update at calibration
    private final static double kElbowAbsoluteEncoderAngleDegAtCalib      = 30.0;     // TODO: update at calibration

    private final PotAndEncoderConfig shoulderPotAndEncoderConfig = new PotAndEncoderConfig(kShoulderPotentiometerGearRatio, kShoulderEncoderGearRatio, 
            kShoulderPotentiometerNTurns, kShoulderAngleAtCalibration, kShoulderPotentiometerAngleDegAtCalib, kShoulderAbsoluteEncoderAngleDegAtCalib);
            
    private final PotAndEncoderConfig elbowPotAndEncoderConfig = new PotAndEncoderConfig(kElbowPotentiometerGearRatio, kElbowEncoderGearRatio, 
            kElbowPotentiometerNTurns, kElbowAngleAtCalibration, kElbowPotentiometerAngleDegAtCalib, kElbowAbsoluteEncoderAngleDegAtCalib);
    
    private final PotAndEncoderHAL shoulderPotAndEncoderHAL;
    private final PotAndEncoderHAL elbowPotAndEncoderHAL;

    public PotAndEncoderStatus getShoulderStatus()
    {
        return shoulder.update();
    }

    private ArmHAL()
    {
        if(RobotBase.isReal())
        {
            shoulderPotAndEncoderHAL = new PotAndEncoderHAL(Constants.kShoulderAnalogInputPort, Constants.kShoulderEncoderId, shoulderPotAndEncoderConfig);            
            elbowPotAndEncoderHAL    = new PotAndEncoderHAL(Constants.kElbowAnalogInputPort, Constants.kElbowEncoderId, elbowPotAndEncoderConfig); 
        }
        else
        {
            shoulderPotAndEncoderHAL = null;
            elbowPotAndEncoderHAL = null;
        }
    }
}
