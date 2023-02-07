package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.lib.sensorCalibration.PotAndEncoderHAL;

public class ArmHAL {

    private static ArmHAL instance;
    public static ArmHAL getInstance() {if(instance == null){instance = new ArmHAL();}return instance;}

    private PotAndEncoderHAL shoulderPotAndEncoderHAL;
    public PotAndEncoderHAL getShoulderPotAndEncoderHAL() {
        return shoulderPotAndEncoderHAL;
    }

    private PotAndEncoderHAL elbowPotAndEncoderHAL;
    public PotAndEncoderHAL getElbowPotAndEncoderHAL() {
        return elbowPotAndEncoderHAL;
    }

    




    private ArmHAL()
    {
        if(RobotBase.isReal())
        {
            shoulderPotAndEncoderHAL = new PotAndEncoderHAL(Constants.kShoulderAnalogInputPort, Constants.kShoulderEncoderId, Arm.getInstance().getShoulderPotAndEncoderConfig());            
            elbowPotAndEncoderHAL    = new PotAndEncoderHAL(Constants.kElbowAnalogInputPort, Constants.kElbowEncoderId, Arm.getInstance().getElbowPotAndEncoderConfig()); 
        }
        else
        {
            shoulderPotAndEncoderHAL = null;
            elbowPotAndEncoderHAL = null;
        }
    }
}
