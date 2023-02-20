package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.lib.sensorCalibration.PotAndEncoder;

public class ArmHAL {

    private static ArmHAL instance;
    public static ArmHAL getInstance() {if(instance == null){instance = new ArmHAL();}return instance;}

    private final WPI_TalonFX shoulderMotor, elbowMotor;

    public static final TalonFXInvertType kShoulderMotorInverted    = TalonFXInvertType.Clockwise;          // TODO: UPDATE!!!
    public static final TalonFXInvertType kElbowMotorInverted       = TalonFXInvertType.CounterClockwise;   // TODO: UPDATE!!!

    public static final double kArmCurrentLimit = 25;
    public static final double kArmTriggerThresholdCurrent = 20;
    public static final double kArmTriggerThresholdTime = 0.5;

	// Constant import
	public static final int kTalonTimeoutMs = 5;
	public static final int kTalonPidIdx = 0;  

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
            shoulderMotor = new WPI_TalonFX(Constants.kShoulderMotorID);
            elbowMotor    = new WPI_TalonFX(Constants.kElbowMotorID);            
            shoulderPotAndEncoderHAL = new PotAndEncoder.HAL(Constants.kShoulderAnalogInputPort, Constants.kShoulderEncoderId, kShoulderPotentiometerNTurns, kShoulderPotentiometerAngleDegAtCalib, kShoulderAngleAtCalibration);            
            elbowPotAndEncoderHAL    = new PotAndEncoder.HAL(Constants.kElbowAnalogInputPort, Constants.kElbowEncoderId, kElbowPotentiometerNTurns, kElbowPotentiometerAngleDegAtCalib, kElbowAngleAtCalibration); 
        }
        else
        {
            shoulderMotor = null;
            elbowMotor    = null;            
            shoulderPotAndEncoderHAL = null;
            elbowPotAndEncoderHAL = null;
        }
        shoulderPotAndEncoderConfig = new PotAndEncoder.Config(kShoulderPotentiometerGearRatio, kShoulderEncoderGearRatio, kShoulderPotentiometerNTurns, kShoulderAngleAtCalibration, kShoulderPotentiometerAngleDegAtCalib, kShoulderAbsoluteEncoderAngleDegAtCalib, shoulderPotAndEncoderHAL);
        elbowPotAndEncoderConfig = new PotAndEncoder.Config(kElbowPotentiometerGearRatio, kElbowEncoderGearRatio, kElbowPotentiometerNTurns, kElbowAngleAtCalibration, kElbowPotentiometerAngleDegAtCalib, kElbowAbsoluteEncoderAngleDegAtCalib, elbowPotAndEncoderHAL);
        shoulderPotEncoder = new PotAndEncoder(shoulderPotAndEncoderConfig);
        elbowPotEncoder = new PotAndEncoder(elbowPotAndEncoderConfig);

        if(shoulderMotor != null)
        {
            shoulderMotor.configFactoryDefault();
            
            // Set up the encoders
            shoulderMotor.setInverted(kShoulderMotorInverted);           
            shoulderMotor.setSensorPhase(true);
            shoulderMotor.setNeutralMode(NeutralMode.Brake);
            shoulderMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, kArmCurrentLimit, kArmTriggerThresholdCurrent, kArmTriggerThresholdTime));
        }  

        if(elbowMotor != null)
        {
            elbowMotor.configFactoryDefault();
            
            // Set up the encoders
            elbowMotor.setInverted(kElbowMotorInverted);           
            elbowMotor.setSensorPhase(true);
            elbowMotor.setNeutralMode(NeutralMode.Brake);
            elbowMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, kArmCurrentLimit, kArmTriggerThresholdCurrent, kArmTriggerThresholdTime));
       }
    }

    public void setShoulderMotorVoltage(double volts) {shoulderMotor.set(ControlMode.PercentOutput, volts/12.0);}
    public void setElbowMotorVoltage(double volts) {elbowMotor.set(ControlMode.PercentOutput, volts/12.0);}
    

    public PotAndEncoder getShoulderPotEncoder() {return shoulderPotEncoder;}
    public PotAndEncoder getElbowPotEncoder() {return elbowPotEncoder;}
}

