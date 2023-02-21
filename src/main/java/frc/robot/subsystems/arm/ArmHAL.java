package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.lib.sensorCalibration.PotAndEncoder;

public class ArmHAL {

    private static ArmHAL instance;
    public static ArmHAL getInstance() {if(instance == null){instance = new ArmHAL();}return instance;}

    private final TalonSRX turretMotor;

    private static final double kEncoderUnitsToDegrees = 360.0 / 4096.0;
    private static final double kTurretGearRatio = 1; // Gear ratio is 1:1 because of worm gear
    private static final boolean kTurretMotorInverted = true;
    private static final boolean kTurretEncoderInverted = true;
    private static final int kRelativePIDId = 0;
    private static final int kAbsolutePIDId = 1;

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
            turretMotor = new TalonSRX(Constants.kTurretMotorID);
            shoulderPotAndEncoderHAL = null;//new PotAndEncoder.HAL(Constants.kShoulderAnalogInputPort, Constants.kShoulderEncoderId, kShoulderPotentiometerNTurns, kShoulderPotentiometerAngleDegAtCalib, kShoulderAngleAtCalibration);            
            elbowPotAndEncoderHAL    = null;//new PotAndEncoder.HAL(Constants.kElbowAnalogInputPort, Constants.kElbowEncoderId, kElbowPotentiometerNTurns, kElbowPotentiometerAngleDegAtCalib, kElbowAngleAtCalibration); 
            shoulderMotor = new WPI_TalonFX(Constants.kShoulderMotorID);
            elbowMotor    = new WPI_TalonFX(Constants.kElbowMotorID);            
        }
        else
        {
            turretMotor = null;
            shoulderMotor = null;
            elbowMotor    = null;            
            shoulderPotAndEncoderHAL = null;
            elbowPotAndEncoderHAL = null;
        }

        if (turretMotor != null) {
            turretMotor.configFactoryDefault();
            turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kRelativePIDId, 0);
            turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kAbsolutePIDId, 0);
            if(turretMotor.getSelectedSensorPosition(kRelativePIDId) == 0) // Reset relative to absolute only on power on
                turretMotor.getSensorCollection().syncQuadratureWithPulseWidth(0, 0, true);
            turretMotor.setSensorPhase(kTurretEncoderInverted);
            turretMotor.setInverted(kTurretMotorInverted);
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

    public ArmHAL setTurretPower(double power){
        if (turretMotor != null) {
            turretMotor.set(ControlMode.PercentOutput, power);
        }
        return this;
    }

    public double getTurretRelative(){
        return turretMotor != null ? turretMotor.getSelectedSensorPosition(kRelativePIDId) * kTurretGearRatio * kEncoderUnitsToDegrees : 0; // Gear ratio is 1:1 because of worm gear
    }
    public double getTurretAbsolute(){
        return turretMotor != null ? turretMotor.getSelectedSensorPosition(kAbsolutePIDId) * kTurretGearRatio * kEncoderUnitsToDegrees : 0; // Gear ratio is 1:1 because of worm gear
    }

    public void setShoulderMotorVoltage(double volts) {shoulderMotor.set(ControlMode.PercentOutput, volts/12.0);}
    public void setElbowMotorVoltage(double volts) {elbowMotor.set(ControlMode.PercentOutput, volts/12.0);}
    
    public PotAndEncoder getShoulderPotEncoder() {return shoulderPotEncoder;}
    public PotAndEncoder getElbowPotEncoder() {return elbowPotEncoder;}
}

