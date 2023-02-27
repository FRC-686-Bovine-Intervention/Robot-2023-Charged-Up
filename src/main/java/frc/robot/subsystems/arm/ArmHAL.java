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
    private final static double kShoulderPotentiometerNTurns              = 5.0;    
    private final static double kShoulderAngleAtCalibration               = -90.0;      // calibrated 2/23 (straight down)
    private final static double kShoulderPotNormalizedVoltageAtCalib      = 0.4955;     // calibrated 2/23
    private final static double kShoulderAbsoluteEncoderAngleDegAtCalib   =  85.08;     // calibrated 2/23
    private final static boolean kShoulderPotInverted                     = false;
    private final static boolean kShoulderEncInverted                     = false;

    private final PotAndEncoder shoulderPotEncoder;
    private final PotAndEncoder.Config shoulderPotAndEncoderConfig;
    private final PotAndEncoder.HAL shoulderPotAndEncoderHAL;

    private final static double kElbowPotentiometerGearRatio              = 64.0/16.0;
    private final static double kElbowEncoderGearRatio                    = 64.0/16.0;
    private final static double kElbowPotentiometerNTurns                 = 5.0;
    private final static double kElbowAngleAtCalibration                  = 0.0;       // calibrated 2/23 (straight out)
    private final static double kElbowPotNormalizedVoltageAtCalib         = 0.4931;    // calibrated 2/23
    private final static double kElbowAbsoluteEncoderAngleDegAtCalib      = 125.15;    // calibrated 2/23
    private final static boolean kElbowPotInverted                        = true;
    private final static boolean kElbowEncInverted                        = true;

    private final PotAndEncoder elbowPotEncoder;
    private final PotAndEncoder.Config elbowPotAndEncoderConfig;
    private final PotAndEncoder.HAL elbowPotAndEncoderHAL;

        
    private ArmHAL()
    {
        if(RobotBase.isReal())
        {
            turretMotor = new TalonSRX(Constants.kTurretMotorID);
            shoulderMotor = null;//ARMDEBUGnew WPI_TalonFX(Constants.kShoulderMotorID);
            elbowMotor    = null;//ARMDEBUGnew WPI_TalonFX(Constants.kElbowMotorID);    
            shoulderPotAndEncoderHAL = new PotAndEncoder.HAL(Constants.kShoulderAnalogInputPort, Constants.kShoulderEncoderId, kShoulderPotentiometerNTurns, 
                                                             kShoulderPotentiometerGearRatio, kShoulderPotNormalizedVoltageAtCalib, kShoulderAngleAtCalibration, 
                                                             kShoulderPotInverted, kShoulderEncInverted);            
            elbowPotAndEncoderHAL    = new PotAndEncoder.HAL(Constants.kElbowAnalogInputPort, Constants.kElbowEncoderId, kElbowPotentiometerNTurns, 
                                                            kElbowPotentiometerGearRatio, kElbowPotNormalizedVoltageAtCalib, kElbowAngleAtCalibration, 
                                                            kElbowPotInverted, kElbowEncInverted); 
        }
        else
        {
            turretMotor = null;
            shoulderMotor = null;
            elbowMotor    = null;            
            shoulderPotAndEncoderHAL = new PotAndEncoder.HAL(Constants.kShoulderAnalogInputPort, Constants.kShoulderEncoderId, kShoulderPotentiometerNTurns, 
                                                             kShoulderPotentiometerGearRatio, kShoulderPotNormalizedVoltageAtCalib, kShoulderAngleAtCalibration, 
                                                             kShoulderPotInverted, kShoulderEncInverted);            
            elbowPotAndEncoderHAL    = new PotAndEncoder.HAL(Constants.kElbowAnalogInputPort, Constants.kElbowEncoderId, kElbowPotentiometerNTurns, 
                                                            kElbowPotentiometerGearRatio, kElbowPotNormalizedVoltageAtCalib, kElbowAngleAtCalibration, 
                                                            kElbowPotInverted, kElbowEncInverted); 
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
        shoulderPotAndEncoderConfig = new PotAndEncoder.Config(kShoulderPotentiometerGearRatio, kShoulderEncoderGearRatio, kShoulderPotentiometerNTurns, 
                                                                kShoulderAngleAtCalibration, kShoulderPotNormalizedVoltageAtCalib, kShoulderAbsoluteEncoderAngleDegAtCalib, 
                                                                kShoulderPotInverted, kShoulderEncInverted, shoulderPotAndEncoderHAL);
        elbowPotAndEncoderConfig = new PotAndEncoder.Config(kElbowPotentiometerGearRatio, kElbowEncoderGearRatio, kElbowPotentiometerNTurns, 
                                                                kElbowAngleAtCalibration, kElbowPotNormalizedVoltageAtCalib, kElbowAbsoluteEncoderAngleDegAtCalib, 
                                                                kElbowPotInverted, kElbowEncInverted, elbowPotAndEncoderHAL);
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
        return turretMotor != null ? turretMotor.getSelectedSensorPosition(kRelativePIDId) * kTurretGearRatio * kEncoderUnitsToDegrees : 0;
    }
    public double getTurretAbsolute(){
        return turretMotor != null ? turretMotor.getSelectedSensorPosition(kAbsolutePIDId) * kTurretGearRatio * kEncoderUnitsToDegrees : 0;
    }

    public ArmHAL setShoulderMotor(double power) {
        if (shoulderMotor != null)
            shoulderMotor.set(ControlMode.PercentOutput, power);
        return this;
    }
    public ArmHAL setElbowMotor(double power) {
        if (elbowMotor != null)
            elbowMotor.set(ControlMode.PercentOutput, power);
        return this;
    }
    
    public PotAndEncoder getShoulderPotEncoder() {return shoulderPotEncoder;}
    public PotAndEncoder getElbowPotEncoder() {return elbowPotEncoder;}
}

