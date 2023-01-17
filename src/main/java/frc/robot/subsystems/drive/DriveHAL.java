package frc.robot.subsystems.drive;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;
import frc.robot.subsystems.framework.HALBase;

public class DriveHAL extends HALBase {
    private static DriveHAL instance;
    public static DriveHAL getInstance(){if(instance == null){instance = new DriveHAL();}return instance;}

    private static final int kVelocityControlSlot = 0;
	private static final int kPositionControlSlot = 1;
	private static final int kMotionMagicControlSlot = 2;

	// Motor Controller Inversions
	
    // public static final TalonFXInvertType kLeftMotorInverted = TalonFXInvertType.CounterClockwise;
    // public static final TalonFXInvertType kRightMotorInverted = TalonFXInvertType.Clockwise;
    public static final boolean kLeftMotorInverted = false;
    public static final boolean kRightMotorInverted = true;

    public static final int kDriveTrainCurrentLimit = 25;

	// Constant import
	public static final int kTalonTimeoutMs = 5;
	public static final int kTalonPidIdx = 0;

	public enum GyroSelectionEnum { BNO055, NAVX, PIGEON; }
    public static final GyroSelectionEnum GyroSelection = GyroSelectionEnum.PIGEON;

    //TODO: Update Drive Coefficients
	// Wheels
	public static final double kDriveWheelCircumInches    = 6*Math.PI*190.5/188.2;
	public static final double kTrackWidthInches          = 22.000;
	public static final double kTrackEffectiveDiameter    = 22.5; //Went 707in in 10 rotations       (kTrackWidthInches * kTrackWidthInches + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
	public static final double kTrackScrubFactor          = 1.0;

	// Wheel Encoder
	public static final int    kTalonFXEncoderUnitsPerRev    = 2048*2;
	public static final double kDriveGearRatio				= (50.0/14.0)*(50.0/14.0);
	public static final double kFalconEncoderStatusFramePeriod = 0.100;	// 100 ms

	// CONTROL LOOP GAINS   
	public static final double kCalEncoderUnitsPer100ms = 1400;		// velocity at a nominal throttle (measured using NI web interface)
	public static final double kCalPercentOutput 		 = 0.49;	// percent output of motor at kCalEncoderPulsePer100ms (using NI web interface)
   
    // CONTROL LOOP GAINS
    public static final double kFullThrottlePercentOutput = 1.0;	
    public static final double kFullThrottleEncoderUnitsPer100ms = 2900; 

    // PID gains for drive velocity loop (sent to Talon)
    // Units: error is 2048 counts/rev.  Max output is +/- 1023 units
    public static final double kDriveVelocityKf = kCalPercentOutput * 1023.0 / kCalEncoderUnitsPer100ms;
    public static final double kDriveVelocityKp = 0.3;
    public static final double kDriveVelocityKi = 0.0;
    public static final double kDriveVelocityKd = 5.0;
    public static final int    kDriveVelocityIZone = 0;
    public static final double kDriveVelocityRampRate = 0;	// seconds from zero to full speed
    public static final int    kDriveVelocityAllowableError = 0;

    // PID gains for drive position loop
    // Units: error is 2048 counts/rev. Max output is +/- 1023 units.
    public static final double kDrivePositionKf = 0;
    public static final double kDrivePositionKp = 0.0001;
    public static final double kDrivePositionKi = 0;
    public static final double kDrivePositionKd = 0;
    public static final int    kDrivePositionIZone = 0;
    public static final double kDrivePositionRampRate = 0;
    public static final int    kDrivePositionAllowableError = 0;

    // PID gains for motion magic (motion profiled position)
    // Units: error is 2048 counts/rev. Max output is +/- 1023 units.
    public static final double kDriveMotionMagicKf = 0;
    public static final double kDriveMotionMagicKp = 0.20;
    public static final double kDriveMotionMagicKi = 0;
    public static final double kDriveMotionMagicKd = 0;
    public static final int    kDriveMotionMagicIZone = 0;
    public static final double kDriveMotionMagicRampRate = 0;
    public static final int    kDriveMotionMagicAllowableError = 0;

    // PID gains for constant heading velocity control
    // Units: Error is degrees. Output is inches/second difference to
    // left/right.
    public static final double kDriveHeadingVelocityKp = 4.0;
    public static final double kDriveHeadingVelocityKi = 0.0;
    public static final double kDriveHeadingVelocityKd = 0.0;//50.0;
    
    // Point Turn constants
    public static final double kPointTurnKp = 0.05;
    public static final double kPointTurnKd = 0.50;
    public static final double kPointTurnKi = 0.00;
    public static final double kPointTurnKf = 0.00;
    public static final double kPointTurnCompletionToleranceDeg = 3.0;
    public static final double kPointTurnMaxOutput = 0.7; 
    
    // Path following constants
    public static final double kPathFollowingMaxVel    = 36.0; 	// inches/sec  		
    public static final double kPathFollowingAccelTime = 0.25;	// sec to reach max velocity
    public static final double kPathFollowingMaxAccel  = kPathFollowingMaxVel / kPathFollowingAccelTime; // inches/sec^2
    public static final double kPathFollowingLookahead = 24.0; // inches
    public static final double kPathFollowingCompletionTolerance = 4.0; 

	public static final double kDriveOpenLoopRampRate = 0.375;	// seconds from zero to full speed

    private DriveHAL()
    {
        lMotorMaster = new WPI_TalonSRX(Constants.kLeftMasterID);
        rMotorMaster = new WPI_TalonSRX(Constants.kRightMasterID);
        lMotorSlaves.add(new WPI_VictorSPX(Constants.kLeftSlaveID));
        rMotorSlaves.add(new WPI_VictorSPX(Constants.kRightSlaveID));

        lMotorMaster.configFactoryDefault();
        rMotorMaster.configFactoryDefault();

        //Config masters here
        // Get status at 100Hz (faster than default 50 Hz)
		lMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0.value, 10, kTalonTimeoutMs);
		rMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0.value, 10, kTalonTimeoutMs);

		lMotorMaster.set(ControlMode.PercentOutput, 0.0);
		rMotorMaster.set(ControlMode.PercentOutput, 0.0);
		lMotorMaster.setNeutralMode(NeutralMode.Coast);
		rMotorMaster.setNeutralMode(NeutralMode.Coast);

		// Set up the encoders
		lMotorMaster.setInverted(kLeftMotorInverted);
		rMotorMaster.setInverted(kRightMotorInverted);

        lMotorMaster.setSensorPhase(true);
        rMotorMaster.setSensorPhase(true);
		
		// Load velocity control gains
		lMotorMaster.config_kF(kVelocityControlSlot, kDriveVelocityKf, kTalonTimeoutMs);
		lMotorMaster.config_kP(kVelocityControlSlot, kDriveVelocityKp, kTalonTimeoutMs);
		lMotorMaster.config_kI(kVelocityControlSlot, kDriveVelocityKi, kTalonTimeoutMs);
		lMotorMaster.config_kD(kVelocityControlSlot, kDriveVelocityKd, kTalonTimeoutMs);
		lMotorMaster.config_IntegralZone(kVelocityControlSlot, kDriveVelocityIZone, kTalonTimeoutMs);

		rMotorMaster.config_kF(kVelocityControlSlot, kDriveVelocityKf, kTalonTimeoutMs);
		rMotorMaster.config_kP(kVelocityControlSlot, kDriveVelocityKp, kTalonTimeoutMs);
		rMotorMaster.config_kI(kVelocityControlSlot, kDriveVelocityKi, kTalonTimeoutMs);
		rMotorMaster.config_kD(kVelocityControlSlot, kDriveVelocityKd, kTalonTimeoutMs);
		rMotorMaster.config_IntegralZone(kVelocityControlSlot, kDriveVelocityIZone, kTalonTimeoutMs);
		
		lMotorMaster.configAllowableClosedloopError(kVelocityControlSlot, kDriveVelocityAllowableError, kTalonTimeoutMs);
		rMotorMaster.configAllowableClosedloopError(kVelocityControlSlot, kDriveVelocityAllowableError, kTalonTimeoutMs);

		
		// Load position control gains
		lMotorMaster.config_kF(kPositionControlSlot, kDrivePositionKf, kTalonTimeoutMs);
		lMotorMaster.config_kP(kPositionControlSlot, kDrivePositionKp, kTalonTimeoutMs);
		lMotorMaster.config_kI(kPositionControlSlot, kDrivePositionKi, kTalonTimeoutMs);
		lMotorMaster.config_kD(kPositionControlSlot, kDrivePositionKd, kTalonTimeoutMs);
		lMotorMaster.config_IntegralZone(kPositionControlSlot, kDrivePositionIZone, kTalonTimeoutMs);

		rMotorMaster.config_kF(kPositionControlSlot, kDrivePositionKf, kTalonTimeoutMs);
		rMotorMaster.config_kP(kPositionControlSlot, kDrivePositionKp, kTalonTimeoutMs);
		rMotorMaster.config_kI(kPositionControlSlot, kDrivePositionKi, kTalonTimeoutMs);
		rMotorMaster.config_kD(kPositionControlSlot, kDrivePositionKd, kTalonTimeoutMs);
		rMotorMaster.config_IntegralZone(kPositionControlSlot, kDrivePositionIZone, kTalonTimeoutMs);

		lMotorMaster.configAllowableClosedloopError(kPositionControlSlot, kDrivePositionAllowableError, kTalonTimeoutMs);
		rMotorMaster.configAllowableClosedloopError(kPositionControlSlot, kDrivePositionAllowableError, kTalonTimeoutMs);


		// Load MotionMagic control gains
		lMotorMaster.config_kF(kMotionMagicControlSlot, kDriveMotionMagicKf, kTalonTimeoutMs);
		lMotorMaster.config_kP(kMotionMagicControlSlot, kDriveMotionMagicKp, kTalonTimeoutMs);
		lMotorMaster.config_kI(kMotionMagicControlSlot, kDriveMotionMagicKi, kTalonTimeoutMs);
		lMotorMaster.config_kD(kMotionMagicControlSlot, kDriveMotionMagicKd, kTalonTimeoutMs);
		lMotorMaster.config_IntegralZone(kMotionMagicControlSlot, kDriveMotionMagicIZone, kTalonTimeoutMs);

		rMotorMaster.config_kF(kMotionMagicControlSlot, kDriveMotionMagicKf, kTalonTimeoutMs);
		rMotorMaster.config_kP(kMotionMagicControlSlot, kDriveMotionMagicKp, kTalonTimeoutMs);
		rMotorMaster.config_kI(kMotionMagicControlSlot, kDriveMotionMagicKi, kTalonTimeoutMs);
		rMotorMaster.config_kD(kMotionMagicControlSlot, kDriveMotionMagicKd, kTalonTimeoutMs);
		rMotorMaster.config_IntegralZone(kMotionMagicControlSlot, kDriveMotionMagicIZone, kTalonTimeoutMs);

		lMotorMaster.configAllowableClosedloopError(kMotionMagicControlSlot, kDriveMotionMagicAllowableError, kTalonTimeoutMs);
		rMotorMaster.configAllowableClosedloopError(kMotionMagicControlSlot, kDriveMotionMagicAllowableError, kTalonTimeoutMs);

		lMotorMaster.configMotionCruiseVelocity(inchesPerSecondToEncoderUnitsPerFrame(kPathFollowingMaxVel), kTalonTimeoutMs);
		rMotorMaster.configMotionCruiseVelocity(inchesPerSecondToEncoderUnitsPerFrame(kPathFollowingMaxVel), kTalonTimeoutMs);
		lMotorMaster.configMotionAcceleration(inchesPerSecondToEncoderUnitsPerFrame(kPathFollowingMaxAccel), kTalonTimeoutMs);
		rMotorMaster.configMotionAcceleration(inchesPerSecondToEncoderUnitsPerFrame(kPathFollowingMaxAccel), kTalonTimeoutMs);

		lMotorMaster.configOpenloopRamp(kDriveOpenLoopRampRate, 0);
		rMotorMaster.configOpenloopRamp(kDriveOpenLoopRampRate, 0);

        for (BaseMotorController lMotorSlave : lMotorSlaves) 
        {
            lMotorSlave.configFactoryDefault();
    		lMotorSlave.follow(lMotorMaster);
    		lMotorSlave.setNeutralMode(NeutralMode.Coast);
			lMotorSlave.setInverted(InvertType.FollowMaster);
        }
        for (BaseMotorController rMotorSlave : rMotorSlaves) 
        {
            rMotorSlave.configFactoryDefault();
            rMotorSlave.follow(rMotorMaster);
    		rMotorSlave.setNeutralMode(NeutralMode.Coast);
			rMotorSlave.setInverted(InvertType.FollowMaster);
        }

        // ArrayList<MotorController> lMotorControllers = new ArrayList<MotorController>(), rMotorControllers = new ArrayList<MotorController>();
        // lMotorControllers.add(lMotorMaster);
        // rMotorControllers.add(rMotorMaster);
        // for(BaseMotorController motorController : lMotorSlaves) {
        //     lMotorControllers.add(motorController);
        // }
        // for(BaseMotorController motorController : rMotorSlaves) {
        //     rMotorControllers.add(motorController);
        // }

        // MotorController[] MCType = new MotorController[lMotorControllers.size()];
        // lControllerGroup = new MotorControllerGroup(lMotorControllers.toArray(MCType));
        // rControllerGroup = new MotorControllerGroup(rMotorControllers.toArray(MCType));


        setMotors(ControlMode.PercentOutput, 0, 0);
        setNeutralMode(NeutralMode.Coast);
        setEncoders();
    }

    private final BaseMotorController lMotorMaster, rMotorMaster;
    private final ArrayList<BaseMotorController> lMotorSlaves = new ArrayList<BaseMotorController>(), rMotorSlaves = new ArrayList<BaseMotorController>();
    // private final MotorControllerGroup lControllerGroup, rControllerGroup;
    
    // Talon SRX reports position in rotations while in closed-loop Position mode
	public static double encoderUnitsToInches(double _encoderPosition)  {return _encoderPosition / kTalonFXEncoderUnitsPerRev / kDriveGearRatio * kDriveWheelCircumInches;}
	public static double inchesToEncoderUnits(double _inches)           {return _inches / kDriveWheelCircumInches * kTalonFXEncoderUnitsPerRev * kDriveGearRatio;}

	// Talon SRX reports speed in RPM while in closed-loop Speed mode
	public static double encoderUnitsPerFrameToInchesPerSecond(double _encoderEdgesPerFrame)    {return encoderUnitsToInches(_encoderEdgesPerFrame) / kFalconEncoderStatusFramePeriod;}
	public static double inchesPerSecondToEncoderUnitsPerFrame(double _inchesPerSecond)         {return inchesToEncoderUnits(_inchesPerSecond) * kFalconEncoderStatusFramePeriod;}

    public DriveHAL setMotors(ControlMode controlMode, double lMotorCtrl, double rMotorCtrl)
    {
        double lVal, rVal;
        switch(controlMode)
        {
            case PercentOutput:
                lVal = lMotorCtrl;
                rVal = rMotorCtrl;
            break;
            case Position:
            case MotionMagic:
                lVal = inchesToEncoderUnits(lMotorCtrl);
                rVal = inchesToEncoderUnits(rMotorCtrl);
            break;
            case Velocity:
                lVal = inchesPerSecondToEncoderUnitsPerFrame(lMotorCtrl);
                rVal = inchesPerSecondToEncoderUnitsPerFrame(rMotorCtrl);
            break;
            default:
            case Disabled:
                lVal = 0;
                rVal = 0;
            break;
        }
        lMotorMaster.set(controlMode, lVal);
        rMotorMaster.set(controlMode, rVal);
        return this;
    }

    public DriveHAL setNeutralMode(NeutralMode neutralMode)
    {
        lMotorMaster.setNeutralMode(neutralMode);
        lMotorMaster.setNeutralMode(neutralMode);
        for (BaseMotorController lMotorSlave : lMotorSlaves)
				lMotorSlave.setNeutralMode(neutralMode);
        for (BaseMotorController rMotorSlave : rMotorSlaves)
            rMotorSlave.setNeutralMode(neutralMode);
        return this;
    }

    //TODO: Add gyro to DriveHAL
    public DriveHAL setEncoders() {return setEncoders(0,0);}
    public DriveHAL setEncoders(double leftInches, double rightInches)
    {
        lMotorMaster.setSelectedSensorPosition(inchesToEncoderUnits(leftInches));
        rMotorMaster.setSelectedSensorPosition(inchesToEncoderUnits(rightInches));
        return this;
    }

    public double getLeftDistanceInches()   {return encoderUnitsToInches(lMotorMaster.getSelectedSensorPosition(kTalonPidIdx));}
    public double getRightDistanceInches()  {return encoderUnitsToInches(rMotorMaster.getSelectedSensorPosition(kTalonPidIdx));}

    public double getLeftSpeedInchesPerSec()    {return encoderUnitsPerFrameToInchesPerSecond(lMotorMaster.getSelectedSensorVelocity(kTalonPidIdx));}
    public double getRightSpeedInchesPerSec()   {return encoderUnitsPerFrameToInchesPerSecond(rMotorMaster.getSelectedSensorVelocity(kTalonPidIdx));}

    // public double getLeftCurrent()  {return lMotorMaster.getStatorCurrent();}
    // public double getRightCurrent() {return rMotorMaster.getStatorCurrent();}

    public double getLeftPIDError()     {return lMotorMaster.getClosedLoopError(kTalonPidIdx);}
    public double getRightPIDError()    {return rMotorMaster.getClosedLoopError(kTalonPidIdx);}

    public double getLeftMotorStatus()     {return getMotorStatus(lMotorMaster);}
    public double getRightMotorStatus()    {return getMotorStatus(rMotorMaster);}
    private double getMotorStatus(BaseMotorController motor)
    {
        switch(motor.getControlMode())
        {
            case PercentOutput:
            case Disabled:
            default:
                return motor.getMotorOutputPercent();
            case Position:
            case MotionMagic:
                return motor.getSelectedSensorPosition(kTalonPidIdx);
            case Velocity:
                return motor.getSelectedSensorVelocity(kTalonPidIdx);
        }
    }

    //TODO: Add gyro
    public double getHeadingRad()   {return -686;}
    public double getHeadingDeg()   {return -686;}

    // public MotorControllerGroup[] getMotorControllerGroups() {return new MotorControllerGroup[]{lControllerGroup, rControllerGroup};}
}
