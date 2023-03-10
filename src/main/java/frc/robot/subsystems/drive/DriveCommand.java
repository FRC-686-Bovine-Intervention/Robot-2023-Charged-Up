package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;

public class DriveCommand {
    public enum DriveControlMode
    {
        OPEN_LOOP,
        BASE_LOCKED,
        POSITION_SETPOINT,
        VELOCITY_SETPOINT,
        VELOCITY_HEADING,
        TURN_TO_HEADING
    }

	private DriveControlMode driveMode = DriveControlMode.OPEN_LOOP;
    public DriveControlMode getDriveMode()  {return driveMode;}
    public DriveCommand setDriveMode(DriveControlMode driveMode) {
        this.driveMode = driveMode;
        switch (driveMode)
    	{
            case OPEN_LOOP:
                setTalonMode(ControlMode.PercentOutput);
            break;
            case BASE_LOCKED:
                setTalonMode(ControlMode.Position);
            break;
            case POSITION_SETPOINT:
            case TURN_TO_HEADING:
                setTalonMode(ControlMode.MotionMagic);
            break;
            case VELOCITY_SETPOINT:
            case VELOCITY_HEADING:
                setTalonMode(ControlMode.Velocity);
            break;
            default:
                setTalonMode(ControlMode.Disabled);
            break;
    	}
        return this;
    }
    
	private ControlMode talonMode = ControlMode.PercentOutput;
    public ControlMode getTalonMode()       {return talonMode;}
    public DriveCommand setTalonMode(ControlMode talonMode) {this.talonMode = talonMode; return this;}
    
	private WheelSpeeds wheelSpeed = new WheelSpeeds();
    public WheelSpeeds getWheelSpeed()      {return wheelSpeed;}
    public double getLeftMotor()            {return wheelSpeed.left;}
    public double getRightMotor()           {return wheelSpeed.right;}
    public double getSpeed()                {return (getLeftMotor() + getRightMotor()) / 2.0;}
    public DriveCommand setWheelSpeed(WheelSpeeds wheelSpeed) {this.wheelSpeed = wheelSpeed; return this;}
    
	private NeutralMode neutralMode = NeutralMode.Coast;
    public NeutralMode getNeutralMode()     {return neutralMode;}
    public DriveCommand setNeutralMode(NeutralMode neutralMode) {this.neutralMode = neutralMode; return this;}
    
	private boolean resetEncoders = false;
    public boolean getResetEncoders()       {return resetEncoders;}
    public DriveCommand setResetEncoders(boolean resetEncoders) {this.resetEncoders = resetEncoders; return this;}
    
    private double commandTime;
    public double getCommandTime()          {return commandTime;}
    public DriveCommand setCommandTime()    {return setCommandTime(Timer.getFPGATimestamp());}
    public DriveCommand setCommandTime(double commandTime) {this.commandTime = commandTime; return this;}

    public DriveCommand(double left, double right)
        {this(new WheelSpeeds(left, right));}
    public DriveCommand(WheelSpeeds wheelSpeed)
        {this(DriveControlMode.OPEN_LOOP, wheelSpeed, NeutralMode.Coast);}
    public DriveCommand(DriveControlMode driveMode, double left, double right)
        {this(driveMode, new WheelSpeeds(left, right));}
    public DriveCommand(DriveControlMode driveMode, WheelSpeeds wheelSpeed)
        {this(driveMode, wheelSpeed, NeutralMode.Coast);}
    public DriveCommand(DriveControlMode driveMode, double left, double right, NeutralMode neutralMode)
        {this(driveMode, new WheelSpeeds(left, right), neutralMode);}
    public DriveCommand(DriveControlMode driveMode, WheelSpeeds wheelSpeed, NeutralMode neutralMode)
    {
        setDriveMode(driveMode);
        setWheelSpeed(wheelSpeed);
        setNeutralMode(neutralMode);
        setCommandTime();
    }

    public static DriveCommand COAST() {return new DriveCommand(DriveControlMode.OPEN_LOOP, 0, 0, NeutralMode.Coast);}
    public static DriveCommand BRAKE() {return new DriveCommand(DriveControlMode.OPEN_LOOP, 0, 0, NeutralMode.Brake);}

    public DriveCommand logCommand(Logger logger, String prefix)
    {
        logger.recordOutput(prefix + "Drive Mode",              getDriveMode().name());
        logger.recordOutput(prefix + "Talon Control Mode",      getTalonMode().name());
        logger.recordOutput(prefix + "Talon Neutral Mode",      getNeutralMode().name());
        logger.recordOutput(prefix + "Drive Setpoint/Left",     getWheelSpeed().left);
        logger.recordOutput(prefix + "Drive Setpoint/Right",    getWheelSpeed().right);
        return this;
    }
}
