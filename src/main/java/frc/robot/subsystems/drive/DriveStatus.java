package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.framework.StatusBase;

public class DriveStatus extends StatusBase {
    private static DriveStatus instance;
    public static DriveStatus getInstance(){if(instance == null){instance = new DriveStatus();}return instance;}

    private final DriveHAL HAL = DriveHAL.getInstance();

    private DriveCommand command = DriveCommand.COAST();
    public DriveCommand getCommand()                    {return command;}
    public DriveStatus  setCommand(DriveCommand command) {this.command = command; return this;}

    private ControlMode talonControlMode = ControlMode.Disabled;
    public ControlMode  getTalonMode()                      {return talonControlMode;}
    public DriveStatus  setTalonMode(ControlMode talonMode) {this.talonControlMode = talonMode; return this;}
    
	private NeutralMode neutralMode = NeutralMode.Coast;
    public NeutralMode  getNeutralMode()                        {return neutralMode;}
    public DriveStatus  setNeutralMode(NeutralMode neutralMode) {this.neutralMode = neutralMode; return this;}
	
	private double lDistanceInches, rDistanceInches;
    public double       getLeftDistanceInches()                         {return lDistanceInches;}
    public double       getRightDistanceInches()                        {return rDistanceInches;}
    public DriveStatus  setLeftDistanceInches(double lDistanceInches)   {this.lDistanceInches = lDistanceInches; return this;}
    public DriveStatus  setRightDistanceInches(double rDistanceInches)  {this.rDistanceInches = rDistanceInches; return this;}
    public DriveStatus  setDistanceInches(double lDistanceInches, double rDistanceInches) {setLeftDistanceInches(lDistanceInches); setRightDistanceInches(rDistanceInches); return this;}
    
	private double lSpeedInchesPerSec, rSpeedInchesPerSec;
    public double       getLeftSpeedInchesPerSec()                              {return lSpeedInchesPerSec;}
    public double       getRightSpeedInchesPerSec()                             {return rSpeedInchesPerSec;}
    public DriveStatus  setLeftSpeedInchesPerSec(double lSpeedInchesPerSec)     {this.lSpeedInchesPerSec = lSpeedInchesPerSec; return this;}
    public DriveStatus  setRightSpeedInchesPerSec(double rSpeedInchesPerSec)    {this.rSpeedInchesPerSec = rSpeedInchesPerSec; return this;}
    public DriveStatus  setSpeedInchesPerSec(double lSpeedInchesPerSec, double rSpeedInchesPerSec) {setLeftSpeedInchesPerSec(lSpeedInchesPerSec); setRightSpeedInchesPerSec(rSpeedInchesPerSec); return this;}
    
	private double headingRad;
    public double       getHeadingRad()                     {return headingRad;}
    public double       getHeadingDeg()                     {return headingRad/Math.PI*180.0;}
    public DriveStatus  setHeadingRad(double headingRad)    {this.headingRad = headingRad; return this;}
    public DriveStatus  setHeadingDeg(double headingDeg)    {this.headingRad = headingDeg*Math.PI/180.0; return this;}
	
	private double lMotorCurrent, rMotorCurrent;
    public double       getLeftMotorCurrent()                       {return lMotorCurrent;}
    public double       getRightMotorCurrent()                      {return rMotorCurrent;}
    public DriveStatus  setLeftMotorCurrent(double lMotorCurrent)   {this.lMotorCurrent = lMotorCurrent; return this;}
    public DriveStatus  setRightMotorCurrent(double rMotorCurrent)  {this.rMotorCurrent = rMotorCurrent; return this;}
    public DriveStatus  setMotorCurrent(double lMotorCurrent, double rMotorCurrent) {setLeftMotorCurrent(lMotorCurrent); setRightMotorCurrent(rMotorCurrent); return this;}
	
    private double lMotorStatus, rMotorStatus;
    public double       getLeftMotorStatus()                        {return lMotorStatus;}
    public double       getRightMotorStatus()                       {return rMotorStatus;}
    public DriveStatus  setLeftMotorStatus(double lMotorStatus)     {this.lMotorStatus = lMotorStatus; return this;}
    public DriveStatus  setRightMotorStatus(double rMotorStatus)    {this.rMotorStatus = rMotorStatus; return this;}
    public DriveStatus  setMotorStatus(double lMotorStatus, double rSpeedInchesPerSec) {setLeftMotorStatus(lMotorStatus); setRightMotorStatus(rMotorStatus); return this;}
	
    private double lMotorPIDError, rMotorPIDError;
    public double       getLeftMotorPIDError()                          {return lMotorPIDError;}
    public double       getRightMotorPIDError()                         {return rMotorPIDError;}
    public DriveStatus  setLeftMotorPIDError(double lMotorPIDError)     {this.lMotorPIDError = lMotorPIDError; return this;}
    public DriveStatus  setRightMotorPIDError(double rMotorPIDError)    {this.rMotorPIDError = rMotorPIDError; return this;}
    public DriveStatus  setMotorPIDError(double lMotorPIDError, double rMotorPIDError) {setLeftMotorPIDError(lMotorPIDError); setRightMotorPIDError(rMotorPIDError); return this;}

    private DriveStatus()
    {
        Subsystem = Drive.getInstance();
        EnabledEntry = Shuffleboard.getTab("Drivetrain").add("Enabled", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    }

    @Override
    public void exportToTable(LogTable table, String prefix) {
        table.put(prefix + "/Encoded Distance (In)/Left",   getLeftDistanceInches());
        table.put(prefix + "/Encoded Distance (In)/Right",  getRightDistanceInches());
        table.put(prefix + "/Encoded Speed (In|Sec)/Left",  getLeftSpeedInchesPerSec());
        table.put(prefix + "/Encoded Speed (In|Sec)/Right", getRightSpeedInchesPerSec());
        table.put(prefix + "/Gyro Heading (Rad)",           getHeadingRad());
        table.put(prefix + "/Motor Current (Amps)/Left",    getLeftMotorCurrent());
        table.put(prefix + "/Motor Current (Amps)/Right",   getRightMotorCurrent());
        table.put(prefix + "/Motor PID Error/Left",         getLeftMotorPIDError());
        table.put(prefix + "/Motor PID Error/Right",        getRightMotorPIDError());
    }
    @Override
    public void importFromTable(LogTable table, String prefix) {
        setLeftDistanceInches       (table.getDouble(prefix + "/Encoded Distance (In)/Left",    getLeftDistanceInches()));
        setRightDistanceInches      (table.getDouble(prefix + "/Encoded Distance (In)/Right",   getRightDistanceInches()));
        setLeftSpeedInchesPerSec    (table.getDouble(prefix + "/Encoded Speed (In|Sec)/Left",   getLeftSpeedInchesPerSec()));
        setRightSpeedInchesPerSec   (table.getDouble(prefix + "/Encoded Speed (In|Sec)/Right",  getRightSpeedInchesPerSec()));
        setHeadingRad               (table.getDouble(prefix + "/Gyro Heading (Rad)",            getHeadingRad()));
        setLeftMotorCurrent         (table.getDouble(prefix + "/Motor Current (Amps)/Left",     getLeftMotorCurrent()));
        setRightMotorCurrent        (table.getDouble(prefix + "/Motor Current (Amps)/Right",    getRightMotorCurrent()));
        setLeftMotorPIDError        (table.getDouble(prefix + "/Motor PID Error/Left",          getLeftMotorPIDError()));
        setRightMotorPIDError       (table.getDouble(prefix + "/Motor PID Error/Right",         getRightMotorPIDError()));
    }
    @Override
    public void updateInputs() {
        setDistanceInches(HAL.getLeftDistanceInches(), HAL.getRightDistanceInches());
        setSpeedInchesPerSec(HAL.getLeftSpeedInchesPerSec(), HAL.getRightSpeedInchesPerSec());
        setHeadingRad(HAL.getHeadingRad());
        // setMotorCurrent(HAL.getLeftCurrent(), HAL.getRightCurrent());
        setMotorPIDError(HAL.getLeftPIDError(), HAL.getRightPIDError());
    }
    @Override
    public void recordOutputs(String prefix) {
        Logger.getInstance().recordOutput(prefix + "/Control Mode",                 getTalonMode().name());
        Logger.getInstance().recordOutput(prefix + "/Neutral Mode",                 getNeutralMode().name());
        Logger.getInstance().recordOutput(prefix + "/Encoded Distance (In)/Left",   getLeftDistanceInches());
        Logger.getInstance().recordOutput(prefix + "/Encoded Distance (In)/Right",  getRightDistanceInches());
        Logger.getInstance().recordOutput(prefix + "/Encoded Speed (In|Sec)/Left",  getLeftSpeedInchesPerSec());
        Logger.getInstance().recordOutput(prefix + "/Encoded Speed (In|Sec)/Right", getRightSpeedInchesPerSec());
        Logger.getInstance().recordOutput(prefix + "/Gyro Heading/Radians",         getHeadingRad());
        Logger.getInstance().recordOutput(prefix + "/Gyro Heading/Degrees",         getHeadingDeg());
        Logger.getInstance().recordOutput(prefix + "/Motor Current (Amps)/Left",    getLeftMotorCurrent());
        Logger.getInstance().recordOutput(prefix + "/Motor Current (Amps)/Right",   getRightMotorCurrent());
        Logger.getInstance().recordOutput(prefix + "/Motor Status/Left",            getLeftMotorStatus());
        Logger.getInstance().recordOutput(prefix + "/Motor Status/Right",           getRightMotorStatus());
        Logger.getInstance().recordOutput(prefix + "/Motor PID Error/Left",         getLeftMotorPIDError());
        Logger.getInstance().recordOutput(prefix + "/Motor PID Error/Right",        getRightMotorPIDError());

        Logger.getInstance().recordOutput(prefix + "/Command/Drive Mode",           getCommand().getDriveMode().name());
        Logger.getInstance().recordOutput(prefix + "/Command/Talon Control Mode",   getCommand().getTalonMode().name());
        Logger.getInstance().recordOutput(prefix + "/Command/Drive Setpoint/Left",  getCommand().getWheelSpeed().left);
        Logger.getInstance().recordOutput(prefix + "/Command/Drive Setpoint/Right", getCommand().getWheelSpeed().right);
    }
    
}
