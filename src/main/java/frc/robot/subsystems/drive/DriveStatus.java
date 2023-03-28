package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.RobotConfiguration;
import frc.robot.subsystems.framework.StatusBase;

public class DriveStatus extends StatusBase {
    private static DriveStatus instance;
    public static DriveStatus getInstance(){if(instance == null){instance = new DriveStatus();}return instance;}

    private final DriveHAL HAL = DriveHAL.getInstance();

    private DriveCommand    command = DriveCommand.COAST();
    public DriveCommand     getCommand()                    {return command;}
    public DriveStatus      setCommand(DriveCommand command) {this.command = command; return this;}

    private ControlMode talonControlMode = ControlMode.Disabled;
    public ControlMode  getTalonMode()                      {return talonControlMode;}
    public DriveStatus  setTalonMode(ControlMode talonMode) {this.talonControlMode = talonMode; return this;}
    
	private NeutralMode neutralMode = NeutralMode.Coast;
    public NeutralMode  getNeutralMode()                        {return neutralMode;}
    public DriveStatus  setNeutralMode(NeutralMode neutralMode) {this.neutralMode = neutralMode; return this;}
	
	private double      lDistanceInches, rDistanceInches;
    public double       getLeftDistanceInches()                         {return lDistanceInches;}
    public double       getRightDistanceInches()                        {return rDistanceInches;}
    public DriveStatus  setLeftDistanceInches(double lDistanceInches)   {this.lDistanceInches = lDistanceInches; return this;}
    public DriveStatus  setRightDistanceInches(double rDistanceInches)  {this.rDistanceInches = rDistanceInches; return this;}
    public DriveStatus  setDistanceInches(double lDistanceInches, double rDistanceInches) {setLeftDistanceInches(lDistanceInches); setRightDistanceInches(rDistanceInches); return this;}
    
    private WheelSpeeds wheelSpeeds = new WheelSpeeds();
    public WheelSpeeds  getWheelSpeeds()                                        {return wheelSpeeds;}
    public double       getLeftSpeedInchesPerSec()                              {return wheelSpeeds.left;}
    public double       getRightSpeedInchesPerSec()                             {return wheelSpeeds.right;}
    public DriveStatus  setWheelSpeeds(WheelSpeeds wheelSpeeds)                 {this.wheelSpeeds = wheelSpeeds; return this;}
    public DriveStatus  setLeftSpeedInchesPerSec(double lSpeedInchesPerSec)     {this.wheelSpeeds.left = lSpeedInchesPerSec; return this;}
    public DriveStatus  setRightSpeedInchesPerSec(double rSpeedInchesPerSec)    {this.wheelSpeeds.right = rSpeedInchesPerSec; return this;}
    
	private Rotation2d  rotation = new Rotation2d();
    public Rotation2d   getRotation()                       {return rotation;}
    public double       getHeadingRad()                     {return rotation.getRadians();}
    public double       getHeadingDeg()                     {return rotation.getDegrees();}
    public DriveStatus  setRotation(Rotation2d rotation)    {this.rotation = rotation; return this;}
    public DriveStatus  setHeadingRad(double headingRad)    {this.rotation = Rotation2d.fromRadians(headingRad); return this;}
    public DriveStatus  setHeadingDeg(double headingDeg)    {this.rotation = Rotation2d.fromDegrees(headingDeg); return this;}

    private double      pitchDeg;
    public double       getPitchDeg()                   {return pitchDeg;}
    public double       getPitchRad()                   {return Units.degreesToRadians(getPitchDeg());}
    public DriveStatus  setPitchDeg(double pitchDeg)    {this.pitchDeg = pitchDeg; return this;}
    public DriveStatus  setPitchRad(double pitchRad)    {setPitchDeg(Units.radiansToDegrees(pitchRad)); return this;}
	
	private double      lMotorCurrent, rMotorCurrent;
    public double       getLeftMotorCurrent()                       {return lMotorCurrent;}
    public double       getRightMotorCurrent()                      {return rMotorCurrent;}
    public DriveStatus  setLeftMotorCurrent(double lMotorCurrent)   {this.lMotorCurrent = lMotorCurrent; return this;}
    public DriveStatus  setRightMotorCurrent(double rMotorCurrent)  {this.rMotorCurrent = rMotorCurrent; return this;}
    public DriveStatus  setMotorCurrent(double lMotorCurrent, double rMotorCurrent) {setLeftMotorCurrent(lMotorCurrent); setRightMotorCurrent(rMotorCurrent); return this;}
	
    private double      lMotorStatus, rMotorStatus;
    public double       getLeftMotorStatus()                        {return lMotorStatus;}
    public double       getRightMotorStatus()                       {return rMotorStatus;}
    public DriveStatus  setLeftMotorStatus(double lMotorStatus)     {this.lMotorStatus = lMotorStatus; return this;}
    public DriveStatus  setRightMotorStatus(double rMotorStatus)    {this.rMotorStatus = rMotorStatus; return this;}
    public DriveStatus  setMotorStatus(double lMotorStatus, double rSpeedInchesPerSec) {setLeftMotorStatus(lMotorStatus); setRightMotorStatus(rMotorStatus); return this;}
	
    private double      lMotorPIDError, rMotorPIDError;
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
    
    @Override protected void loadConfiguration(RobotConfiguration configuration) {}

    @Override
    protected void updateInputs() {
        setDistanceInches(HAL.getLeftDistanceInches(), HAL.getRightDistanceInches());
        setWheelSpeeds(new WheelSpeeds(HAL.getLeftSpeedInchesPerSec(), HAL.getRightSpeedInchesPerSec()));
        setRotation(HAL.getRotation());
        setMotorCurrent(HAL.getLeftCurrent(), HAL.getRightCurrent());
        setMotorPIDError(HAL.getLeftPIDError(), HAL.getRightPIDError());
        setHeadingDeg(HAL.getHeadingDeg());
        setPitchDeg(HAL.getPitchDeg());
    }

    @Override
    protected void exportToTable(LogTable table) {
        table.put("Encoded Distance (In)/Left",   getLeftDistanceInches());
        table.put("Encoded Distance (In)/Right",  getRightDistanceInches());
        table.put("Encoded Speed (In|Sec)/Left",  getLeftSpeedInchesPerSec());
        table.put("Encoded Speed (In|Sec)/Right", getRightSpeedInchesPerSec());
        table.put("Gyro/Heading (Rad)",           getHeadingRad());
        table.put("Motor Current (Amps)/Left",    getLeftMotorCurrent());
        table.put("Motor Current (Amps)/Right",   getRightMotorCurrent());
        table.put("Motor PID Error/Left",         getLeftMotorPIDError());
        table.put("Motor PID Error/Right",        getRightMotorPIDError());
    }

    @Override
    protected void importFromTable(LogTable table) {
        setLeftDistanceInches       (table.getDouble("Encoded Distance (In)/Left",    getLeftDistanceInches()));
        setRightDistanceInches      (table.getDouble("Encoded Distance (In)/Right",   getRightDistanceInches()));
        setLeftSpeedInchesPerSec    (table.getDouble("Encoded Speed (In|Sec)/Left",   getLeftSpeedInchesPerSec()));
        setRightSpeedInchesPerSec   (table.getDouble("Encoded Speed (In|Sec)/Right",  getRightSpeedInchesPerSec()));
        setHeadingRad               (table.getDouble("Gyro/Heading (Rad)",            getHeadingRad()));
        setLeftMotorCurrent         (table.getDouble("Motor Current (Amps)/Left",     getLeftMotorCurrent()));
        setRightMotorCurrent        (table.getDouble("Motor Current (Amps)/Right",    getRightMotorCurrent()));
        setLeftMotorPIDError        (table.getDouble("Motor PID Error/Left",          getLeftMotorPIDError()));
        setRightMotorPIDError       (table.getDouble("Motor PID Error/Right",         getRightMotorPIDError()));
    }

    @Override protected void processTable() {}

    @Override
    protected void processOutputs(Logger logger, String prefix) {
        // TODO: LESS LOGGING
        // logger.recordOutput(prefix + "Control Mode",                    getTalonMode().name());
        // logger.recordOutput(prefix + "Neutral Mode",                    getNeutralMode().name());
        logger.recordOutput(prefix + "Encoded Distance (In)/Left",      getLeftDistanceInches());
        logger.recordOutput(prefix + "Encoded Distance (In)/Right",     getRightDistanceInches());
        logger.recordOutput(prefix + "Encoded Speed (In|Sec)/Left",     getLeftSpeedInchesPerSec());
        logger.recordOutput(prefix + "Encoded Speed (In|Sec)/Right",    getRightSpeedInchesPerSec());
        // logger.recordOutput(prefix + "Gyro/Heading (Deg)",              getHeadingDeg());
        logger.recordOutput(prefix + "Gyro/Pitch (Deg)",                getPitchDeg());
        // logger.recordOutput(prefix + "Motor Current (Amps)/Left",       getLeftMotorCurrent());
        // logger.recordOutput(prefix + "Motor Current (Amps)/Right",      getRightMotorCurrent());
        // logger.recordOutput(prefix + "Motor Status/Left",               getLeftMotorStatus());
        // logger.recordOutput(prefix + "Motor Status/Right",              getRightMotorStatus());
        // logger.recordOutput(prefix + "Motor PID Error/Left",            getLeftMotorPIDError());
        // logger.recordOutput(prefix + "Motor PID Error/Right",           getRightMotorPIDError());

        // getCommand().logCommand(logger, prefix + "Command/");
    }
    
}
