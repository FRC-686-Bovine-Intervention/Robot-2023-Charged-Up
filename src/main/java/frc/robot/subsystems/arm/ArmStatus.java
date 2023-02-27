package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import frc.robot.lib.sensorCalibration.PotAndEncoder;
import frc.robot.subsystems.framework.StatusBase;

public class ArmStatus extends StatusBase {
    private static ArmStatus instance;
    public static ArmStatus getInstance(){if(instance == null) instance = new ArmStatus(); return instance;}

    private final Arm arm = Arm.getInstance();
    private final ArmHAL HAL = ArmHAL.getInstance();

    public enum ArmState {
        ZeroDistal,         // =============================================================================
        ZeroTurret,         // If arm isn't where it's supposed to be on enable, move arm to proper position
        ZeroProximal,       // =============================================================================
        Defense,            // Arm is idling, retracted fully and waiting for the intake to do something
        IdentifyPiece,      // Arm is identifying the piece in the intake
        Grab,               // Arm has identified the location of the piece and arm is grabbing the piece from the intake
        Hold,               // Arm has grabbed the piece and is holding it in the defense position
        Align,              // Robot has entered the community and the turret should align to the alliance wall
        Extend,             // Driver has selected a node to extend to and arm is extending to it
        Adjust,             // Driver and limelight are fudging the position of the turret to align piece on the node
        Release,            // Driver has decided the piece will score on the node and tells the arm to release the piece
        SubstationExtend,   // Driver has decided to grab a piece from the substation
        SubstationGrab      // Driver is ready to grab piece from the substation
    }

    private ArmStatus() {Subsystem = arm;}

    // Generic
    private ArmCommand      command = new ArmCommand();
    protected ArmCommand    getCommand()                    {return command;}
    private ArmStatus       setCommand(ArmCommand command)  {this.command = command; return this;}

    private ArmState    armState = ArmState.Defense;
    public ArmState     getArmState()                   {return armState;}
    protected ArmStatus setArmState(ArmState armState)  {this.armState = armState; return this;}

    // Turret
    private double      turretPosition; 
    public double       getTurretPosition() {return turretPosition;}
    private ArmStatus   setTurretPosition(double turretPosition) {this.turretPosition = turretPosition; return this;}

    private double      targetTurretAngle;
    public double       getTargetTurretAngle()              {return targetTurretAngle;}
    protected ArmStatus setTargetTurretAngle(double angle)  {targetTurretAngle = angle; return this;}

    private double      turretPower;
    public double       getTurretPower()                    {return turretPower;}
    protected ArmStatus setTurretPower(double turretPower)  {this.turretPower = turretPower; return this;}

    // Arm
    private ArmPose.Preset  targetArmPose = ArmPose.Preset.DEFENSE;
    public ArmPose.Preset   getTargetArmPose()                              {return targetArmPose;}
    protected ArmStatus     setTargetArmPose(ArmPose.Preset targetArmPose)  {this.targetArmPose = targetArmPose; return this;}

    private ArmPose.Preset  currentArmPose = null;
    public ArmPose.Preset   getCurrentArmPose()                                 {return currentArmPose;}
    protected ArmStatus     setCurrentArmPose(ArmPose.Preset currentArmPose)    {this.currentArmPose = currentArmPose; return this;}

    private ArmTrajectory   currentArmTrajectory = null;
    public ArmTrajectory    getCurrentArmTrajectory()                                   {return currentArmTrajectory;}
    protected ArmStatus     setCurrentArmTrajectory(ArmTrajectory currentArmTrajectory) {this.currentArmTrajectory = currentArmTrajectory; return this;}

    // Shoulder
    private PotAndEncoder.Reading   shoulderReading = new PotAndEncoder.Reading(0,0,0);
    private ArmStatus               setShoulderReading(PotAndEncoder.Reading shoulderReading)   {this.shoulderReading = shoulderReading; return this;}
    
    private PotAndEncoder.Status    shoulderStatus;
    public PotAndEncoder.Status     getShoulderStatus()                                     {return shoulderStatus;}
    private ArmStatus               setShoulderStatus(PotAndEncoder.Status shoulderStatus)  {this.shoulderStatus = shoulderStatus; return this;}

    private double      shoulderPower;
    public double       getShoulderPower()                      {return shoulderPower;}
    protected ArmStatus setShoulderPower(double shoulderPower)  {this.shoulderPower = shoulderPower; return this;}
    
    // Elbow
    private PotAndEncoder.Reading   elbowReading = new PotAndEncoder.Reading(0,0,0);
    private ArmStatus               setElbowReading(PotAndEncoder.Reading elbowReading) {this.elbowReading = elbowReading; return this;}
    
    private PotAndEncoder.Status    elbowStatus;
    public PotAndEncoder.Status     getElbowStatus()                                    {return elbowStatus;}
    private ArmStatus               setElbowStatus(PotAndEncoder.Status elbowStatus)    {this.elbowStatus = elbowStatus; return this;}

    private double      elbowPower;
    public double       getElbowPower()                     {return elbowPower;}
    protected ArmStatus setElbowPower(double elbowPower)    {this.elbowPower = elbowPower; return this;}

    // Claw
    private boolean     clawGrabbing;
    public boolean      getClawGrabbing()                       {return clawGrabbing;}
    protected ArmStatus setClawGrabbing(boolean clawGrabbing)   {this.clawGrabbing = clawGrabbing; return this;}

    @Override
    public void updateInputs() {
        setCommand(arm.getCommand());
        setTurretPosition(HAL.getTurretRelative());
        setShoulderReading(HAL.getShoulderPotEncoder().getReading());
        setElbowReading(HAL.getElbowPotEncoder().getReading());
    }

    @Override
    public void exportToTable(LogTable table) {
        table.put("Turret Position", getTurretPosition());
        shoulderReading.exportToTable(table, "Shoulder Reading");
        elbowReading.exportToTable(table, "Elbow Reading");
    }
    
    @Override
    public void importFromTable(LogTable table) {
        setTurretPosition(table.getDouble("Turret Position", turretPosition));
        setShoulderReading(shoulderReading.importFromTable(table, "Shoulder Reading"));
        setElbowReading(elbowReading.importFromTable(table, "Elbow Reading"));
    }

    @Override
    public void processTable() {
        setShoulderStatus(HAL.getShoulderPotEncoder().update(shoulderReading));
        setElbowStatus(HAL.getElbowPotEncoder().update(elbowReading));
    }

    @Override
    public void processOutputs(Logger logger, String prefix) {
        HAL.setTurretPower(turretPower);

        // Generic
        command.recordOutputs(logger, prefix + "Command");
        logger.recordOutput(prefix + "Current Arm State", armState != null ? armState.name() : "null");

        // Turret
        logger.recordOutput(prefix + "Turret/Power",        getTurretPower());
        logger.recordOutput(prefix + "Turret/Position",     getTurretPosition());
        logger.recordOutput(prefix + "Turret/Target Angle", getTargetTurretAngle());

        // Arm
        logger.recordOutput(prefix + "Arm/Target Pose",     targetArmPose != null ? targetArmPose.name() : "null");
        logger.recordOutput(prefix + "Arm/Current Pose",    currentArmPose != null ? currentArmPose.name() : "null");
        // logger.recordOutput(prefix + "Arm/Current Trajectory", currentArmTrajectory != null ? currentArmTrajectory.name() : "null");

        // Shoulder
        shoulderStatus.recordOutputs(logger, prefix + "Arm/Shoulder Status");
        logger.recordOutput(prefix + "Arm/Shoulder/Power", shoulderPower);
        
        // Elbow
        elbowStatus.recordOutputs(logger, prefix + "Arm/Elbow Status");
        logger.recordOutput(prefix + "Arm/Elbow/Power", elbowPower);
    }
}