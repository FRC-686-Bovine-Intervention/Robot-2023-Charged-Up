package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.arm.ArmHAL.LimelightPipeline;
import frc.robot.subsystems.framework.StatusBase;

public class ArmStatus extends StatusBase {
    private static ArmStatus instance;
    public static ArmStatus getInstance(){if(instance == null) instance = new ArmStatus(); return instance;}

    public final ArmHAL HAL = ArmHAL.getInstance();

    private ArmStatus() {Subsystem = Arm.getInstance();}

    public enum ArmState {
        Defense,        // Arm is idling, retracted fully and waiting for the intake to do something
        IdentifyCone,   // Arm is checking if the piece in the intake is a cone
        IdentifyCube,   // Arm is checking if the piece in the intake is a cube
        Grab,           // Arm has identified the location of the piece and arm is grabbing the piece from the intake
        Hold,           // Arm has grabbed the piece and is holding it in the defense position
        Align,          // Robot has entered the community and the turret should align to the alliance wall
        Extend,         // Driver has selected a node to extend to and arm is extending to it
        Adjust,         // Driver and limelight are fudging the position of the turret to align piece on the node
        Release         // Driver has decided the piece will score on the node and tells the arm to release the piece
    }

    private ArmState armState = ArmState.IdentifyCube;
    public ArmState getArmState()                   {return armState;}
    public ArmStatus setArmState(ArmState armState) {this.armState = armState; return this;}

    private double targetTurretAngle; //TODO: Units
    public ArmStatus setTargetTurretAngle(double angle){
        targetTurretAngle = angle;
        return this;
    }
    public double getTargetTurretAngle(){
        return targetTurretAngle;
    }

    private LimelightPipeline targetPipeline;
    private LimelightPipeline currentPipeline;
    public LimelightPipeline getPipeline() {
        return currentPipeline;
    }
    public ArmStatus setPipeline(LimelightPipeline pipeline) {
        this.targetPipeline = pipeline;
        return this;
    }

    private boolean targetInView;
    public ArmStatus setTargetInView(boolean targetInView) {
        this.targetInView = targetInView;
        return this;
    }
    public boolean getTargetInView() {
        return targetInView;
    }

    private double turretPower;
    public double getTurretPower() {
        return turretPower;
    }
    public ArmStatus setTurretPower(double turretPower) {
        this.turretPower = turretPower;
        return this;
    }

    private double turretPosition; 
    public ArmStatus setTurretPosition(double turretPosition) {
        this.turretPosition = turretPosition;
        return this;
    }
    public double getTurretPosition() {
        return turretPosition;
    }

    private double targetXOffset = 0.0;
    private ArmStatus setTargetXOffset(double offset) {
        targetXOffset = offset;
        return this;
    }
    public double getTargetXOffset() {
        return targetXOffset;
    }

    @Override
    protected void exportToTable(LogTable table) {
        table.put("Turret Position", getTurretPosition());
        table.put("Target X Offset", getTargetXOffset());
        table.put("Target In View?", getTargetInView());
        table.put("Current Pipeline", currentPipeline != null ? currentPipeline.name() : "null");
    }

    @Override
    protected void importFromTable(LogTable table) {
        setTurretPosition(table.getDouble("Turret Position", turretPosition));
        setTargetXOffset(table.getDouble("Target X Offset", targetXOffset));
        setTargetInView(table.getBoolean("Target In View?", targetInView));
        currentPipeline = LimelightPipeline.getFromName(table.getString("Current Pipeline", currentPipeline != null ? currentPipeline.name() : "null"));
    }

    @Override
    protected void updateInputs() {
        setTurretPosition(HAL.getTurretRelative());
        setTargetXOffset(HAL.getTargetXOffset());
        setTargetInView(HAL.getTargetInView());
        currentPipeline = HAL.getPipeline();
    }

    @Override
    protected void recordOutputs(Logger logger, String prefix) {
        HAL.setTurretPower(turretPower);
        HAL.setPipeline(targetPipeline);

        logger.recordOutput(prefix + "Current Arm State", armState != null ? armState.name() : "null");

        logger.recordOutput(prefix + "Limelight/Target X Offset", getTargetXOffset());
        logger.recordOutput(prefix + "Limelight/Current Pipeline", currentPipeline != null ? currentPipeline.name() : "null");
        logger.recordOutput(prefix + "Limelight/Target Pipeline", targetPipeline != null ? targetPipeline.name() : "null");
        logger.recordOutput(prefix + "Limelight/Valid Target", getTargetInView());

        logger.recordOutput(prefix + "Turret/Power", getTurretPower());
        logger.recordOutput(prefix + "Turret/Relative Position", getTurretPosition());
        logger.recordOutput(prefix + "Turret/Absolute Position", HAL.getTurretAbsolute());
        logger.recordOutput(prefix + "Turret/Target Angle", getTargetTurretAngle());
    }
}
