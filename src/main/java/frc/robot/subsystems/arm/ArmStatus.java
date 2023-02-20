package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import frc.robot.lib.sensorCalibration.PotAndEncoder;
import frc.robot.subsystems.framework.StatusBase;

public class ArmStatus extends StatusBase {
    private static ArmStatus instance;
    public static ArmStatus getInstance(){if(instance == null) instance = new ArmStatus(); return instance;}

    private final ArmHAL hal = ArmHAL.getInstance();

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

    private ArmState armState = ArmState.Defense;
    public ArmState getArmState()                   {return armState;}
    public ArmStatus setArmState(ArmState armState) {this.armState = armState; return this;}

    private PotAndEncoder.Status shoulderStatus;
    public PotAndEncoder.Status getShoulderState() {return shoulderStatus;}
    public ArmStatus setShoulderState(PotAndEncoder.Status state) {this.shoulderStatus = state; return this;}

    private PotAndEncoder.Status elbowStatus;
    public PotAndEncoder.Status getElbowState(){return elbowStatus;}
    public ArmStatus setElbowState(PotAndEncoder.Status state) {this.elbowStatus = state; return this;}

    @Override
    public void exportToTable(LogTable table) {
    }
    
    @Override
    public void importFromTable(LogTable table) {
    }
    
    @Override
    public void updateInputs() {
        setShoulderState(hal.getShoulderPotEncoder().update());
        setElbowState(hal.getElbowPotEncoder().update());
    }

    @Override
    public void recordOutputs(Logger logger, String prefix) {
        shoulderStatus.recordOutputs(logger, prefix+"Shoulder Encoder");
        elbowStatus.recordOutputs(logger, prefix+"Elbow Encoder");
    }
}