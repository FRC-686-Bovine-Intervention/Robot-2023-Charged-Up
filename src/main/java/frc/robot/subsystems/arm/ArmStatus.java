package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.framework.StatusBase;

public class ArmStatus extends StatusBase {
    private static ArmStatus instance;
    public static ArmStatus getInstance(){if(instance == null) instance = new ArmStatus(); return instance;}

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

    private double targetTurretAngle; //TODO: Units
    public ArmStatus setTargetTurretAngle(double angle){
        targetTurretAngle = angle;
        return this;
    }
    public double getTargetTurretAngle(){
        return targetTurretAngle;
    }

    @Override
    protected void exportToTable(LogTable table) {
        
    }

    @Override
    protected void importFromTable(LogTable table) {
        
    }

    @Override
    protected void updateInputs() {
        
    }

    @Override
    protected void recordOutputs(Logger logger, String prefix) {
        
    }
}
