package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.framework.StatusBase;

public class ArmStatus extends StatusBase {
    private static ArmStatus instance;
    public static ArmStatus getInstance(){if(instance == null) instance = new ArmStatus(); return instance;}

    private ArmStatus() {Subsystem = Arm.getInstance();}

    public enum ArmState {
        Defense,    // Arm idling, retracted fully and waiting to grab a piece
        Grab,       // Intake has a piece and arm is grabbing the piece from the intake
        Hold,       // Arm has grabbed the piece and is holding it in the defense position
        Align,      // Robot has entered the community and the turret should align to the alliance wall
        Extend,     // Driver has selected a node to extend to and arm is extending to it
        Release     // Driver has decided the piece will score on the node and tells the arm to release the piece
    }

    private ArmState armState = ArmState.Defense;
    public ArmState getArmState()                   {return armState;}
    public ArmStatus setArmState(ArmState armState) {this.armState = armState; return this;}

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
