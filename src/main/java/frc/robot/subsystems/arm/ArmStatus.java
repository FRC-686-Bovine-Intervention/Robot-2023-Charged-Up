package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.framework.StatusBase;

public class ArmStatus extends StatusBase {
    private static ArmStatus instance;
    public static ArmStatus getInstance(){if(instance == null) instance = new ArmStatus(); return instance;}

    private ArmStatus() {Subsystem = Arm.getInstance();}

    public enum ArmState {
        Defense,
        Grab,
        Hold,
        Align,
        Extend,
        Release
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
