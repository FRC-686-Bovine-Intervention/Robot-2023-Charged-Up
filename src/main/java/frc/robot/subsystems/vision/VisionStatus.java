package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;

import frc.robot.subsystems.framework.StatusBase;

public class VisionStatus extends StatusBase {
    public static VisionStatus instance;
    public static VisionStatus getInstance(){
        if(instance == null){
            instance = new VisionStatus();
        }
        return instance;
    }

    @Override
    public void exportToTable(LogTable table, String prefix) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void importFromTable(LogTable table, String prefix) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void updateInputs() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void recordOutputs(String prefix) {
        // TODO Auto-generated method stub
        
    }
    
}
