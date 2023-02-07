package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import frc.robot.lib.sensorCalibration.PotAndEncoderStatus;
import frc.robot.subsystems.framework.StatusBase;

public class ArmStatus extends StatusBase {
    private static ArmStatus instance;
    public static ArmStatus getInstance(){if(instance == null){instance = new ArmStatus();}return instance;}

    private PotAndEncoderStatus shoulderPotAndEncoderStatus;
    private PotAndEncoderStatus elbowPotAndEncoderStatus;

    private double shoulderAngleDeg;
    private double elbowAngleDeg;
    public void setShoulderAngleDeg(double shoulderAngleDeg) {this.shoulderAngleDeg = shoulderAngleDeg; }
    public void setElbowAngleDeg(double elbowAngleDeg) {this.elbowAngleDeg = elbowAngleDeg; }
    public double getShoulderAngleDeg()   {return shoulderPotAndEncoderStatus.getPositionDeg();}
    public double getElbowAngleDeg()      {return elbowPotAndEncoderStatus.getPositionDeg();}


    private ArmStatus()
    {
        Subsystem = Arm.getInstance();

        shoulderPotAndEncoderStatus = new PotAndEncoderStatus(ArmHAL.getInstance().getShoulderPotAndEncoderHAL(), Arm.getInstance().getShoulderPotAndEncoderConfig());
        elbowPotAndEncoderStatus    = new PotAndEncoderStatus(ArmHAL.getInstance().getElbowPotAndEncoderHAL(), Arm.getInstance().getElbowPotAndEncoderConfig());
    }

    @Override
    public void exportToTable(LogTable table) {
    }
    
    @Override
    public void importFromTable(LogTable table) {
    }
    
    @Override
    public void updateInputs() {
        shoulderPotAndEncoderStatus.update();
        elbowPotAndEncoderStatus.update();

        setShoulderAngleDeg(shoulderPotAndEncoderStatus.getPositionDeg()); 
        setElbowAngleDeg(elbowPotAndEncoderStatus.getPositionDeg()); 
    }

    @Override
    public void recordOutputs(Logger logger, String prefix) {
    }
    
}
