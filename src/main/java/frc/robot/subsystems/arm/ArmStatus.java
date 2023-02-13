package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import frc.robot.lib.sensorCalibration.PotAndEncoder;
import frc.robot.subsystems.framework.StatusBase;

public class ArmStatus extends StatusBase {
    private static ArmStatus instance;
    public static ArmStatus getInstance(){if(instance == null){instance = new ArmStatus();}return instance;}

    private final ArmHAL HAL = ArmHAL.getInstance();

    private PotAndEncoder.Status shoulderPotAndEncoderStatus;
    public PotAndEncoder.Status getShoulderStatus() {return shoulderPotAndEncoderStatus;}
    public ArmStatus setShoulderStatus(PotAndEncoder.Status shoulderPotAndEncoderStatus) {this.shoulderPotAndEncoderStatus = shoulderPotAndEncoderStatus; return this;}
    
    private PotAndEncoder.Status elbowPotAndEncoderStatus;
    public PotAndEncoder.Status getElbowStatus() {return elbowPotAndEncoderStatus;}
    public ArmStatus setElbowStatus(PotAndEncoder.Status elbowPotAndEncoderStatus) {this.elbowPotAndEncoderStatus = elbowPotAndEncoderStatus; return this;}

    private ArmStatus() {Subsystem = Arm.getInstance();}

    @Override
    public void exportToTable(LogTable table) {
        setShoulderStatus(HAL.getShoulderPotEncoder().exportToTable(table, "Shoulder PotEncoder"));
        setElbowStatus(HAL.getElbowPotEncoder().exportToTable(table, "Elbow PotEncoder"));
    }
    
    @Override
    public void importFromTable(LogTable table) {
        setShoulderStatus(HAL.getShoulderPotEncoder().importFromTable(table, "Shoulder PotEncoder", shoulderPotAndEncoderStatus.reading));
        setElbowStatus(HAL.getElbowPotEncoder().importFromTable(table, "Elbow PotEncoder", elbowPotAndEncoderStatus.reading));
    }
    
    @Override
    public void updateInputs() {
    }

    @Override
    public void recordOutputs(Logger logger, String prefix) {
        shoulderPotAndEncoderStatus.recordOutputs(logger, prefix + "Shoulder PotEncoder");
        elbowPotAndEncoderStatus.recordOutputs(logger, prefix + "Elbow PotEncoder");
    }
    
}
