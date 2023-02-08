package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import frc.robot.lib.sensorCalibration.PotAndEncoderStatus;
import frc.robot.subsystems.framework.StatusBase;

public class ArmStatus extends StatusBase {
    private static ArmStatus instance;
    public static ArmStatus getInstance(){if(instance == null){instance = new ArmStatus();}return instance;}

    private final ArmHAL HAL = ArmHAL.getInstance();

    private final PotAndEncoderStatus shoulderPotAndEncoderStatus = new PotAndEncoderStatus(HAL.getShoulderPotAndEncoderHAL(), HAL.getShoulderPotAndEncoderConfig());
    private final PotAndEncoderStatus elbowPotAndEncoderStatus    = new PotAndEncoderStatus(HAL.getElbowPotAndEncoderHAL(), HAL.getElbowPotAndEncoderConfig());

    private double shoulderAngleDeg;
    public double getShoulderAngleDeg()                             {return shoulderAngleDeg;}
    public ArmStatus setShoulderAngleDeg(double shoulderAngleDeg)   {this.shoulderAngleDeg = shoulderAngleDeg; return this;}
    private double elbowAngleDeg;
    public double getElbowAngleDeg()                        {return elbowAngleDeg;}
    public ArmStatus setElbowAngleDeg(double elbowAngleDeg) {this.elbowAngleDeg = elbowAngleDeg; return this;}

    private ArmStatus() {Subsystem = Arm.getInstance();}

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
