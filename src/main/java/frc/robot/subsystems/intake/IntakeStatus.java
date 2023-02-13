package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.subsystems.framework.StatusBase;

public class IntakeStatus extends StatusBase {
    private static IntakeStatus instance;
    public static IntakeStatus getInstance(){if(instance == null) instance = new IntakeStatus(); return instance;}

    private static final Intake intake = Intake.getInstance();
    private static final IntakeHAL HAL = IntakeHAL.getInstance();

    private IntakeStatus() {Subsystem = Intake.getInstance();}

    public enum IntakeState {
        Defense (0,     false,  IdleMode.kCoast),
        Grab    (0.7,   true,   IdleMode.kCoast),
        Hold    (0.2,   false,  IdleMode.kBrake),
        Release (-0.7,  false,  IdleMode.kCoast);
        public final double intakePower;
        public final boolean intakeDeployed;
        public final IdleMode intakeNeutralMode;
        IntakeState(double intakePower, boolean intakeDeployed, IdleMode intakeNeutralMode)
        {
            this.intakePower = intakePower;
            this.intakeDeployed = intakeDeployed;
            this.intakeNeutralMode = intakeNeutralMode;
        }
    }

    private IntakeCommand intakeCommand = new IntakeCommand();
    public IntakeCommand getIntakeCommand()                             {return intakeCommand;}
    public IntakeStatus setIntakeCommand(IntakeCommand intakeCommand)   {this.intakeCommand = intakeCommand; return this;}

    private IntakeState intakeState = IntakeState.Release;
    public IntakeState getIntakeState()                         {return intakeState;}
    public IntakeStatus setIntakeState(IntakeState intakeState) {this.intakeState = intakeState; return this;}

    private double intakeCurrent;
    public double getIntakeCurrent()                            {return intakeCurrent;}
    public IntakeStatus setIntakeCurrent(double intakeCurrent)  {this.intakeCurrent = intakeCurrent; return this;}

    @Override
    protected void exportToTable(LogTable table) {
        table.put("Intake Motor Current", getIntakeCurrent());
    }
    @Override
    protected void importFromTable(LogTable table) {
        setIntakeCurrent(table.getDouble("Intake Motor Current", getIntakeCurrent()));
    }
    @Override
    protected void updateInputs() {
        setIntakeCommand(intake.getCommand());
        setIntakeCurrent(HAL.getIntakeCurrent());
    }
    @Override
    protected void recordOutputs(Logger logger, String prefix) {
        logger.recordOutput(prefix + "Intake State", getIntakeState().name());
        logger.recordOutput(prefix + "Command/Input State", (getIntakeCommand().getIntakeState() == null ? "null" : getIntakeCommand().getIntakeState().name()));
    }
}
