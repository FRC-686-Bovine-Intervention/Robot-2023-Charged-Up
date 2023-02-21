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
        Grab    (1,     true,   IdleMode.kCoast),
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
    private IntakeStatus setIntakeCommand(IntakeCommand intakeCommand)  {this.intakeCommand = intakeCommand; return this;}

    private IntakeState intakeState = IntakeState.Defense;
    public IntakeState getIntakeState()                         {return intakeState;}
    public IntakeStatus setIntakeState(IntakeState intakeState) {this.intakeState = intakeState; return this;}

    private double intakeCurrent;
    public double getIntakeCurrent()                            {return intakeCurrent;}
    private IntakeStatus setIntakeCurrent(double intakeCurrent) {this.intakeCurrent = intakeCurrent; return this;}

    private double intakePower;
    public double getIntakePower()                          {return intakePower;}
    public IntakeStatus setIntakePower(double intakePower)  {this.intakePower = intakePower; return this;}

    private IdleMode intakeNeutralMode;
    public IdleMode getIntakeNeutralMode()                                  {return intakeNeutralMode;}
    public IntakeStatus setIntakeNeutralMode(IdleMode intakeNeutralMode)    {this.intakeNeutralMode = intakeNeutralMode; return this;}

    private boolean deploySolenoid;
    public boolean getDeploySolenoid()                              {return deploySolenoid;}
    public IntakeStatus setDeploySolenoid(boolean deploySolenoid)   {this.deploySolenoid = deploySolenoid; return this;}

    @Override
    protected void updateInputs() {
        setIntakeCommand(intake.getCommand());
        setIntakeCurrent(HAL.getIntakeCurrent());
    }
    @Override
    protected void exportToTable(LogTable table) {
        table.put("Intake Motor Current", intakeCurrent);
    }
    @Override
    protected void importFromTable(LogTable table) {
        setIntakeCurrent(table.getDouble("Intake Motor Current", intakeCurrent));
    }
    @Override
    protected void processOutputs(Logger logger, String prefix) {
        HAL.setIntakeMotor(intakePower)
           .setIntakeNeutralMode(intakeNeutralMode)
           .setDeploySolenoid(deploySolenoid);

        logger.recordOutput(prefix + "Intake State", intakeState.name());
        logger.recordOutput(prefix + "Intake Motor/Power", intakePower);
        logger.recordOutput(prefix + "Intake Motor/Neutral Mode", intakeNeutralMode.name());
        logger.recordOutput(prefix + "Intake Solenoid Deployed", deploySolenoid);
        intakeCommand.log(logger, prefix + "Command");
    }

    @Override protected void processTable() {}
}
