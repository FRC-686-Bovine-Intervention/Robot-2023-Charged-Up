package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.intake.IntakeStatus.IntakeState;

public class IntakeLoop extends LoopBase {
    private static IntakeLoop instance;
    public static IntakeLoop getInstance(){if(instance == null) instance = new IntakeLoop(); return instance;}

    private final IntakeStatus status = IntakeStatus.getInstance();
    private final IntakeHAL HAL = IntakeHAL.getInstance();

    private static final double kDisabledNeutralModeThreshold = 5;
    private static final double kCurrentThreshold = 20;

    private IntakeLoop() {Subsystem = Intake.getInstance();}

    @Override
    protected void Enabled() {
        IntakeCommand newCommand = status.getIntakeCommand();

        // Determine new state
        switch (status.getIntakeState())
        {
            case Release:
                if(newCommand.getIntakeState() == IntakeState.Grab)
                    status.setIntakeState(IntakeState.Grab);
            break;

            case Grab:
                if(newCommand.getIntakeState() == IntakeState.Release)
                    status.setIntakeState(IntakeState.Release);
                if(status.getIntakeCurrent() > kCurrentThreshold)
                    status.setIntakeState(IntakeState.Hold);
            break;

            case Hold:
                if(newCommand.getIntakeState() == IntakeState.Release)
                    status.setIntakeState(IntakeState.Release);
            break;
        }

        // Execute state
        HAL.setDeploySolenoid(status.getIntakeState().intakeDeployed);
        HAL.setIntakeMotor(status.getIntakeState().intakePower);
        HAL.setIntakeNeutralMode(status.getIntakeState().intakeNeutralMode);
    }

    private double disabledTimestamp;
    @Override
    protected void Disabled() {
        if(status.Enabled.IsInitState)
            disabledTimestamp = Timer.getFPGATimestamp();
        
        if(Timer.getFPGATimestamp() - disabledTimestamp < kDisabledNeutralModeThreshold)
            HAL.setIntakeNeutralMode(status.getIntakeState().intakeNeutralMode);
        else
        {
            HAL.setIntakeNeutralMode(IdleMode.kCoast);
            status.setIntakeState(IntakeState.Release);
        }

        HAL.setDeploySolenoid(false);
        HAL.setIntakeMotor(0);
    }
    @Override
    protected void Update() {
        
    }
}
