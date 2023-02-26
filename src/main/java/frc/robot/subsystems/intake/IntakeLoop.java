package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.intake.IntakeStatus.IntakeState;

public class IntakeLoop extends LoopBase {
    private static IntakeLoop instance;
    public static IntakeLoop getInstance(){if(instance == null) instance = new IntakeLoop(); return instance;}

    private final IntakeStatus status = IntakeStatus.getInstance();

    private static final double kDisabledTimeThreshold = 5;
    private static final double kSpikeCurrentThreshold = 15;
    private static final double kSpikeTimeThreshold = 0.25;

    private IntakeLoop() {Subsystem = Intake.getInstance();}

    private double spikeStartTime;

    @Override
    protected void Enabled() {
        IntakeCommand newCommand = status.getIntakeCommand();
        double currentTime = Timer.getFPGATimestamp();

        // Determine new state                    (Don't change if current spike and grabbing)
        if(newCommand.getIntakeState() != null && (status.getIntakeState() != IntakeState.Grab || status.getIntakeCurrent() < kSpikeCurrentThreshold))
            status.setIntakeState(newCommand.getIntakeState());
        
        // Execute state
        switch (status.getIntakeState())
        {
            case Defense:
            break;

            case Release:
            break;

            case Grab:
                if(status.getIntakeCurrent() < kSpikeCurrentThreshold)
                    spikeStartTime = currentTime;
                if(currentTime - spikeStartTime > kSpikeTimeThreshold)
                    status.setIntakeState(IntakeState.Hold);
            break;

            case Hold:
            break;
        }

        status.setDeploySolenoid(status.getIntakeState().intakeDeployed)
              .setIntakePower(status.getIntakeState().intakePower)
              .setIntakeNeutralMode(status.getIntakeState().intakeNeutralMode);
    }

    private double disabledTimestamp;
    @Override
    protected void Disabled() {
        double currentTime = Timer.getFPGATimestamp();

        if(status.EnabledState.IsInitState)
            disabledTimestamp = currentTime;
        
        if(currentTime - disabledTimestamp < kDisabledTimeThreshold)
            status.setIntakeNeutralMode(status.getIntakeState().intakeNeutralMode);
        else
        {
            status.setIntakeNeutralMode(IdleMode.kCoast)
                  .setIntakeState(IntakeState.Defense);
        }

        status.setDeploySolenoid(false)
              .setIntakePower(0);
    }
    @Override
    protected void Update() {
        
    }
}
