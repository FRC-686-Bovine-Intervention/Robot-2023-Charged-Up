package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeStatus;
import frc.robot.subsystems.intake.IntakeStatus.IntakeState;

public class ArmLoop extends LoopBase {
    private static ArmLoop instance;
    public static ArmLoop getInstance(){if(instance == null) instance = new ArmLoop(); return instance;}

    private final ArmStatus status = ArmStatus.getInstance();
    private final Intake intake = Intake.getInstance();
    private final IntakeStatus intakeStatus = IntakeStatus.getInstance();

    private static final double kTurretMaxAngularVelocity = 135;
    private static final double kTurretMaxAngularAcceleration = 270;
    private final TrapezoidProfile.Constraints turretPIDConstraints = new TrapezoidProfile.Constraints(kTurretMaxAngularVelocity, kTurretMaxAngularAcceleration);
    private final ProfiledPIDController turretPID = new ProfiledPIDController(0.08, 0, 0, turretPIDConstraints);

    private ArmLoop() {Subsystem = Arm.getInstance();}

    private double stateStartTimestamp;
    private ArmState prevState;

    @Override
    protected void Enabled() {
        ArmCommand newCommand = status.getCommand();
        double currentTimestamp = Timer.getFPGATimestamp();

        if(newCommand.getArmState() != null)
            status.setArmState(newCommand.getArmState());
        
        if(prevState != status.getArmState())
            stateStartTimestamp = currentTimestamp;

        prevState = status.getArmState();

        status.setTurretPower(0.0);

        switch(status.getArmState())
        {
            case Defense:
                // Set arm pos to defense
                if(intakeStatus.getIntakeState() == IntakeState.Hold)
                    status.setArmState(ArmState.IdentifyPiece);
            break;

            case IdentifyPiece:
                // Check for piece in intake bounding box
            break;
            
            case Grab:
                // Move turret to target pos
                // Extend arm to grab piece
                // Grab piece
                // Set intake to release
                if(currentTimestamp - stateStartTimestamp >= 2)
                    intake.setCommand(new IntakeCommand(IntakeState.Release));
                // Jump to Hold
                if(currentTimestamp - stateStartTimestamp >= 3)
                    status.setArmState(ArmState.Hold);
            break;

            case Hold:
                intake.setCommand(new IntakeCommand(IntakeState.Defense));
                // Move arm to hold position
                // Check if robot is in community, if so jump to Align
            break;

            case Align:
                // Align turret to alliance wall
                // Check if robot is in not in community, if so jump to Hold
                // Check if driver has selected node, if so jump to Extend
            break;

            case Extend:
                // Extend to selected node
                // When completed, jump to Adjust
            break;

            case Adjust:
                // Rotate turret according to limelight and driver controls
                // Check if driver has pushed release button, if so jump to Release
            break;

            case Release:
                // Outtake piece
                // Wait a bit then jump to Defense
                if(currentTimestamp - stateStartTimestamp >= 1)
                    status.setArmState(ArmState.Defense);
            break;
        }
    }
    @Override
    protected void Disabled() {
        status.setTurretPower(0.0);
        status.setArmState(ArmState.Defense); //TODO: Add disable time threshold
    }
    @Override
    protected void Update() {
        
    }
}
