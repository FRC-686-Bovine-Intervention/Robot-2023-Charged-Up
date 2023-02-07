package frc.robot.subsystems.arm;

import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeStatus;
import frc.robot.subsystems.intake.IntakeStatus.IntakeState;

public class ArmLoop extends LoopBase {
    private static ArmLoop instance;
    public static ArmLoop getInstance(){if(instance == null) instance = new ArmLoop(); return instance;}

    private final ArmStatus status = ArmStatus.getInstance();
    private final ArmHAL HAL = ArmHAL.getInstance();
    private final Intake intake = Intake.getInstance();
    private final IntakeStatus intakeStatus = IntakeStatus.getInstance();

    private ArmLoop() {Subsystem = Arm.getInstance();}

    @Override
    protected void Enabled() {
        switch(status.getArmState())
        {
            case Defense:
                // Set arm pos to defense
                if(intakeStatus.getIntakeState() == IntakeState.Hold)
                    status.setArmState(ArmState.IdentifyCone);
            break;

            case IdentifyCone:
                // Swap limelight pipeline
                // Check for cone in intake bounding box
                // If false, jump to IdentifyCube
                // If true, set turret target pos and jump to Grab
            break;
            
            case IdentifyCube:
                // Swap limelight pipeline
                // Check for cube in intake bounding box
                // If false, jump to IdentifyCone
                // If true, set turret target pos and jump to Grab
            break;

            case Grab:
                // Move turret to target pos
                // Extend arm to grab piece
                // Grab piece
                // Set intake to release
                // Jump to Hold
            break;

            case Hold:
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
            break;
        }
    }
    @Override
    protected void Disabled() {
        
    }
    @Override
    protected void Update() {
        
    }
}
