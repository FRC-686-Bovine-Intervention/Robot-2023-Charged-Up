package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
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

    private static final double kTurretMaxAngularVelocity = 30;
    private static final double kTurretMaxAngularAcceleration = 10;

    private final TrapezoidProfile.Constraints turretPIDConstraints = new TrapezoidProfile.Constraints(kTurretMaxAngularVelocity, kTurretMaxAngularAcceleration);
    private final ProfiledPIDController turretPID = new ProfiledPIDController(0, 0, 0, turretPIDConstraints);

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
                HAL.setPipeline(Constants.kConePipeline);
                // Check for cone in intake bounding box
                // If false, jump to IdentifyCube
                // If true, set turret target pos and jump to Grab
                if(HAL.getTargetInView()){
                    status.setTargetTurretAngle(status.getTargetTurretAngle() + HAL.getTargetXOffset());
                } else {
                    status.setArmState(ArmState.IdentifyCube);
                    status.setArmState(ArmState.Grab);
                }
            break;
            
            case IdentifyCube:
                // Swap limelight pipeline
                HAL.setPipeline(Constants.kCubePipeline);
                // Check for cube in intake bounding box
                // If false, jump to IdentifyCone
                // If true, set turret target pos and jump to Grab
                if(HAL.getTargetInView()){
                    status.setTargetTurretAngle(status.getTargetTurretAngle() + HAL.getTargetXOffset());
                    status.setArmState(ArmState.Grab);
                } else {
                    status.setArmState(ArmState.IdentifyCube);
                }
            break;

            case Grab:
                // Move turret to target pos
                turretPID.setGoal(status.getTargetTurretAngle());
                HAL.setTurretPower(turretPID.calculate(HAL.getTurretPosition()));
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
