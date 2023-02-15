package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.arm.ArmHAL.LimelightPipeline;
import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.driverInteraction.DriverInteractionStatus.DriverControlAxes;
import frc.robot.subsystems.framework.LoopBase;

public class ArmLoop extends LoopBase {
    private static ArmLoop instance;
    public static ArmLoop getInstance(){if(instance == null) instance = new ArmLoop(); return instance;}

    private final ArmStatus status = ArmStatus.getInstance();
    // private final Intake intake = Intake.getInstance();
    // private final IntakeStatus intakeStatus = IntakeStatus.getInstance();

    private static final double kTurretMaxAngularVelocity = 30;
    private static final double kTurretMaxAngularAcceleration = 10;
    private final TrapezoidProfile.Constraints turretPIDConstraints = new TrapezoidProfile.Constraints(kTurretMaxAngularVelocity, kTurretMaxAngularAcceleration);
    private final ProfiledPIDController turretPID = new ProfiledPIDController(0.0025, 0, 0, turretPIDConstraints);

    private ArmLoop() {Subsystem = Arm.getInstance();}

    @Override
    protected void Enabled() {
        switch(status.getArmState())
        {
            case Defense:
                // Set arm pos to defense
                // if(intakeStatus.getIntakeState() == IntakeState.Hold)
                //     status.setArmState(ArmState.IdentifyCone);
            break;

            case IdentifyCone:
                // Swap limelight pipeline
                status.setPipeline(LimelightPipeline.cone);
                // Check for cone in intake bounding box
                // If false, jump to IdentifyCube
                // If true, set turret target pos and jump to Grab
                if (status.getPipeline() == LimelightPipeline.cone) {
                    if(status.getTargetInView()){
                        status.setTargetTurretAngle(status.getTargetTurretAngle() + status.getTargetXOffset());
                        status.setArmState(ArmState.Grab);
                    } else {
                        // status.setArmState(ArmState.IdentifyCube);
                    }
                }
            break;
            
            case IdentifyCube:
                // Swap limelight pipeline
                status.setPipeline(LimelightPipeline.cube);
                // Check for cube in intake bounding box
                // If false, jump to IdentifyCone
                // System.out.print("I AM A CUBE");
                // If true, set turret target pos and jump to Grab
                if (status.getPipeline() == LimelightPipeline.cube) {
                    if(status.getTargetInView()){
                        status.setTargetTurretAngle(status.getTargetTurretAngle() + status.getTargetXOffset());
                        status.setArmState(ArmState.Grab);
                    } else {
                        //status.setArmState(ArmState.IdentifyCone);
                    }
                }
            break;

            case Grab:
                // Move turret to target pos
                turretPID.setGoal(status.getTargetTurretAngle());
                status.setTurretPower(turretPID.calculate(status.getTurretPosition()));
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
