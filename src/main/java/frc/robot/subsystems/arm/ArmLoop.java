package frc.robot.subsystems.arm;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.FieldDimensions;
import frc.robot.lib.util.GeomUtil;
import frc.robot.subsystems.arm.ArmDynamics.JointConfig;
import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.arm.ArmStatus.MotorControlMode;
import frc.robot.subsystems.arm.json.ArmConfigJson;
import frc.robot.subsystems.arm.json.ArmPathsJson;
import frc.robot.subsystems.arm.json.ArmPresetsJson;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeStatus;
import frc.robot.subsystems.intake.IntakeStatus.IntakeState;
import frc.robot.subsystems.odometry.OdometryStatus;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionCommand;
import frc.robot.subsystems.vision.VisionStatus;
import frc.robot.subsystems.vision.VisionStatus.LimelightPipeline;
import frc.robot.util.AllianceFlipUtil;

public class ArmLoop extends LoopBase {
    private static ArmLoop instance;
    public static ArmLoop getInstance(){if(instance == null) instance = new ArmLoop(); return instance;}

    private final Arm arm = Arm.getInstance();
    private final ArmStatus status = ArmStatus.getInstance();
    private final Intake intake = Intake.getInstance();
    private final IntakeStatus intakeStatus = IntakeStatus.getInstance();
    private final OdometryStatus odometryStatus = OdometryStatus.getInstance();
    private final Vision vision = Vision.getInstance();
    private final VisionStatus visionStatus = VisionStatus.getInstance();

    private static final double kTurretMaxAngularVelocity = 180;
    private static final double kTurretMaxAngularAcceleration = kTurretMaxAngularVelocity * 2;
    private final TrapezoidProfile.Constraints turretPIDConstraints = new TrapezoidProfile.Constraints(kTurretMaxAngularVelocity, kTurretMaxAngularAcceleration);
    private static final double kTurretFastP = 0.015;
    private static final double kTurretSlowP = 0.015;
    private final ProfiledPIDController turretPID = 
        new ProfiledPIDController(
            0.015, 
            0, 
            0, 
            turretPIDConstraints
        );
    private static final double kTurretPIDMaxError = 10;
    private static final double kTurretExtendMaxError = 3;

    private static final double kDistalZeroPower =      0.15;
    private static final double kProximalZeroPower =    0.1;
    private static final double kTurretZeroPower =      0.2;

    private static final double kDistalZeroErrorThreshold = Units.degreesToRadians(2.5);
    private static final double kTurretZeroErrorThreshold = 2.5;
    private static final double kProximalZeroErrorThreshold = Units.degreesToRadians(2.5);

    private static final double kDistalZeroRadUp = Units.degreesToRadians(150);

    private ArmState prevState;

    private static final double kDisabledResetTimerThreshold = 5;

    // private final String configJson;
    private final ArmPresetsJson presets;
    private final ArmKinematics kinematics;
    private final ArmDynamics dynamics;    
    
    private final ArmTrajectory[][] armTrajectories = new ArmTrajectory[ArmPose.Preset.values().length+1][ArmPose.Preset.values().length+1];
    // private ArmTrajectory currentTrajectory = null;
    private final Timer trajectoryTimer = new Timer();

    private Matrix<N2,N3> setpointState = null;     
    private Matrix<N2,N3> finalTrajectoryState = null;

    private final PIDController shoulderPID = 
        new PIDController(
            20.0, 
            0.0, 
            0.0, 
            Constants.loopPeriodSecs
        );
    private final PIDController elbowPID = 
        new PIDController(
            15.0, 
            0.0, 
            0.0, 
            Constants.loopPeriodSecs
        );    

    private final Timer internalDisableTimer = new Timer();
    public static final double internalDisableMaxError = Units.degreesToRadians(10.0);    
    public static final double internalDisableMaxErrorTime = 0.5;
    public static final double internalDisableBeyondLimitThreshold = Units.degreesToRadians(5.0);
    
    private Supplier<Boolean> disableSupplier = () -> false;

    private final double shoulderMaxAngleRad;
    private final double shoulderMinAngleRad;
    private final double elbowMaxAngleRad;
    private final double elbowMinAngleRad;
    public static final double kRelativeMaxAngleRad = Math.toRadians(180.0);    // don't let grabber smash into proximal arm
    public static final double kRelativeMinAngleRad = Math.toRadians(-135.0);   // we'll probably never need this one

    private static final double kMaxElbowPlusClawLength = Units.inchesToMeters(33.0); 
 
    private final double xMinSetpoint = Units.inchesToMeters(0.0);
    private final double xMaxSetpoint;  // calculated
    private final double zMinSetpoint = Units.inchesToMeters(0.0);
    private final double zMaxSetpoint = Units.inchesToMeters(72.0); 

    private final double manualMaxArmAdjustmentRangeDegrees = 45.0;
    private final double manualMaxArmAdjustmentRangeRadians = Units.degreesToRadians(manualMaxArmAdjustmentRangeDegrees);

    private final double manualMaxShoulderSpeedDegreesPerSec = 30.0 * 1.5;    // speed the arm is allowed to extend manually in the turret's XZ plane
    private final double manualMaxShoulderSpeedRadiansPerSec = Units.degreesToRadians(manualMaxShoulderSpeedDegreesPerSec);
    private final double manualMaxElbowSpeedDegreesPerSec = 45.0 * 1.5;    // speed the arm is allowed to extend manually in the turret's XZ plane
    private final double manualElbowRaisedDegrees = 15.0;    // speed the arm is allowed to extend manually in the turret's XZ plane
    private final double manualMaxElbowSpeedRadiansPerSec = Units.degreesToRadians(manualMaxElbowSpeedDegreesPerSec);
    private final double manualMaxTurretPercentOutput = 0.2;  // speed the turret is allowed to manually spin
    private final double manualMaxShoulderPercentOutput = 0.2;  // speed the shoulder is allowed to manually spin when in emergency mode
    private final double manualMaxElbowPercentOutput = 0.3;  // speed the elbow is allowed to manually spin when in emergency mode


    private ArmLoop() {
        Subsystem = arm;

        // Get config from JSON
        ArmConfigJson config = arm.getConfig();

        // JointConfig claw = ArmDynamics.rigidlyCombineJoints(config.wrist(), config.cone());
        // JointConfig elbow = ArmDynamics.rigidlyCombineJoints(config.elbow(), claw);
        JointConfig elbow = ArmDynamics.rigidlyCombineJoints(config.elbow(), config.wrist());
        JointConfig shoulder = config.shoulder();

        kinematics = new ArmKinematics(new Translation2d(config.origin().getX(), config.origin().getY()),
                                        shoulder.length(), elbow.length(),
                                        config.shoulder().minAngle(), config.shoulder().maxAngle(), 
                                        elbow.minAngle(), elbow.maxAngle(),
                                        kRelativeMinAngleRad, kRelativeMaxAngleRad);

        dynamics = new ArmDynamics(shoulder, elbow);

        shoulderMinAngleRad = config.shoulder().minAngle();
        shoulderMaxAngleRad = config.shoulder().maxAngle();
        elbowMinAngleRad = config.elbow().minAngle();
        elbowMaxAngleRad = config.elbow().maxAngle();
        
        
        // Get presets from JSON
        File presetFile = new File(Filesystem.getDeployDirectory(), ArmPresetsJson.jsonFilename);
        presets = ArmPresetsJson.loadJson(presetFile);   
        ArmPose.Preset.writePresets(presets);
        
        // Get paths from JSON
        // also create trajectories for each path
        loadArmTrajectory(ArmPose.Preset.DEFENSE, ArmPose.Preset.INTAKE);
        loadArmTrajectory(ArmPose.Preset.DEFENSE, ArmPose.Preset.DOUBLE_SUBSTATION);

        loadArmTrajectory(ArmPose.Preset.HOLD, ArmPose.Preset.DEFENSE);
        loadArmTrajectory(ArmPose.Preset.DOUBLE_SUBSTATION, ArmPose.Preset.DEFENSE);

        loadArmTrajectory(ArmPose.Preset.INTAKE, ArmPose.Preset.HOLD);
        loadArmTrajectory(ArmPose.Preset.DOUBLE_SUBSTATION, ArmPose.Preset.HOLD);

        loadArmTrajectory(ArmPose.Preset.AUTO_START, ArmPose.Preset.SCORE_HYBRID);
        loadArmTrajectory(ArmPose.Preset.AUTO_START, ArmPose.Preset.SCORE_MID_CUBE);
        loadArmTrajectory(ArmPose.Preset.AUTO_START, ArmPose.Preset.SCORE_HIGH_CUBE);
        loadArmTrajectory(ArmPose.Preset.AUTO_START, ArmPose.Preset.SCORE_MID_CONE);
        loadArmTrajectory(ArmPose.Preset.AUTO_START, ArmPose.Preset.SCORE_HIGH_CONE);

        loadArmTrajectory(ArmPose.Preset.HOLD, ArmPose.Preset.SCORE_HYBRID);
        loadArmTrajectory(ArmPose.Preset.HOLD, ArmPose.Preset.SCORE_MID_CUBE);
        loadArmTrajectory(ArmPose.Preset.HOLD, ArmPose.Preset.SCORE_HIGH_CUBE);
        loadArmTrajectory(ArmPose.Preset.HOLD, ArmPose.Preset.SCORE_MID_CONE);
        loadArmTrajectory(ArmPose.Preset.HOLD, ArmPose.Preset.SCORE_HIGH_CONE);

        loadArmTrajectory(ArmPose.Preset.SCORE_HYBRID, ArmPose.Preset.DEFENSE);
        loadArmTrajectory(ArmPose.Preset.SCORE_MID_CUBE, ArmPose.Preset.DEFENSE);
        loadArmTrajectory(ArmPose.Preset.SCORE_HIGH_CUBE, ArmPose.Preset.DEFENSE);
        loadArmTrajectory(ArmPose.Preset.SCORE_MID_CONE, ArmPose.Preset.DEFENSE);
        loadArmTrajectory(ArmPose.Preset.SCORE_HIGH_CONE, ArmPose.Preset.DEFENSE);

        loadArmTrajectory(ArmPose.Preset.SCORE_HYBRID, ArmPose.Preset.HOLD);
        loadArmTrajectory(ArmPose.Preset.SCORE_MID_CUBE, ArmPose.Preset.HOLD);
        loadArmTrajectory(ArmPose.Preset.SCORE_HIGH_CUBE, ArmPose.Preset.HOLD);
        loadArmTrajectory(ArmPose.Preset.SCORE_MID_CONE, ArmPose.Preset.HOLD);
        loadArmTrajectory(ArmPose.Preset.SCORE_HIGH_CONE, ArmPose.Preset.HOLD);

        double clawLengthPastToolCenterPoint = kMaxElbowPlusClawLength - config.elbow().length() - config.wrist().length();
        xMaxSetpoint = Units.inchesToMeters(config.frame_width_inches() + 48.0 - clawLengthPastToolCenterPoint);
        resetTrajectoryState();
    }


    private void loadArmTrajectory(ArmPose.Preset startPos, ArmPose.Preset finalPos)
    {
        int startIdx = startPos.getFileIdx();
        int finalIdx = finalPos.getFileIdx();

        String pathFilename = String.format(ArmPathsJson.jsonFilename, startIdx, finalIdx);
        
        File pathFile = new File(Filesystem.getDeployDirectory(), pathFilename);
        var path = ArmPathsJson.loadJson(pathFile);

        // create trajectory for each path
        List<Vector<N2>> points = new ArrayList<>();
        for (int k=0; k<path.theta1().size(); k++) {
            points.add(VecBuilder.fill(path.theta1().get(k), path.theta2().get(k)));
        }

        armTrajectories[startIdx][finalIdx] = new ArmTrajectory(path.startPos(), path.finalPos(), path.totalTime(), path.grannyFactor(), points);
    }

    double clawGrabTimestamp;

    @Override
    protected void Update() {
        checkArmCalibration();

        status.setTurretPIDPosition(turretPID.getSetpoint().position);
        status.setTurretPIDVelocity(turretPID.getSetpoint().velocity);
    }
    
    // call this every update cycle
    public void checkArmCalibration() {
        ArmStatus status = ArmStatus.getInstance();
        if (!status.getShoulderFalconCalibrated() && (status.getShoulderPotEncStatus().calibrated)) {
            double shoulderAngleRad = Units.degreesToRadians(status.getShoulderPotEncStatus().positionDeg);
            status.setShoulderCalibAngleRad(shoulderAngleRad);
            status.setShoulderMinAngleRad(shoulderMinAngleRad);
            status.setShoulderMaxAngleRad(shoulderMaxAngleRad);
            status.setShoulderFalconCalibrated(true);
        }
        if (!status.getElbowFalconCalibrated() && status.getElbowPotEncStatus().calibrated) {
            double elbowAngleRad = Units.degreesToRadians(status.getElbowPotEncStatus().positionDeg);
            status.setElbowCalibAngleRad(elbowAngleRad);
            status.setElbowMinAngleRad(elbowMinAngleRad);
            status.setElbowMaxAngleRad(elbowMaxAngleRad);
            status.setElbowFalconCalibrated(true);
        }
    }
    
    private final Timer stateTimer = new Timer();
    
    @Override
    protected void Enabled() {
        // ================= Pulling Data from Command =================
        ArmCommand newCommand = status.getCommand();

        if(newCommand.getTargetNode() != null) {
            switch(status.getArmState()) {
                case AlignNode:
                case Extend:
                case Release:
                break;
                default:
                    status.setTargetNode(newCommand.getTargetNode());
                break;
            }
        }
        if(newCommand.getArmState() != null)
            status.setArmState(newCommand.getArmState());
        if(newCommand.getShoulderAdjustment() != null)
            status.setshoulderThrottle(newCommand.getShoulderAdjustment());
        if(newCommand.getElbowAdjustment() != null)
            status.setElbowThrottle(newCommand.getElbowAdjustment());
        if(newCommand.getElbowRaised() != null)
            status.setElbowRaised(newCommand.getElbowRaised());
        if(newCommand.getTurretAdjustment() != null)
            status.setTurretThrottle(newCommand.getTurretAdjustment());

        stateTimer.start();

        status.setTurretPower(0.0)
              .setTurretNeutralMode(NeutralMode.Brake)
              .setShoulderPower(0)
              .setElbowPower(0);

        LimelightPipeline pipeline = LimelightPipeline.Cone;
        // ================= Trajectory Logic =================

        runTrajectory();

        // ================= State Logic =================

        // Get measured positions
        double shoulderAngleRad = status.getShoulderAngleRad();
        double elbowAngleRad = status.getElbowAngleRad();

        if(prevState != status.getArmState())
            stateTimer.reset();

        prevState = status.getArmState();

        switch(status.getArmState())
        {
            case ZeroDistalUp:
                status.setInternalDisable(true, "Bringing Distal up")     // disable PID during zeroing
                      .setShoulderPower(0)
                      .setCurrentArmPose(null)
                      .setClawGrabbing(false)
                      .setTurretControlMode(MotorControlMode.PercentOutput)
                      .setTurretPower(0);
                if(status.getElbowPotEncStatus().calibrated && status.getShoulderPotEncStatus().calibrated)
                {
                    status.setElbowPower(kDistalZeroPower * Math.signum(shoulderAngleRad + kDistalZeroRadUp - elbowAngleRad));
                    if(Math.abs(shoulderAngleRad + kDistalZeroRadUp - elbowAngleRad) <= kDistalZeroErrorThreshold || Math.abs(shoulderAngleRad - ArmPose.Preset.DEFENSE.getShoulderAngleRad()) <= kProximalZeroErrorThreshold)
                        status.setArmState(ArmState.ZeroProximal);
                }
            break;
            
            case ZeroProximal:
                status.setCurrentArmPose(null)
                      .setInternalDisable(true, "Zeroing Proximal")
                      .setClawGrabbing(false)
                      .setTurretControlMode(MotorControlMode.PercentOutput)
                      .setTurretPower(0);
                if(status.getShoulderPotEncStatus().calibrated)
                {
                    status.setElbowPower(kProximalZeroPower * Math.signum(ArmPose.Preset.DEFENSE.getShoulderAngleRad() - shoulderAngleRad) * ArmStatus.kElbowMotorGearRatio / ArmStatus.kShoulderMotorGearRatio);
                    status.setShoulderPower(kProximalZeroPower * Math.signum(ArmPose.Preset.DEFENSE.getShoulderAngleRad() - shoulderAngleRad));
                    if(Math.abs(shoulderAngleRad - ArmPose.Preset.DEFENSE.getShoulderAngleRad()) <= kProximalZeroErrorThreshold)
                        status.setArmState(ArmState.ZeroTurret);
                }
            break;

            case ZeroTurret:
                status.setElbowPower(0)
                      .setTurretControlMode(MotorControlMode.PercentOutput)
                      .setTurretPower(kTurretZeroPower * Math.signum(-status.getTurretAngleDeg()))
                      .setInternalDisable(true, "Zeroing Turret")
                      .setClawGrabbing(false);
                if(Math.abs(status.getTurretAngleDeg()) <= kTurretZeroErrorThreshold)
                    status.setArmState(ArmState.ZeroDistal);
            break;

            case ZeroDistal:
                status.setCurrentArmPose(null)
                      .setInternalDisable(true, "Zeroing Distal")
                      .setClawGrabbing(false);
                if(status.getElbowPotEncStatus().calibrated)
                {
                    status.setElbowPower(kDistalZeroPower * Math.signum(ArmPose.Preset.DEFENSE.getElbowAngleRad() - elbowAngleRad));
                    if(Math.abs(elbowAngleRad - ArmPose.Preset.DEFENSE.getElbowAngleRad()) <= kDistalZeroErrorThreshold)
                    {
                        resetTrajectoryState();
                        status.setArmState(ArmState.Defense)
                              .setCurrentArmPose(ArmPose.Preset.DEFENSE)
                              .setTargetArmPose(ArmPose.Preset.DEFENSE)
                              .setInternalDisable(false, "")
                              .setElbowAdjustment(0)
                              .setShoulderAdjustment(0);     // enable arm trajectories
                    }
                }
            break;

            case Defense:
                if(stateTimer.get() == 0) {
                    intake.setCommand(new IntakeCommand(IntakeState.Defense));
                }
                status.setTurretControlMode(MotorControlMode.PID)
                      .setTargetTurretAngleDeg(0)
                      .setTargetArmPose(ArmPose.Preset.DEFENSE)
                      .setClawGrabbing(false);
                if(intakeStatus.getIntakeState() == IntakeState.Hold)
                    status.setArmState(ArmState.Grab);
            break;

            case Emergency:
                if(stateTimer.get() == 0) {
                    status.recalFalcons();
                }
                status.setTurretControlMode(MotorControlMode.PercentOutput)
                      .setTurretPower(status.getTurretThrottle() * manualMaxTurretPercentOutput)
                      .setShoulderPower(status.getShoulderThrottle() * manualMaxShoulderPercentOutput)
                      .setElbowPower(status.getElbowThrottle() * manualMaxElbowPercentOutput)
                      .setClawGrabbing(false);
            break;
            
            case Grab:
                status.setTurretControlMode(MotorControlMode.PercentOutput)
                      .setTurretPower(0)
                      .setTurretNeutralMode(NeutralMode.Coast);
                if(!stateTimer.hasElapsed(1))
                    break;
                if(!status.getClawGrabbing()) {
                    status.setTargetArmPose(ArmPose.Preset.INTAKE);
                    if(status.getCurrentArmPose() == ArmPose.Preset.INTAKE)
                    {
                        // Grab piece
                        status.setClawGrabbing(true);
                        clawGrabTimestamp = stateTimer.get();
                    }
                } else {
                    // Set intake to release
                    if(stateTimer.hasElapsed(clawGrabTimestamp + 0.5)) {
                        intake.setCommand(new IntakeCommand(IntakeState.Release));
                    }
                    if(stateTimer.hasElapsed(clawGrabTimestamp + 0.75)) {
                        status.setTargetArmPose(ArmPose.Preset.HOLD);
                        if(status.getCurrentArmPose() == ArmPose.Preset.HOLD)
                            status.setArmState(ArmState.Hold);
                    }
                }
            break;

            case Hold:
                intake.setCommand(new IntakeCommand(IntakeState.Defense));
                // Move arm to hold position
                status.setTargetArmPose(ArmPose.Preset.HOLD)
                      .setTargetTurretAngleDeg(0)
                      .setTurretControlMode(MotorControlMode.PID)
                      .setClawGrabbing(true);
                // Check if robot is in community, if so jump to Align
                if(stateTimer.get() == 0)
                    status.setStateLocked(false);

                if(newCommand.getArmState() == ArmState.Hold)
                    status.setStateLocked(true);

                if(FieldDimensions.chargeStation.withinBounds(odometryStatus.getRobotPose()))
                    status.setStateLocked(true);
                    
                // if(!status.getStateLocked() && FieldDimensions.communityWithoutChargeStation.withinBounds(odometryStatus.getRobotPose()))
                //     status.setArmState(ArmState.AlignWall);
            break;

            case AlignWall:
                turretPID.setP(kTurretFastP);
                status.setTargetArmPose(ArmPose.Preset.HOLD);
                pipeline = LimelightPipeline.Pole;
                
                // unwrap turret angle because odometry wraps it to +/-180
                
                double turretAngleDeg = getTurretBestAngle(
                    new Translation2d(
                        (DriverStation.getAlliance() == Alliance.Red ? 10 : -10) + odometryStatus.getRobotPose().getX(), 
                        odometryStatus.getRobotPose().getY()
                    )
                );

                // Align turret to alliance wall
                status.setTargetTurretAngleDeg(turretAngleDeg)
                      .setTurretControlMode(MotorControlMode.PID);
                
                // Check if robot is in not in community, if so jump to Hold
                if(stateTimer.get() == 0)
                    status.setStateLocked(false);

                if(newCommand.getArmState() == ArmState.AlignWall)
                    status.setStateLocked(true);

                // if(!status.getStateLocked() && !FieldDimensions.communityWithoutChargeStation.withinBounds(odometryStatus.getRobotPose()))
                //     status.setArmState(ArmState.Hold);
                // Check if driver has selected node, if so jump to Extend
            break;

            case AlignNode:
                pipeline = LimelightPipeline.Pole;
                status.setTargetTurretAngleDeg(getTurretBestAngle(getClosestNodeXY(status.getTargetNode(), status.getTurretToField().getTranslation().toTranslation2d()).get()))
                      .setTurretControlMode(MotorControlMode.PID);
                if(status.getTargetNode().isCone && visionStatus.getCurrentPipeline() == pipeline && visionStatus.getTargetExists()) {
                    turretPID.setP(kTurretSlowP);
                    status.setTargetTurretAngleDeg(status.getTurretAngleDeg() + visionStatus.getTargetYAngle())
                          .setArmState(ArmState.Extend);
                }
                if(!status.getTargetNode().isCone || Math.abs(status.getTargetTurretAngleDeg() - status.getTurretAngleDeg()) < kTurretExtendMaxError) {
                    status.setArmState(ArmState.Extend);
                }
            break;

            case Extend:
                // Extend to selected node
                status.setTargetArmPose(status.getTargetNode().armPreset);
                // When completed, jump to Adjust
                if(status.getCurrentArmPose() == status.getTargetArmPose())
                    status.setArmState(ArmState.Adjust);
            break;

            case Adjust:
                turretPID.setP(kTurretFastP);
                if(stateTimer.get() == 0)
                    status.setElbowRaised(false);
                // Rotate turret according to limelight and driver controls
                status.setTurretControlMode(MotorControlMode.PercentOutput)
                      .setTurretPower(status.getTurretThrottle() * manualMaxTurretPercentOutput);
                // Check if driver has pushed release button, if so jump to Release
            break;

            case Release:
                // Outtake piece
                status.setClawGrabbing(false);
                // Wait a bit then jump to Defense
                if(stateTimer.hasElapsed(0.25))
                    status.setTargetArmPose(ArmPose.Preset.HOLD);
                if(status.getCurrentArmPose() == ArmPose.Preset.HOLD)
                    status.setArmState(ArmState.Defense);
            break;

            case SubstationExtend:
                if(stateTimer.get() == 0) {
                    status.setElbowRaised(false);
                }
                status.setTargetArmPose(ArmPose.Preset.DOUBLE_SUBSTATION);
            break;

            case SubstationGrab:
                status.setClawGrabbing(true);
                if(stateTimer.hasElapsed(1))
                    status.setArmState(ArmState.Hold);
            break;
        }

        vision.setVisionCommand(new VisionCommand(pipeline));

        runTurret();

        if(status.getCurrentArmPose() != null && status.getTargetArmPose() != status.getCurrentArmPose())
            startTrajectory(status.getCurrentArmPose(), status.getTargetArmPose());
    }

    private boolean largeTurretError = false;
    private void runTurret() {
        turretPID.setGoal(status.getTargetTurretAngleDeg());
        if(status.getTargetTurretAngleDeg() - status.getTurretAngleDeg() >= kTurretPIDMaxError) {
            if(!largeTurretError) {
                turretPID.reset(status.getTurretAngleDeg());
            }
            largeTurretError = true;
        } else {
            largeTurretError = false;
        }
        status.setTurretPIDOutput(turretPID.calculate(status.getTurretAngleDeg()));
        if(status.getTurretControlMode() == MotorControlMode.PID)
            status.setTurretPower(status.getTurretPIDOutput());
    }

    public void startTrajectory(ArmPose.Preset startPos, ArmPose.Preset finalPos) {

        Translation2d turretXY = GeomUtil.translation3dTo2dXY(status.getTurretToField().getTranslation());
        double turretAngleToTarget = 0.0;

        // get current arm positions
        double startShoulderAngleRad = status.getShoulderAngleRad();
        double startElbowAngleRad = status.getElbowAngleRad();

        double finalShoulderAngleRad = finalPos.getShoulderAngleRad();
        double finalElbowAngleRad = finalPos.getElbowAngleRad();

        // get pre-planned trajectory
        ArmTrajectory baseTrajectory = armTrajectories[startPos.getFileIdx()][finalPos.getFileIdx()];

        // if trying to score, find closest node
        if ((finalPos.ordinal() >= ArmPose.Preset.SCORE_HYBRID.ordinal()) && 
            (finalPos.ordinal() <= ArmPose.Preset.SCORE_HIGH_CONE.ordinal())) {

            ArmStatus.NodeEnum targetNode = status.getTargetNode();

            // find closest node
            Optional<Translation2d> optNodeXY = getClosestNodeXY(targetNode, turretXY);

            if (optNodeXY.isPresent()) {
                Translation2d nodeXY = optNodeXY.get();

                // calculate turret angle to target
                Translation2d turretToTarget = nodeXY.minus(turretXY);
                turretAngleToTarget = getTurretBestAngle(nodeXY);

                // extend the distance of the base trajectory
                double x = turretToTarget.getNorm();
                double z = Units.inchesToMeters(finalPos.getZ());
                Optional<Vector<N2>> theta = kinematics.inverse(new Translation2d(x,z));
                if (theta.isPresent()) {
                    finalShoulderAngleRad = theta.get().get(0,0);
                    finalElbowAngleRad = theta.get().get(1,0);
                } else {
                    // if the arm can't reach the target node, reach out as far as we can
                    finalShoulderAngleRad = -0.1;   // should reach to 60" extension, 53" above floor
                    finalElbowAngleRad = 0.28;      // hardcoding to make sure we always get a result
                }
            }
            // set outputs
            // status.setTargetTurretAngleDeg(turretAngleToTarget);
        }
        status.setCurrentArmTrajectory(baseTrajectory.interpolateEndPoints(startShoulderAngleRad, startElbowAngleRad, finalShoulderAngleRad, finalElbowAngleRad));
        trajectoryTimer.reset();
    }


    public void resetTrajectoryState()  {
        finalTrajectoryState = armTrajectories[ArmPose.Preset.HOLD.getFileIdx()][ArmPose.Preset.DEFENSE.getFileIdx()].getFinalState();
        setpointState = finalTrajectoryState;
    }


    public void runTrajectory() {

        if (!status.getShoulderPotEncStatus().calibrated || !status.getElbowPotEncStatus().calibrated) {
            // do not run trajectories until our arm angles have been calibrated
            return;
        }
               
        // check if current trajectory is finished
        if (status.getCurrentArmTrajectory() != null && trajectoryTimer.hasElapsed(status.getCurrentArmTrajectory().getTotalTime()))
        {
            status.setCurrentArmTrajectory(null)
                  .setCurrentArmPose(status.getTargetArmPose())
                  .setShoulderAdjustment(0)
                  .setElbowAdjustment(0);
        }

        // Get measured positions
        double shoulderAngleRad = status.getShoulderAngleRad();
        double elbowAngleRad = status.getElbowAngleRad();
        
        // default setpoint
        setpointState = finalTrajectoryState;

        Vector<N2> voltages = VecBuilder.fill(0,0);
        double shoulderPIDOutput = 0;
        double elbowPIDOutput = 0;


        // if internally disabled, set the setpoint to the current position (don't move when enabling)
        if (status.getInternalDisable()) {
            setpointState = ArmTrajectory.getFixedState(shoulderAngleRad, elbowAngleRad);
            status.setCurrentArmTrajectory(null);
        } else {
            // move arm!
            if (status.getCurrentArmTrajectory() != null) {
                status.setCurrentArmPose(null);
                // follow trajectory
                trajectoryTimer.start();                            // multiple calls to start will not restart timer
                finalTrajectoryState = status.getCurrentArmTrajectory().getFinalState();

                // get setpoint from current trajectory
                setpointState = status.getCurrentArmTrajectory().sample(trajectoryTimer.get());

            } else {
                // update manual adjustments
                // shoulderThrottle and elbowThrottle are assumed to be joystick inputs in the range [-1, +1]
                double shoulderIncr = status.getShoulderThrottle() * manualMaxShoulderSpeedRadiansPerSec * Constants.loopPeriodSecs;
                double elbowIncr = status.getElbowThrottle() * manualMaxElbowSpeedRadiansPerSec * Constants.loopPeriodSecs;

                // clamp manual adjustments
                double shoulderAdj = MathUtil.clamp(status.getShoulderAdjustment() + shoulderIncr, -manualMaxArmAdjustmentRangeRadians, +manualMaxArmAdjustmentRangeRadians);
                double elbowAdj = MathUtil.clamp(status.getElbowAdjustment() + elbowIncr, -manualMaxArmAdjustmentRangeRadians, +manualMaxArmAdjustmentRangeRadians);

                elbowAdj += (status.getElbowRaised() ? Units.degreesToRadians(manualElbowRaisedDegrees) : 0);

                Vector<N2> thetaFinalTrajectory = new Vector<>(finalTrajectoryState.extractColumnVector(0));
                double shoulderNew = thetaFinalTrajectory.get(0,0) + shoulderAdj;
                double elbowNew = thetaFinalTrajectory.get(1,0) + elbowAdj;

                // check angular limits
                if (shoulderNew <= shoulderMinAngleRad) {
                    shoulderIncr = Math.max(shoulderIncr, 0);   // allow positive movement
                } else if (shoulderNew >= shoulderMaxAngleRad) {
                    shoulderIncr = Math.min(shoulderIncr, 0);   // allow negative movement
                }

                if (elbowNew <= elbowMinAngleRad) {
                    elbowIncr = Math.max(elbowIncr, 0);     // allow positive movement
                } else if (elbowNew >= elbowMaxAngleRad) {
                    elbowIncr = Math.min(elbowIncr, 0);     // allow negative movement
                }

                // check frame perimeter limits
                Translation2d xz = kinematics.forward(shoulderNew, elbowNew);
                if ((xz.getX() >= xMaxSetpoint) || (xz.getY() >= zMaxSetpoint)) {
                    // allow negative movement
                    shoulderIncr = Math.min(shoulderIncr, 0.0);
                    elbowIncr = Math.min(elbowIncr, 0.0);
                }
                if ((xz.getX() <= xMinSetpoint) || (xz.getY() <= zMinSetpoint)) {
                    // allow positive movement
                    shoulderIncr = Math.max(shoulderIncr, 0.0);
                    elbowIncr = Math.max(elbowIncr, 0.0);
                }


                // redo setpoint calcs after clamping increments
                shoulderAdj = MathUtil.clamp(status.getShoulderAdjustment() + shoulderIncr, -manualMaxArmAdjustmentRangeRadians, +manualMaxArmAdjustmentRangeRadians);
                elbowAdj = MathUtil.clamp(status.getElbowAdjustment() + elbowIncr, -manualMaxArmAdjustmentRangeRadians, +manualMaxArmAdjustmentRangeRadians);

                // elbowAdj += (status.getElbowRaised() ? Units.degreesToRadians(manualElbowRaisedDegrees) : 0);

                thetaFinalTrajectory = new Vector<>(finalTrajectoryState.extractColumnVector(0));
                shoulderNew = thetaFinalTrajectory.get(0,0) + shoulderAdj;
                elbowNew = thetaFinalTrajectory.get(1,0) + elbowAdj;

                status.setShoulderAdjustment(shoulderAdj)
                      .setElbowAdjustment(elbowAdj);
                
                setpointState = ArmTrajectory.getFixedState(shoulderNew, elbowNew);
            }

            // calculate feedforward voltages
            voltages = dynamics.feedforward(setpointState);

            // calculate feedback voltages (current location compared to where we wanted to go to last cycle)
            double shoulderAngleSetpoint = setpointState.get(0,0);
            double elbowAngleSetpoint = setpointState.get(1,0);            
            shoulderPIDOutput = shoulderPID.calculate(shoulderAngleRad, shoulderAngleSetpoint);
            elbowPIDOutput = elbowPID.calculate(elbowAngleRad, elbowAngleSetpoint) ;
            
            // set motors to achieve the setpoint
            status.setShoulderVoltage(voltages.get(0,0) + shoulderPIDOutput)
                  .setElbowVoltage   (voltages.get(1,0) + elbowPIDOutput);
        }

        double shoulderAngleSetpoint = setpointState.get(0,0);
        double elbowAngleSetpoint = setpointState.get(1,0);            

        // trigger emergency stop if necessary
        internalDisableTimer.start();
        if (status.getInternalDisable()){
            internalDisableTimer.reset();
        } else {
            if ((Math.abs(shoulderAngleRad - shoulderAngleSetpoint) < internalDisableMaxError) &&
                (Math.abs(elbowAngleRad - elbowAngleSetpoint) < internalDisableMaxError)) {
                internalDisableTimer.reset();
            } else if (internalDisableTimer.hasElapsed(internalDisableMaxErrorTime)) {
                // status.setInternalDisable(true, "Outside 10 deg for 0.5 sec");
            }

            // Check if beyond limits
            if (!dynamics.isGoodShoulderAngle(shoulderAngleRad, internalDisableBeyondLimitThreshold)) {
                // status.setInternalDisable(true, "Shoulder beyond limit");
            }

            if (!dynamics.isGoodElbowAngle(elbowAngleRad, internalDisableBeyondLimitThreshold)) {
                    // status.setInternalDisable(true, "Elbow beyond limit");
            }
        }

        // Reset internal emergency stop when override is active
        if (disableSupplier.get()) {
            status.setInternalDisable(false, "");
        }

        status.setShoulderAngleRadSetpoint(shoulderAngleSetpoint)
              .setElbowAngleRadSetpoint(elbowAngleSetpoint)
              .setSetpointTrajState(setpointState)
              .setShoulderFeedforward(voltages.get(0,0))
              .setElbowFeedforward(voltages.get(1,0))
              .setShoulderPIDOutput(shoulderPIDOutput)
              .setElbowPIDOutput(elbowPIDOutput);        
    }

    public void manualAdjustment(double shoulderThrottle, double yThrottle, double elbowThrottle) {
        // shoulderThrottle and elbowThrottle are assumed to be joystick inputs in the range [-1, +1]
        status.setshoulderThrottle(shoulderThrottle)
              .setElbowThrottle(elbowThrottle);
        // TODO: adjust turret angle target
        // yAdjustmentInches += yThrottle * manualMaxSpeedDegreesPerSec * Constants.loopPeriodSecs;
    }    

    public Optional<Translation2d> getClosestNodeXY(ArmStatus.NodeEnum _targetNode, Translation2d turretLoc) {
        // target node selects which node in a 3x3 grid
        // this function finds the closest node out of the 3 possible choices

        int startingRow = _targetNode.xPos;

        double minDist = xMaxSetpoint;
        Translation3d nodeTranslation3d = new Translation3d();   
        Optional<Translation2d> bestTranslation = Optional.empty();

        for (int row = startingRow; row < FieldDimensions.Grids.nodeRowCount; row+=3) {
            switch (_targetNode.yPos) {
                case 0:
                    nodeTranslation3d = FieldDimensions.Grids.complexLow3dTranslations[row];
                    break;
                case 1:
                    nodeTranslation3d = FieldDimensions.Grids.mid3dTranslations[row];
                    break;
                case 2:
                    nodeTranslation3d = FieldDimensions.Grids.high3dTranslations[row];
                    break;
                default:
                    nodeTranslation3d = new Translation3d();
            }

            Translation2d nodeLocation = AllianceFlipUtil.apply(GeomUtil.translation3dTo2dXY(nodeTranslation3d));
            double dist = turretLoc.getDistance(nodeLocation);
            if (dist < minDist) {
                minDist = dist;
                bestTranslation = Optional.of(nodeLocation);
            }
        }

        return bestTranslation;
    }

    private final Timer disabledTimer = new Timer();
    @Override
    protected void Disabled() {
        trajectoryTimer.stop(); // Pause trajectory so it can continue on re-enable (Autonomous to Tele-op)
        disabledTimer.start();
        if(status.EnabledState.IsInitState)
            disabledTimer.reset();
        status.setTurretPower(0.0);
        if(disabledTimer.hasElapsed(kDisabledResetTimerThreshold)) {
            status.setCurrentArmPose(null)
                  .setCurrentArmTrajectory(null)
                  .setClawGrabbing(true)
                  .setCurrentTrajState(ArmTrajectory.getFixedState(ArmPose.Preset.DEFENSE.getShoulderAngleRad(), ArmPose.Preset.DEFENSE.getElbowAngleRad()));
            if(status.getShoulderPotEncStatus().calibrated && Math.abs(status.getShoulderAngleRad() - ArmPose.Preset.DEFENSE.getShoulderAngleRad()) > kProximalZeroErrorThreshold) {
                status.setArmState(ArmState.ZeroDistalUp); // Calibrate Proximal
            } else if(Math.abs(status.getTurretAngleDeg()) > kTurretZeroErrorThreshold) {
                status.setArmState(ArmState.ZeroTurret); // Calibrate Turret
            } else if(status.getElbowPotEncStatus().calibrated && Math.abs(status.getElbowAngleRad() - ArmPose.Preset.DEFENSE.getElbowAngleRad()) > kDistalZeroErrorThreshold) {
                status.setArmState(ArmState.ZeroDistal); // Calibrate Distal
            } else {
                status.setArmState(ArmState.Defense)
                      .setCurrentArmPose(ArmPose.Preset.DEFENSE);
            }
        }
        internalDisableTimer.reset();
    }

    private double getTurretBestAngle(Translation2d target) {
        Translation2d pointRel = new Pose2d(target, new Rotation2d()).relativeTo(odometryStatus.getRobotPose().transformBy(new Transform2d(ArmStatus.robotToTurretTranslation.toTranslation2d(), new Rotation2d()))).getTranslation();
        double raw = Units.radiansToDegrees(Math.atan2(pointRel.getY(), pointRel.getX()));
        double bestAngle = raw;
        double bestError = Math.abs(raw - status.getTurretAngleDeg());
        double test = raw - 360;
        if(Math.abs(test - status.getTurretAngleDeg()) < bestError) {
            bestAngle = test;
            bestError = Math.abs(test - status.getTurretAngleDeg());
        }
        test = raw + 360;
        if(Math.abs(test - status.getTurretAngleDeg()) < bestError) {
            bestAngle = test;
            bestError = Math.abs(test - status.getTurretAngleDeg());
        }
        return bestAngle;
    }
}
