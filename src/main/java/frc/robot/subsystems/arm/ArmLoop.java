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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.FieldDimensions;
import frc.robot.lib.util.GeomUtil;
import frc.robot.lib.util.Unwrapper;
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
import frc.robot.util.AllianceFlipUtil;

public class ArmLoop extends LoopBase {
    private static ArmLoop instance;
    public static ArmLoop getInstance(){if(instance == null) instance = new ArmLoop(); return instance;}

    private final Arm arm = Arm.getInstance();
    private final ArmStatus status = ArmStatus.getInstance();
    private final Intake intake = Intake.getInstance();
    private final IntakeStatus intakeStatus = IntakeStatus.getInstance();
    private final OdometryStatus odometryStatus = OdometryStatus.getInstance();

    private static final double kTurretMaxAngularVelocity = 180;
    private static final double kTurretMaxAngularAcceleration = kTurretMaxAngularVelocity * 2;
    private final TrapezoidProfile.Constraints turretPIDConstraints = new TrapezoidProfile.Constraints(kTurretMaxAngularVelocity, kTurretMaxAngularAcceleration);
    private final ProfiledPIDController turretPID = 
        new ProfiledPIDController(
            0.015, 
            0, 
            0, 
            turretPIDConstraints
        );
    private static final double kTurretPIDMaxError = 10;
    private static final double kTurretClockwiseLockoutThreshold = -90;
    private static final double kTurretCounterLockoutThreshold = 90;

    private static final double kDistalZeroPower =      0.15;
    private static final double kProximalZeroPower =    0.1;
    private static final double kTurretZeroPower =      0.2;

    private static final double kDistalZeroErrorThreshold = Units.degreesToRadians(2.5);
    private static final double kTurretZeroErrorThreshold = 2.5;
    private static final double kProximalZeroErrorThreshold = Units.degreesToRadians(2.5);

    private static final double kDistalZeroRadUp = Units.degreesToRadians(125);

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
            10.0, 
            0.0, 
            0.0, 
            Constants.loopPeriodSecs
        );
    private final PIDController elbowPID = 
        new PIDController(
            10.0, 
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
    public static final double kRelativeMaxAngleRad = Math.toRadians(160.0);    // don't let grabber smash into proximal arm
    public static final double kRelativeMinAngleRad = Math.toRadians(-135.0);   // we'll probably never need this one

    private static final double kMaxElbowPlusClawLength = Units.inchesToMeters(26.0); 

    private final double xMinSetpoint = Units.inchesToMeters(0.0);
    private final double xMaxSetpoint;  // calculated
    private final double zMinSetpoint = Units.inchesToMeters(0.0);
    private final double zMaxSetpoint = Units.inchesToMeters(72.0); 

    private final double manualMaxAdjustmentRangeInches = 12.0;
    private final double manualMaxAdjustmentRange = Units.inchesToMeters(manualMaxAdjustmentRangeInches);

    private final double manualMaxSpeedInchesPerSec = 6.0;    // speed the arm is allowed to extend manually in the turret's XZ plane
    private final double manualMaxSpeedMetersPerSec = Units.inchesToMeters(manualMaxSpeedInchesPerSec);
    private final double manualMaxSpeedDegreesPerSec = 10.0;  // speed the turret is allowed to manually spin


    private ArmLoop() {
        Subsystem = arm;

        // Get config from JSON
        ArmConfigJson config = arm.getConfig();

        JointConfig shoulder = config.shoulder();
        JointConfig elbow = ArmDynamics.rigidlyCombineJoints(config.elbow(), config.wrist());

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
        for (ArmPose.Preset startPos : ArmPose.Preset.values()) {
            for (ArmPose.Preset finalPos : ArmPose.Preset.values()) {
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

                armTrajectories[startIdx][finalIdx] = new ArmTrajectory(path.startPos(), path.finalPos(), path.totalTime(), points);
            }
        }


        double clawLengthPastToolCenterPoint = kMaxElbowPlusClawLength - config.elbow().length() - config.wrist().length();
        xMaxSetpoint = Units.inchesToMeters(config.frame_width_inches() + 48.0 - clawLengthPastToolCenterPoint);
        finalTrajectoryState = armTrajectories[ArmPose.Preset.DEFENSE.getFileIdx()][ArmPose.Preset.DEFENSE.getFileIdx()].getFinalState();
        setpointState = finalTrajectoryState;
    }


    private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    private final GenericEntry kPEntry = tab.add("kP",0).withWidget(BuiltInWidgets.kTextView).getEntry();
    private final GenericEntry kIEntry = tab.add("kI",0).withWidget(BuiltInWidgets.kTextView).getEntry();
    private final GenericEntry kDEntry = tab.add("kD",0).withWidget(BuiltInWidgets.kTextView).getEntry();
    private final GenericEntry setShoulderEntry =   tab.add("Set Shoulder",false)   .withWidget(BuiltInWidgets.kToggleButton).getEntry();
    private final GenericEntry setElbowEntry =      tab.add("Set Elbow",false)      .withWidget(BuiltInWidgets.kToggleButton).getEntry();
    private final GenericEntry resetLockoutEntry =  tab.add("Reset Turret Lockout",false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    @Override
    protected void Update() {
        if(!status.getCheckedForTurretLockout()) {
            status.setTurretLockout(
                      status.getTurretAngleDeg() <= kTurretClockwiseLockoutThreshold
                                                 ||
                      status.getTurretAngleDeg() >= kTurretCounterLockoutThreshold)
                  .setCheckedForTurretLockout(true);
        }
        checkArmCalibration();

        if(setShoulderEntry.getBoolean(false))
        {
            turretPID.setP(kPEntry.getDouble(turretPID.getP()));
            turretPID.setI(kIEntry.getDouble(turretPID.getI()));
            turretPID.setD(kDEntry.getDouble(turretPID.getD()));
            setShoulderEntry.setBoolean(false);
        }
        if(setElbowEntry.getBoolean(false))
        {
            elbowPID.setP(kPEntry.getDouble(elbowPID.getP()));
            elbowPID.setI(kIEntry.getDouble(elbowPID.getI()));
            elbowPID.setD(kDEntry.getDouble(elbowPID.getD()));
            setElbowEntry.setBoolean(false);
        }
        if(resetLockoutEntry.getBoolean(false))
        {
            status.setCheckedForTurretLockout(false);
            resetLockoutEntry.setBoolean(false);
        }

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
    private Unwrapper turretUnwrapper = new Unwrapper(0.0, 360.0);
    
    @Override
    protected void Enabled() {
        // ================= Pulling Data from Command =================
        ArmCommand newCommand = status.getCommand();

        if(newCommand.getArmState() != null)
            status.setArmState(newCommand.getArmState());
        if(newCommand.getTargetNode() != null)
            status.setTargetNode(newCommand.getTargetNode());
        if(newCommand.getXAdjustment() != null)
            status.setXThrottle(newCommand.getXAdjustment());
        if(newCommand.getZAdjustment() != null)
            status.setZThrottle(newCommand.getZAdjustment());

        stateTimer.start();

        status.setTurretPower(0.0)
              .setTurretNeutralMode(NeutralMode.Brake)
              .setShoulderPower(0)
              .setElbowPower(0);

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
                        status.setArmState(ArmState.Defense)
                              .setCurrentArmPose(ArmPose.Preset.DEFENSE)
                              .setTargetArmPose(ArmPose.Preset.DEFENSE)
                              .setInternalDisable(false, "");     // enable arm trajectories
                    }
                }
            break;

            case Defense:
                status.setTurretControlMode(MotorControlMode.PID)
                      .setTargetTurretAngleDeg(0)
                      .setTargetArmPose(ArmPose.Preset.DEFENSE)
                      .setClawGrabbing(false);
                if(intakeStatus.getIntakeState() == IntakeState.Hold)
                    status.setArmState(ArmState.Grab);
            break;

            case IdentifyPiece:
                status.setTurretControlMode(MotorControlMode.PID)
                      .setTargetTurretAngleDeg(0)
                      .setTargetArmPose(ArmPose.Preset.DEFENSE);
                // Check for piece in intake bounding box
            break;
            
            case Grab:
                status.setTurretControlMode(MotorControlMode.PercentOutput)
                      .setTurretPower(0)
                      .setTurretNeutralMode(NeutralMode.Coast);
                if(!status.getClawGrabbing()) {
                    status.setTargetArmPose(ArmPose.Preset.INTAKE);
                    if(status.getCurrentArmPose() == ArmPose.Preset.INTAKE)
                    {
                        // Grab piece
                        status.setClawGrabbing(true);
                        // Set intake to release
                        intake.setCommand(new IntakeCommand(IntakeState.Release));
                        status.setTargetArmPose(ArmPose.Preset.DEFENSE);
                    }
                } else {
                    status.setTargetArmPose(ArmPose.Preset.DEFENSE);
                    if(status.getCurrentArmPose() == ArmPose.Preset.DEFENSE)
                        status.setArmState(ArmState.Hold);
                }
            break;

            case Hold:
                intake.setCommand(new IntakeCommand(IntakeState.Defense));
                // Move arm to hold position
                status.setTargetArmPose(ArmPose.Preset.DEFENSE)
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
                //     status.setArmState(ArmState.Align);
            break;

            case Align:
                status.setTargetArmPose(ArmPose.Preset.DEFENSE);
                
                // unwrap turret angle because odometry wraps it to +/-180
                
                double turretAngleDeg = getTurretBestAngle(
                    new Translation2d(
                        (DriverStation.getAlliance() == Alliance.Red ? FieldDimensions.fieldLength : 0), 
                        odometryStatus.getRobotPose().getY()
                    )
                );

                // Align turret to alliance wall
                status.setTargetTurretAngleDeg(turretAngleDeg)
                      .setTurretControlMode(MotorControlMode.PID);
                
                // Check if robot is in not in community, if so jump to Hold
                if(stateTimer.get() == 0)
                    status.setStateLocked(false);

                if(newCommand.getArmState() == ArmState.Align)
                    status.setStateLocked(true);

                // if(!status.getStateLocked() && !FieldDimensions.communityWithoutChargeStation.withinBounds(odometryStatus.getRobotPose()))
                //     status.setArmState(ArmState.Hold);
                // Check if driver has selected node, if so jump to Extend
            break;

            case Extend:
                // Extend to selected node
                status.setTargetArmPose(status.getTargetNode().armPreset);
                // When completed, jump to Adjust
                if(status.getCurrentArmPose() == status.getTargetArmPose())
                    status.setArmState(ArmState.Adjust);
            break;

            case Adjust:
                // Rotate turret according to limelight and driver controls
                // Check if driver has pushed release button, if so jump to Release
            break;

            case Release:
                // Outtake piece
                status.setClawGrabbing(false);
                // Wait a bit then jump to Defense
                if(stateTimer.hasElapsed(1))
                    status.setArmState(ArmState.Defense);
            break;

            case SubstationExtend:
                status.setTargetArmPose(ArmPose.Preset.DOUBLE_SUBSTATION);
            break;

            case SubstationGrab:
                status.setClawGrabbing(true);
                if(stateTimer.hasElapsed(1))
                    status.setArmState(ArmState.Hold);
            break;
        }

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
                double targetAngle = turretToTarget.getAngle().getRadians();
                double robotAngle = OdometryStatus.getInstance().getRobotPose().getRotation().getRadians();
                turretAngleToTarget = getTurretBestAngle(nodeXY);

                // extend the distance of the base trajectory
                double x = turretToTarget.getNorm();
                double z = Units.inchesToMeters(finalPos.getZ());
                Optional<Vector<N2>> theta = kinematics.inverse(new Translation2d(x,z));
                if (theta.isPresent()) {
                    finalShoulderAngleRad = theta.get().get(0,0);
                    finalElbowAngleRad = theta.get().get(1,0);
                }
            }
            // set outputs
            status.setTargetTurretAngleDeg(turretAngleToTarget);
        }
        status.setCurrentArmTrajectory(baseTrajectory.interpolateEndPoints(startShoulderAngleRad, startElbowAngleRad, finalShoulderAngleRad, finalElbowAngleRad));
        trajectoryTimer.reset();
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
                  .setXAdjustment(0)
                  .setZAdjustment(0);
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
                // make manual adjustments to final XZ pose
                Vector<N2> thetaFinalTrajectory = new Vector<>(finalTrajectoryState.extractColumnVector(0));
                Translation2d xzFinalTrajectory = kinematics.forward(thetaFinalTrajectory);
                double xFinalTrajectory = xzFinalTrajectory.getX();
                double zfinalTrajectory = xzFinalTrajectory.getY(); // note: Translation2d assumes XY plane, but we are using it in the XZ plane

                // update manual adjustments
                // xThrottle and zThrottle are assumed to be joystick inputs in the range [-1, +1]
                status.incrementXAdjustment(status.getXThrottle() * manualMaxSpeedMetersPerSec * Constants.loopPeriodSecs)
                      .incrementZAdjustment(status.getZThrottle() * manualMaxSpeedMetersPerSec * Constants.loopPeriodSecs);

                // clamp manual adjustments
                status.setXAdjustment(MathUtil.clamp(status.getXAdjustment(), -manualMaxAdjustmentRange, +manualMaxAdjustmentRange))
                      .setZAdjustment(MathUtil.clamp(status.getZAdjustment(), -manualMaxAdjustmentRange, +manualMaxAdjustmentRange));

                // verify frame perimeter
                double xSetpoint = MathUtil.clamp(xFinalTrajectory + status.getXAdjustment(), xMinSetpoint, xMaxSetpoint);
                double zSetpoint = MathUtil.clamp(zfinalTrajectory + status.getZAdjustment(), zMinSetpoint, zMaxSetpoint);

                // calcualate current manual adjustment after clamping
                status.setXAdjustment(xSetpoint - xFinalTrajectory)
                      .setZAdjustment(zSetpoint - zfinalTrajectory);

                // find new setpoint
                Optional<Vector<N2>> optTheta = kinematics.inverse(xSetpoint, zSetpoint);
                if (optTheta.isPresent()) {
                    Vector<N2> setpointTheta = optTheta.get();
                    setpointState = ArmTrajectory.getFixedState(setpointTheta);
                }
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
                status.setInternalDisable(true, "Shoulder beyond limit");
            }

            if (!dynamics.isGoodElbowAngle(elbowAngleRad, internalDisableBeyondLimitThreshold)) {
                    status.setInternalDisable(true, "Elbow beyond limit");
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

    public void manualAdjustment(double xThrottle, double yThrottle, double zThrottle) {
        // xThrottle and zThrottle are assumed to be joystick inputs in the range [-1, +1]
        status.setXThrottle(xThrottle)
              .setZThrottle(zThrottle);
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
                  .setClawGrabbing(true);
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
