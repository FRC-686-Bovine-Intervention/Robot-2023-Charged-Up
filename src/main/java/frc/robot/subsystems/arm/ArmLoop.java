package frc.robot.subsystems.arm;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.subsystems.arm.json.ArmConfigJson;
import frc.robot.subsystems.arm.json.ArmPathsJson;
import frc.robot.subsystems.arm.json.ArmPresetsJson;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeStatus;
import frc.robot.subsystems.intake.IntakeStatus.IntakeState;
import frc.robot.subsystems.odometry.OdometryStatus;

public class ArmLoop extends LoopBase {
    private static ArmLoop instance;
    public static ArmLoop getInstance(){if(instance == null) instance = new ArmLoop(); return instance;}

    private final Arm arm = Arm.getInstance();
    private final ArmStatus status = ArmStatus.getInstance();
    private final Intake intake = Intake.getInstance();
    private final IntakeStatus intakeStatus = IntakeStatus.getInstance();

    private static final double kTurretMaxAngularVelocity = 135;
    private static final double kTurretMaxAngularAcceleration = 270;
    private final TrapezoidProfile.Constraints turretPIDConstraints = new TrapezoidProfile.Constraints(kTurretMaxAngularVelocity, kTurretMaxAngularAcceleration);
    private final ProfiledPIDController turretPID = 
        new ProfiledPIDController(
            0.08, 
            0, 
            0, 
            turretPIDConstraints
        );
    private static final double kTurretPIDMaxError = 10;
    private static final double kTurretClockwiseLockoutThreshold = 90;
    private static final double kTurretCounterLockoutThreshold = -90;

    private static final double kDistalZeroPower =      0.2;
    private static final double kProximalZeroPower =    0.2;
    private static final double kTurretZeroPower =      0.1;

    private static final double kDistalZeroErrorThreshold = Units.degreesToRadians(2.5);
    private static final double kTurretZeroErrorThreshold = Units.degreesToRadians(2.5);
    private static final double kProximalZeroErrorThreshold = Units.degreesToRadians(2.5);

    private static final double kDistalZeroRadUp = Math.PI/2;

    private ArmState prevState;

    private static final double kDisabledTimerThreshold = 5;

    // private final String configJson;
    private final ArmPresetsJson presets;
    private final ArmKinematics kinematics;
    private final ArmDynamics dynamics;    
    
    private final ArmTrajectory[][] armTrajectories = new ArmTrajectory[ArmPose.Preset.values().length+1][ArmPose.Preset.values().length+1];
    // private ArmTrajectory currentTrajectory = null;
    private final Timer trajectoryTimer = new Timer();

    private Matrix<N2,N3> setpointState = null;     
    private Matrix<N2,N3> prevSetpointState = null;
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



        xMaxSetpoint = Units.inchesToMeters(config.frame_width_inches() + 48.0);
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
                      status.getTurretAngleDeg() >= kTurretClockwiseLockoutThreshold
                                                 &&
                      status.getTurretAngleDeg() <= kTurretCounterLockoutThreshold)
                  .setCheckedForTurretLockout(true);
        }
        checkArmCalibration();

        if(setShoulderEntry.getBoolean(false))
        {
            shoulderPID.setP(kPEntry.getDouble(shoulderPID.getP()));
            shoulderPID.setI(kIEntry.getDouble(shoulderPID.getI()));
            shoulderPID.setD(kDEntry.getDouble(shoulderPID.getD()));
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
            status.setTurretLockout(false);
            resetLockoutEntry.setBoolean(false);
        }
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
                status.setInternalDisable(true)     // disable PID during zeroing
                      .setShoulderPower(0)
                      .setCurrentArmPose(null);
                if(status.getElbowPotEncStatus().calibrated && status.getShoulderPotEncStatus().calibrated)
                {
                    status.setElbowPower(kDistalZeroPower * Math.signum(shoulderAngleRad + kDistalZeroRadUp - elbowAngleRad));
                    if(Math.abs(shoulderAngleRad + kDistalZeroRadUp - elbowAngleRad) <= kDistalZeroErrorThreshold || Math.abs(shoulderAngleRad - ArmPose.Preset.DEFENSE.getShoulderAngleRad()) <= kProximalZeroErrorThreshold)
                        status.setArmState(ArmState.ZeroProximal);
                }
            break;
            
            case ZeroProximal:
                status.setCurrentArmPose(null);
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
                      .setTurretPower(kTurretZeroPower * Math.signum(-status.getTurretAngleDeg()));
                if(Math.abs(status.getTurretAngleDeg()) <= kTurretZeroErrorThreshold)
                    status.setArmState(ArmState.Defense);//TODO TurretDebug
            break;

            case ZeroDistal:
                status.setCurrentArmPose(null);
                if(status.getElbowPotEncStatus().calibrated)
                {
                    status.setElbowPower(kDistalZeroPower * Math.signum(ArmPose.Preset.DEFENSE.getElbowAngleRad() - elbowAngleRad));
                    if(Math.abs(elbowAngleRad - ArmPose.Preset.DEFENSE.getElbowAngleRad()) <= kDistalZeroErrorThreshold)
                    {
                        status.setArmState(ArmState.Defense)
                              .setCurrentArmPose(ArmPose.Preset.DEFENSE)
                              .setTargetArmPose(ArmPose.Preset.DEFENSE)
                              .setInternalDisable(false);     // enable arm trajectories
                    }
                }
            break;

            case Defense:
                status.setTargetTurretAngleDeg(0);
                status.setTargetArmPose(ArmPose.Preset.DEFENSE);
                if(intakeStatus.getIntakeState() == IntakeState.Hold)
                    status.setArmState(ArmState.IdentifyPiece);
            break;

            case IdentifyPiece:
                status.setTargetTurretAngleDeg(0);
                status.setTargetArmPose(ArmPose.Preset.DEFENSE);
                // Check for piece in intake bounding box
            break;
            
            case Grab:
                // Move turret to target pos
                // Extend arm to grab piece
                if(turretPID.atGoal())
                {
                    status.setTargetArmPose(ArmPose.Preset.INTAKE);
                    if(status.getCurrentArmTrajectory() == null)
                    {
                        // Grab piece
                        status.setClawGrabbing(true);
                        // Set intake to release
                        intake.setCommand(new IntakeCommand(IntakeState.Release));
                        // Jump to Hold
                        if(trajectoryTimer.hasElapsed(status.getCurrentArmTrajectory().getTotalTime() + 0.5))
                            status.setArmState(ArmState.Hold);
                    }
                }
            break;

            case Hold:
                intake.setCommand(new IntakeCommand(IntakeState.Defense));
                // Move arm to hold position
                status.setTargetArmPose(ArmPose.Preset.DEFENSE);
                // Check if robot is in community, if so jump to Align
            break;

            case Align:
                status.setTargetArmPose(ArmPose.Preset.DEFENSE);
                
                // unwrap turret angle because odometry wraps it to +/-180
                double turretAngleDeg = OdometryStatus.getInstance().getRobotPose().getRotation().getDegrees();
                if (stateTimer.get() == 0.0) {
                    turretUnwrapper.reset(turretAngleDeg);
                }
                double unwrappedTurretAngleDeg = turretUnwrapper.unwrap(turretAngleDeg);

                // Align turret to alliance wall
                status.setTargetTurretAngleDeg((DriverStation.getAlliance() == Alliance.Red ? 0 : 180) - unwrappedTurretAngleDeg);
                
                // Check if robot is in not in community, if so jump to Hold
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
                    status.setArmState(ArmState.Defense);
            break;
        }

        runTurret();

        if(status.getCurrentArmPose() != null && status.getTargetArmPose() != status.getCurrentArmPose())
            startTrajectory(status.getCurrentArmPose(), status.getTargetArmPose(), status.getTargetNode(), status.getTurretToField());
    }

    private boolean largeTurretError = false;
    private void runTurret() {
        turretPID.setGoal(status.getTargetTurretAngleDeg());
        if(status.getTargetTurretAngleDeg() - status.getTurretAngleDeg() >= kTurretPIDMaxError) {
            if(!largeTurretError) {
                turretPID.reset(status.getTurretAngleDeg());
            }
            largeTurretError = true;
        } else
            largeTurretError = false;
        status.setTurretPower(turretPID.calculate(status.getTurretAngleDeg()));
    }




    public void startTrajectory(ArmPose.Preset startPos, ArmPose.Preset finalPos, ArmStatus.NodeEnum targetNode, Pose3d turretPose3d) {
        Translation2d turretXY = GeomUtil.translation3dTo2dXY(turretPose3d.getTranslation());
        
        // get current arm positions
        double startShoulderAngleRad = status.getShoulderAngleRad();
        double startElbowAngleRad = status.getElbowAngleRad();

        // get pre-planned trajectory
        ArmTrajectory baseTrajectory = armTrajectories[startPos.getFileIdx()][finalPos.getFileIdx()];

        // find closest node
        Translation2d nodeXY = getClosestNodeXY(targetNode, turretXY);

        // calculate turret angle to target
        Translation2d turretToTarget = nodeXY.minus(turretXY);
        double targetAngle = turretToTarget.getAngle().getRadians();
        double robotAngle = OdometryStatus.getInstance().getRobotPose().getRotation().getRadians();
        double turretAngleToTarget = targetAngle - robotAngle;

        // extend the distance of the base trajectory
        double finalShoulderAngleRad = finalPos.getShoulderAngleRad();
        double finalElbowAngleRad = finalPos.getElbowAngleRad();

        // calculate turret distance
        double x = turretToTarget.getNorm();
        double z = Units.inchesToMeters(finalPos.getZ());
        Optional<Vector<N2>> theta = kinematics.inverse(new Translation2d(x,z));
        if (theta.isPresent()) {
            finalShoulderAngleRad = theta.get().get(0,0);
            finalElbowAngleRad = theta.get().get(1,0);
        }

        // set outputs
        status.setTargetTurretAngleDeg(turretAngleToTarget);
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
        
        prevSetpointState = setpointState;
        double shoulderAngleSetpoint = prevSetpointState.get(0,0);
        double elbowAngleSetpoint = prevSetpointState.get(1,0);            

        Vector<N2> voltages = VecBuilder.fill(0,0);
        double shoulderPIDOutput = 0;
        double elbowPIDOutput = 0;


        // if internally disabled, set the setpoint to the current position (don't move when enabling)
        if (status.getInternalDisable()) {
            setpointState = ArmTrajectory.getFixedState(shoulderAngleRad, elbowAngleRad);
            prevSetpointState = setpointState;
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
            shoulderPIDOutput = shoulderPID.calculate(shoulderAngleRad, shoulderAngleSetpoint);
            elbowPIDOutput = elbowPID.calculate(elbowAngleRad, elbowAngleSetpoint) ;
            
            // set motors to achieve the setpoint
            status.setShoulderVoltage(voltages.get(0,0) + shoulderPIDOutput)
                  .setElbowVoltage   (voltages.get(1,0) + elbowPIDOutput);
        }

        // trigger emergency stop if necessary
        internalDisableTimer.start();
        if (status.getInternalDisable()){
            internalDisableTimer.reset();
        } else {
            if ((Math.abs(shoulderAngleRad - shoulderAngleSetpoint) < internalDisableMaxError) &&
                (Math.abs(elbowAngleRad - elbowAngleSetpoint) < internalDisableMaxError)) {
                internalDisableTimer.reset();
            } else if (internalDisableTimer.hasElapsed(internalDisableMaxErrorTime)) {
                status.setInternalDisable(true);
            }

            // Check if beyond limits
            if ((!dynamics.isGoodShoulderAngle(shoulderAngleRad, internalDisableBeyondLimitThreshold)) ||
                (!dynamics.isGoodElbowAngle(elbowAngleRad, internalDisableBeyondLimitThreshold))) {
                    status.setInternalDisable(true);
            }
        }

        // Reset internal emergency stop when override is active
        if (disableSupplier.get()) {
            status.setInternalDisable(false);
        }

        status.setShoulderAngleRadSetpoint(shoulderAngleSetpoint)
              .setElbowAngleRadSetpoint(elbowAngleSetpoint)
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



    

    public Translation2d getClosestNodeXY(ArmStatus.NodeEnum _targetNode, Translation2d turretLoc) {
        // target node selects which node in a 3x3 grid
        // this function finds the closest node out of the 3 possible choices

        int startingRow = _targetNode.xPos;

        double minDist = Double.MAX_VALUE;
        Translation3d nodeTranslation3d = new Translation3d();   
        Translation2d bestTranslation = null;

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

            Translation2d nodeLocation = GeomUtil.translation3dTo2dXY(nodeTranslation3d);
            double dist = turretLoc.getDistance(nodeLocation);
            if (dist < minDist) {
                minDist = dist;
                bestTranslation = nodeLocation;
            }
        }

        return bestTranslation;
    }



    private final Timer disabledTimer = new Timer();

    @Override
    protected void Disabled() {
        disabledTimer.start();
        if(status.EnabledState.IsInitState)
            disabledTimer.reset();
        status.setTurretPower(0.0);
        if(disabledTimer.hasElapsed(kDisabledTimerThreshold))
        {
            status.setArmState(ArmState.DEFAULT); //TODO: Add disable time threshold
        }
        internalDisableTimer.reset();
    }
}
