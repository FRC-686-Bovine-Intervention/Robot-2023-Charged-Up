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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.arm.json.ArmConfigJson;
import frc.robot.subsystems.arm.json.ArmPathsJson;
import frc.robot.subsystems.arm.json.ArmPresetsJson;
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


    private static final double kDistalZeroPower = 0.08;
    private static final double kTurretZeroPower = 0.08;
    private static final double kProximalZeroPower = 0.08;

    private static final double kDistalZeroErrorThreshold = 2.5;
    private static final double kTurretZeroErrorThreshold = 2.5;
    private static final double kProximalZeroErrorThreshold = 2.5;

    private ArmState prevState;

    static final String configFilename = "arm_config.json";
    final String presetFilename = "arm_preset_poses.json";

    // private final String configJson;
    private final ArmPresetsJson presets;
    private final ArmConfigJson config;
    private final ArmKinematics kinematics;
    private final ArmDynamics dynamics;    
    
    private final ArmTrajectory[][] armTrajectories = new ArmTrajectory[ArmPose.Preset.values().length+1][ArmPose.Preset.values().length+1];
    // private ArmTrajectory currentTrajectory = null;
    private final Timer trajectoryTimer = new Timer();

    private Matrix<N2,N3> finalTrajectoryState = null;
    private Matrix<N2,N3> setpointState = null;       
  
    private PIDController shoulderFeedback = new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
    private PIDController elbowFeedback    = new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);    

    private boolean internalDisable = false;
    private Timer internalDisableTimer = new Timer();
    public static final double internalDisableMaxError = Units.degreesToRadians(10.0);    
    public static final double internalDisableMaxErrorTime = 0.5;
    public static final double internalDisableBeyondLimitThreshold = Units.degreesToRadians(5.0);
    
    private Supplier<Boolean> disableSupplier = () -> false;

    private final double xMinSetpoint =  0.0; 
    private final double xMaxSetpoint;          // calculated
    private final double zMinSetpoint =  0.0; 
    private final double zMaxSetpoint = 72.0; 

    private double xThrottle = 0.0;
    private double zThrottle = 0.0;
    private double xAdjustment = 0.0; // extension from turret center of rotation
    private double zAdjustment = 0.0; // height
    private final double xAdjustmentMaxRangeInches = 12.0;
    private final double zAdjustmentMaxRangeInches = 12.0;
    private final double xAdjustmentMaxRange = Units.inchesToMeters(xAdjustmentMaxRangeInches);
    private final double zAdjustmentMaxRange = Units.inchesToMeters(zAdjustmentMaxRangeInches);

    private final double manualMaxSpeedInchesPerSec = 6.0;    // speed the arm is allowed to extend manually in the turret's XZ plane
    private final double manualMaxSpeedMetersPerSec = Units.inchesToMeters(manualMaxSpeedInchesPerSec);
    private final double manualMaxSpeedDegreesPerSec = 10.0;  // speed the turret is allowed to manually spin
    
    public double getStartingShoulderAngleRad(ArmPose.Preset preset) {
        int startIdx = preset.getFileIdx();
        var traj = armTrajectories[startIdx][startIdx];
        var state = traj.getFinalState();
        return state.get(0,0);
    }
    public double getStartingElbowAngleRad(ArmPose.Preset preset) {
        int startIdx = preset.getFileIdx();
        var traj = armTrajectories[startIdx][startIdx];
        var state = traj.getFinalState();
        return state.get(1,0);
    }

    private ArmLoop() {
        Subsystem = Arm.getInstance();
    
         // Get presets from JSON
         File presetFile = new File(Filesystem.getDeployDirectory(), ArmPresetsJson.jsonFilename);
         presets = ArmPresetsJson.loadJson(presetFile);   
         ArmPose.Preset.writePresets(presets);
         
         // Get config from JSON
         File configFile = new File(Filesystem.getDeployDirectory(), ArmConfigJson.jsonFilename);
         config = ArmConfigJson.loadJson(configFile);
 
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
 
         kinematics = new ArmKinematics(new Translation2d(config.origin().getX(), config.origin().getY()),
                                         config.shoulder().length(), config.elbow().length(),
                                         config.shoulder().minAngle(), config.shoulder().maxAngle(), 
                                         config.elbow().minAngle(), config.elbow().maxAngle());
         
         dynamics = new ArmDynamics(config.shoulder(), ArmDynamics.rigidlyCombineJoints(config.elbow(), config.wrist()));

         xMaxSetpoint = Units.inchesToMeters(config.frame_width_inches());
         finalTrajectoryState = armTrajectories[ArmPose.Preset.DEFENSE.getFileIdx()][ArmPose.Preset.DEFENSE.getFileIdx()].getFinalState();
         setpointState = finalTrajectoryState;
    }

    private final Timer stateTimer = new Timer();
    
    @Override
    protected void Enabled() {
        ArmCommand newCommand = status.getCommand();

        if(newCommand.getArmState() != null)
            status.setArmState(newCommand.getArmState());

        stateTimer.start();
        // Get measured positions
        double shoulderAngleRad = Units.degreesToRadians(status.getShoulderStatus().positionDeg);
        double elbowAngleRad = Units.degreesToRadians(status.getElbowStatus().positionDeg);

        // if internally disabled, set the setpoint to the current position (don't move when enabling)
        if (internalDisable) {
            setpointState = ArmTrajectory.getFixedState(shoulderAngleRad, elbowAngleRad);
            status.setCurrentArmTrajectory(null);
        }

        // check if current trajectory is finished
        if (status.getCurrentArmTrajectory() != null && trajectoryTimer.hasElapsed(status.getCurrentArmTrajectory().getTotalTime()))
        {
            status.setCurrentArmTrajectory(null);
            xAdjustment = 0.0;
            zAdjustment = 0.0;
        }

        // move arm!
        Matrix<N2,N3> currentState = setpointState;
        if (status.getCurrentArmTrajectory() != null) {
            // follow trajectory
            trajectoryTimer.start();                            // multiple calls to start will not restart timer
            finalTrajectoryState = status.getCurrentArmTrajectory().getFinalState();
            setpointState = finalTrajectoryState;               // if the trajectory is interrupted, go to the last setpoint

            // interpolate state from trajectory
            currentState = status.getCurrentArmTrajectory().sample(trajectoryTimer.get());

        } else {
            // make manual adjustments to final XZ pose
            Vector<N2> thetaFinalTrajectory = new Vector<>(finalTrajectoryState.extractColumnVector(0));
            Translation2d xzFinalTrajectory = kinematics.forward(thetaFinalTrajectory);
            double xFinalTrajectory = xzFinalTrajectory.getX();
            double zfinalTrajectory = xzFinalTrajectory.getY(); // note: Translation2d assumes XY plane, but we are using it in the XZ plane

            // update manual adjustments
            // xThrottle and zThrottle are assumed to be joystick inputs in the range [-1, +1]
            xAdjustment += xThrottle * manualMaxSpeedMetersPerSec * Constants.loopPeriodSecs;
            zAdjustment += zThrottle * manualMaxSpeedMetersPerSec * Constants.loopPeriodSecs;

            // clamp manual adjustments
            xAdjustment = MathUtil.clamp(xAdjustment, -xAdjustmentMaxRange, +xAdjustmentMaxRange);
            zAdjustment = MathUtil.clamp(zAdjustment, -zAdjustmentMaxRange, +zAdjustmentMaxRange);

            // verify frame perimeter
            double xSetpoint = MathUtil.clamp(xFinalTrajectory + xAdjustment, xMinSetpoint, xMaxSetpoint);
            double zSetpoint = MathUtil.clamp(zfinalTrajectory + zAdjustment, zMinSetpoint, zMaxSetpoint);

            // calcualate current manual adjustment after clamping
            xAdjustment = xSetpoint - xFinalTrajectory;
            zAdjustment = zSetpoint - zfinalTrajectory;

            // find new setpoint
            Optional<Vector<N2>> optTheta = kinematics.inverse(xSetpoint, zSetpoint);
            if (optTheta.isPresent()) {
                Vector<N2> setpointTheta = optTheta.get();
                setpointState = ArmTrajectory.getFixedState(setpointTheta);
                currentState = setpointState;
            }
        }

        // calculate feedforward and feedback voltages
        var voltages = dynamics.feedforward(currentState);
        double shoulderAngleSetpoint = currentState.get(0,0);
        double elbowAngleSetpoint = currentState.get(1,0);            
        double shoulderPidFeedback = shoulderFeedback.calculate(shoulderAngleRad, shoulderAngleSetpoint);
        double elbowPidFeedback = elbowFeedback.calculate(elbowAngleRad, elbowAngleSetpoint) ;
        
        // set motors to achieve the currentState
        status.setShoulderPower((voltages.get(0,0) + shoulderPidFeedback) / 12)
              .setElbowPower((voltages.get(1,0) + elbowPidFeedback) / 12);
        

        // trigger emergency stop if necessary
        internalDisableTimer.start();
        if (internalDisable){
            internalDisableTimer.reset();
        } else {
            if ((Math.abs(shoulderAngleRad - shoulderAngleSetpoint) < internalDisableMaxError) &&
                (Math.abs(elbowAngleRad - elbowAngleSetpoint) < internalDisableMaxError)) {
                internalDisableTimer.reset();
            } else if (internalDisableTimer.hasElapsed(internalDisableMaxErrorTime)) {
                internalDisable = true;
            }

            // Check if beyond limits
            if ((!dynamics.isGoodShoulderAngle(shoulderAngleRad, internalDisableBeyondLimitThreshold)) ||
                (!dynamics.isGoodElbowAngle(elbowAngleRad, internalDisableBeyondLimitThreshold))) {
                internalDisable = true;
            }
        }

        // Reset internal emergency stop when override is active
        if (disableSupplier.get()) {
            internalDisable = false;
        }
        if(prevState != status.getArmState())
            stateTimer.reset();

        prevState = status.getArmState();

        status.setTurretPower(0.0);

        switch(status.getArmState())
        {
            case ZeroDistal:
                if(status.getElbowStatus().calibrated)
                {
                    status.setElbowPower(kDistalZeroPower * Math.signum(getStartingElbowAngleRad(ArmPose.Preset.DEFENSE) - elbowAngleRad));
                    if(Math.abs(elbowAngleRad - getStartingElbowAngleRad(ArmPose.Preset.DEFENSE)) <= kDistalZeroErrorThreshold)
                        status.setArmState(ArmState.ZeroTurret);
                }
                break;
            
            case ZeroTurret:
                status.setElbowPower(kTurretZeroPower * Math.signum(-status.getTurretPosition()));
                if(Math.abs(status.getTurretPosition()) <= kTurretZeroErrorThreshold)
                    status.setArmState(ArmState.ZeroProximal);
            break;
            
            case ZeroProximal:
                if(status.getShoulderStatus().calibrated)
                {
                    status.setShoulderPower(kProximalZeroPower * Math.signum(getStartingShoulderAngleRad(ArmPose.Preset.DEFENSE) - shoulderAngleRad));
                    if(Math.abs(shoulderAngleRad - getStartingShoulderAngleRad(ArmPose.Preset.DEFENSE)) <= kProximalZeroErrorThreshold)
                        status.setArmState(ArmState.Defense);
                }
            break;

            case Defense:
                if(intakeStatus.getIntakeState() == IntakeState.Hold)
                    status.setArmState(ArmState.IdentifyPiece);
            break;

            case IdentifyPiece:
                // Check for piece in intake bounding box
            break;
            
            case Grab:
                // Move turret to target pos
                turretPID.setGoal(status.getTargetTurretAngle());
                if(stateTimer.get() == 0)
                    turretPID.reset(status.getTurretPosition());
                
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

        if(status.getCurrentArmPose() != null && status.getTargetArmPose() != status.getCurrentArmPose())
            startTrajectory(status.getCurrentArmPose(), status.getTargetArmPose());
    }


    @Override
    protected void Disabled() {
        status.setTurretPower(0.0);
        status.setArmState(ArmState.Defense); //TODO: Add disable time threshold
        internalDisableTimer.reset();        
    }


    @Override
    protected void Update() {
    }


    public void startTrajectory(ArmPose.Preset startPos, ArmPose.Preset finalPos) {
        // get pre-planned trajectory
        ArmTrajectory baseTrajectory = armTrajectories[startPos.getFileIdx()][finalPos.getFileIdx()];

        // get current arm positions
        double shoulderAngleRad = Units.degreesToRadians(status.getShoulderStatus().positionDeg);
        double elbowAngleRad = Units.degreesToRadians(status.getElbowStatus().positionDeg);

        // throw error if selected trajectory is no where near the current position
        if (!baseTrajectory.startIsNear(shoulderAngleRad, elbowAngleRad, internalDisableMaxError)) {
            internalDisable = true;
            status.setCurrentArmTrajectory(null);
            return;
        }

        // any errors in the starting position (in particular due to manual XZ adjustments)
        // will be linearly intepolated into the trajectory
        // to smoothly remove these adjustments without needing a separate path
        status.setCurrentArmTrajectory(baseTrajectory);
        status.getCurrentArmTrajectory().interpolateEndPoints(Double.valueOf(shoulderAngleRad), Double.valueOf(elbowAngleRad), null, null);

        trajectoryTimer.reset();
    }

    public void manualAdjustment(double xThrottle, double yThrottle, double zThrottle) {
        // xThrottle and zThrottle are assumed to be joystick inputs in the range [-1, +1]
        this.xThrottle = xThrottle;
        this.zThrottle = zThrottle;

        // TODO: adjust turret angle target
        // yAdjustmentInches += yThrottle * manualMaxSpeedDegreesPerSec * Constants.loopPeriodSecs;
    }
}
