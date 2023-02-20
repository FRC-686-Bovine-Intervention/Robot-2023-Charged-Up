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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.arm.json.ArmConfigJson;
import frc.robot.subsystems.arm.json.ArmPathsJson;
import frc.robot.subsystems.arm.json.ArmPresetsJson;
import frc.robot.subsystems.framework.LoopBase;

public class ArmLoop extends LoopBase {
    private static ArmLoop instance;
    public static ArmLoop getInstance(){if(instance == null) instance = new ArmLoop(); return instance;}

    private final Arm arm = Arm.getInstance();
    private final ArmHAL hal = ArmHAL.getInstance();
    private final ArmStatus status = ArmStatus.getInstance();

    static final String configFilename = "arm_config.json";
    final String presetFilename = "arm_preset_poses.json";

    // private final String configJson;
    private final ArmPresetsJson presets;
    private final ArmConfigJson config;
    private final ArmKinematics kinematics;
    private final ArmDynamics dynamics;    
    
    private final ArmTrajectory[][] armTrajectories = new ArmTrajectory[ArmPose.Preset.values().length+1][ArmPose.Preset.values().length+1];
    private ArmTrajectory currentTrajectory = null;
    private Timer trajectoryTimer = new Timer();

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
 
                 if (startPos != finalPos) {
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
         }
 
         kinematics = new ArmKinematics(new Translation2d(config.shoulder().getX(), config.shoulder().getY()),
                                         config.proximal().length(), config.distal().length(),
                                         config.proximal().minAngle(), config.proximal().maxAngle(), 
                                         config.distal().minAngle(), config.distal().maxAngle());
         
         dynamics = new ArmDynamics(config.proximal(), ArmDynamics.rigidlyCombineJoints(config.distal(), config.grabber()));

         xMaxSetpoint = Units.inchesToMeters(config.frame_width_inches());
         finalTrajectoryState = armTrajectories[ArmPose.Preset.DEFENSE.getFileIdx()][ArmPose.Preset.DEFENSE.getFileIdx()].getFinalState();
         setpointState = finalTrajectoryState;
    } 

    @Override
    protected void Enabled() {
        // Get measured positions
        double shoulderAngleRad = Units.degreesToRadians(status.getShoulderState().positionDeg);
        double elbowAngleRad = Units.degreesToRadians(status.getElbowState().positionDeg);

        // if internally disabled, set the setpoint to the current position (don't move when enabling)
        if (internalDisable) {
            setpointState = ArmTrajectory.getFixedState(shoulderAngleRad, elbowAngleRad);
            currentTrajectory = null;
        }

        // check if current trajectory is finished
        if (currentTrajectory != null && trajectoryTimer.hasElapsed(currentTrajectory.getTotalTime()))
        {
            trajectoryTimer.stop();
            trajectoryTimer.reset();
            currentTrajectory = null;
            xAdjustment = 0.0;
            zAdjustment = 0.0;
        }

        // move arm!
        Matrix<N2,N3> currentState = setpointState;
        if (currentTrajectory != null) {
            // follow trajectory
            trajectoryTimer.start();                            // multiple calls to start will not restart timer
            finalTrajectoryState = currentTrajectory.getFinalState();
            setpointState = finalTrajectoryState;               // if the trajectory is interrupted, go to the last setpoint

            // interpolate state from trajectory
            currentState = currentTrajectory.sample(trajectoryTimer.get());

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
        hal.setShoulderMotorVoltage(voltages.get(0,0) + shoulderPidFeedback);
        hal.setElbowMotorVoltage(voltages.get(1,0) + elbowPidFeedback);
        

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
            if ((!dynamics.isGoodProximalAngle(shoulderAngleRad, internalDisableBeyondLimitThreshold)) ||
                (!dynamics.isGoodDistalAngle(elbowAngleRad, internalDisableBeyondLimitThreshold))) {
                internalDisable = true;
            }
        }

        // Reset internal emergency stop when override is active
        if (disableSupplier.get()) {
            internalDisable = false;
        }
    }


    @Override
    protected void Disabled() {
        internalDisableTimer.reset();        
    }


    @Override
    protected void Update() {
    }


    public void startTrajectory(ArmPose.Preset startPos, ArmPose.Preset finalPos) {
        // get pre-planned trajectory
        ArmTrajectory baseTrajectory = armTrajectories[startPos.getFileIdx()][finalPos.getFileIdx()];

        // get current arm positions
        double shoulderAngleRad = Units.degreesToRadians(status.getShoulderState().positionDeg);
        double elbowAngleRad = Units.degreesToRadians(status.getElbowState().positionDeg);

        // throw error if selected trajectory is no where near the current position
        if (!baseTrajectory.startIsNear(shoulderAngleRad, elbowAngleRad, internalDisableMaxError)) {
            internalDisable = true;
            currentTrajectory = null;
            return;
        }

        // any errors in the starting position (in particular due to manual XZ adjustments)
        // will be linearly intepolated into the trajectory
        // to smoothly remove these adjustments without needing a separate path
        currentTrajectory = baseTrajectory;
        currentTrajectory.interpolateStaringPositionError(shoulderAngleRad, elbowAngleRad);
    }

    public void manualAdjustment(double xThrottle, double yThrottle, double zThrottle) {
        // xThrottle and zThrottle are assumed to be joystick inputs in the range [-1, +1]
        this.xThrottle = xThrottle;
        this.zThrottle = zThrottle;

        // TODO: adjust turret angle target
        // yAdjustmentInches += yThrottle * manualMaxSpeedDegreesPerSec * Constants.loopPeriodSecs;
    }
    
}
