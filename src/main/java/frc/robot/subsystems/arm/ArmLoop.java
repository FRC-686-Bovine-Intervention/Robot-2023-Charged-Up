package frc.robot.subsystems.arm;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
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
    private Matrix<N2,N3> setpointState = null;         // angles to revert to when not following trajectory
  
    private PIDController shoulderFeedback = new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
    private PIDController elbowFeedback    = new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);    

    private boolean internalDisable = false;
    private Timer internalDisableTimer = new Timer();
    public static final double internalDisableMaxError = Units.degreesToRadians(10.0);    
    public static final double internalDisableMaxErrorTime = 0.5;
    public static final double internalDisableBeyondLimitThreshold = Units.degreesToRadians(5.0);
    
    private Supplier<Boolean> disableSupplier = () -> false;

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
    } 

    @Override
    protected void Enabled() {
        // Get measured positions
        double shoulderAngleRad = Units.degreesToRadians(status.getShoulderState().positionDeg);
        double elbowAngleRad = Units.degreesToRadians(status.getElbowState().positionDeg);

        // record current setpoint.  revert to defense on estop
        setpointState = armTrajectories[ArmPose.Preset.DEFENSE.getFileIdx()][ArmPose.Preset.DEFENSE.getFileIdx()].getFinalState();
        double shoulderAngleSetpoint = setpointState.get(0,0); 
        double elbowAngleSetpoint = setpointState.get(1,0);

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
        }

        // move arm!
        Matrix<N2,N3> currentState;
        if (currentTrajectory != null) {
            // follow trajectory
            trajectoryTimer.start();                            // multiple calls to start will not restart timer
            setpointState = currentTrajectory.getFinalState();  // if the trajectory is interrupted, go to the last setpoint

            // interpolate state from trajectory
            currentState = currentTrajectory.sample(trajectoryTimer.get());
        } else {
            // maintain setpoint
            currentState = setpointState;
        }
        var voltages = dynamics.feedforward(currentState);
        shoulderAngleSetpoint = currentState.get(0,0);
        elbowAngleSetpoint = currentState.get(1,0);            
        double shoulderPidFeedback = shoulderFeedback.calculate(shoulderAngleRad, shoulderAngleSetpoint);
        double elbowPidFeedback = elbowFeedback.calculate(elbowAngleRad, elbowAngleSetpoint) ;
        
        hal.setShoulderMotorVoltage(voltages.get(0,0) + shoulderPidFeedback);
        hal.setElbowMotorVoltage(voltages.get(1,0) + elbowPidFeedback);
        

        // trigger emergency stop if necessary
        internalDisableTimer.start();
        if (internalDisable){
            internalDisableTimer.reset();
        } else {
            if ((Math.abs(shoulderAngleRad - shoulderAngleSetpoint) < internalDisableMaxError) &&
                Math.abs(elbowAngleRad - elbowAngleSetpoint) < internalDisableMaxError) {
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
}
