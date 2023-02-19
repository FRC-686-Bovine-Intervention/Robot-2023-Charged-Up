package frc.robot.subsystems.arm;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.arm.json.ArmConfigJson;
import frc.robot.subsystems.arm.json.ArmPathsJson;
import frc.robot.subsystems.arm.json.ArmPresetsJson;
import frc.robot.subsystems.framework.SubsystemBase;



public class Arm extends SubsystemBase {
    private static Arm instance;
    public static Arm getInstance(){if(instance == null) instance = new Arm(); return instance;}

    static final String configFilename = "arm_config.json";
    final String presetFilename = "arm_preset_poses.json";

    // private final String configJson;
    private final ArmPresetsJson presets;
    private final ArmConfigJson config;
    private final ArmKinematics kinematics;
    private final ArmDynamics dynamics;    
    
    private final ArmTrajectory[][] armTrajectories = new ArmTrajectory[ArmPose.Preset.values().length+1][ArmPose.Preset.values().length+1];
    private ArmTrajectory currentTrajectory = null;
    private ArmPose setpointPose = null; // Pose to revert to when not following trajectory
    private ArmPose queuedPose = null; // Use as setpoint once trajectory is completed
    private Timer trajectoryTimer = new Timer();
  
    private PIDController shoulderFeedback = new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
    private PIDController elbowFeedback    = new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);    
    
    private Arm() { 
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
    public void init()
    {
        Loop = ArmLoop.getInstance();
        Status = ArmStatus.getInstance();
    }

    private ArmCommand command = new ArmCommand();
    public ArmCommand getCommand()                  {return command;}
    public Arm setCommand(ArmCommand armCommand)    {this.command = armCommand; return this;}
}
