package frc.robot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto.actions.Action;
import frc.robot.subsystems.arm.ArmPose;

public abstract class AutoMode {
    public final ArrayList<Action> actionList = new ArrayList<Action>();
    public Pose2d initialPose = new Pose2d();
    public ArmPose.Preset initialArmPose = ArmPose.Preset.AUTO_START;
    
    protected final void addAction(Action action)           {actionList.add(action);}
    protected final void addActions(List<Action> actions)   {actionList.addAll(actions);}
}
