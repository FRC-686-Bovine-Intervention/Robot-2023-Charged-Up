package frc.robot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto.actions.Action;

public abstract class AutoMode {
    public final ArrayList<Action> actionList = new ArrayList<Action>();
    public Pose2d initialPose = new Pose2d();
    
    protected final void addAction(Action action)           {actionList.add(action);}
    protected final void addActions(List<Action> actions)   {actionList.addAll(actions);}
}
