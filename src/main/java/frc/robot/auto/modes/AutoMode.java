package frc.robot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import frc.robot.RobotConfiguration;
import frc.robot.auto.actions.Action;

public abstract class AutoMode {
    public final ArrayList<Action> actionList = new ArrayList<Action>();
    public RobotConfiguration startConfiguration = new RobotConfiguration();
    
    protected final void addAction(Action action)           {actionList.add(action);}
    protected final void addActions(List<Action> actions)   {actionList.addAll(actions);}
}
