package frc.robot.subsystems.driverAssist;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.subsystems.framework.StatusBase;

public class DriverAssistStatus extends StatusBase {
    private static DriverAssistStatus instance;
    public static DriverAssistStatus getInstance(){if(instance == null){instance = new DriverAssistStatus();}return instance;}

    private DriverAssistStatus() {Subsystem = DriverAssist.getInstance();}

    private DriverAssistCommand command = new DriverAssistCommand();
    public DriverAssistCommand getCommand() {return command;}
    public DriverAssistStatus setCommand(DriverAssistCommand command) {this.command = command; return this;}

    private Trajectory trajectory = new Trajectory();
    public Trajectory getTrajectory() {return trajectory;}
    public DriverAssistStatus setTrajectory(Trajectory trajectory) {this.trajectory = trajectory; return this;}

    @Override public void updateInputs() {
        setCommand(DriverAssist.getInstance().getCommand());
    }

    @Override
    public void recordOutputs(Logger logger, String prefix) {
        logger.recordOutput(prefix + "Command/Target Pose", command.getTargetPose());
        logger.recordOutput(prefix + "Calculated Trajectory", getTrajectory());
    }
    
    @Override public void exportToTable(LogTable table) {}
    @Override public void importFromTable(LogTable table) {}
}
