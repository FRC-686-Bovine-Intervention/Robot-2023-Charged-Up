package frc.robot.subsystems.driverAssist;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.framework.StatusBase;

public class DriverAssistStatus extends StatusBase {
    private static DriverAssistStatus instance;
    public static DriverAssistStatus getInstance(){if(instance == null){instance = new DriverAssistStatus();}return instance;}

    public enum DriverAssistState {
        Disabled,
        AutoBalance,
        AutoDrive
    }

    private DriverAssistStatus() {Subsystem = DriverAssist.getInstance();}

    private DriverAssistCommand assistCommand = new DriverAssistCommand();
    public DriverAssistCommand getAssistCommand()                           {return assistCommand;}
    public DriverAssistStatus setAssistCommand(DriverAssistCommand command) {this.assistCommand = command; return this;}

    private DriverAssistState driverAssistState = DriverAssistState.Disabled;
    public DriverAssistState getDriverAssistState()                                     {return driverAssistState;}
    public DriverAssistStatus setDriverAssistState(DriverAssistState driverAssistState) {this.driverAssistState = driverAssistState; return this;}

    private Trajectory trajectory = new Trajectory();
    public Trajectory getTrajectory()                               {return trajectory;}
    public DriverAssistStatus setTrajectory(Trajectory trajectory)  {this.trajectory = trajectory; return this;}

    private DriveCommand driveCommand;
    public DriveCommand getDriveCommand()                           {return driveCommand;}
    public DriverAssistStatus setDriveCommand(DriveCommand command) {this.driveCommand = command; return this;}

    private double pitchVelocity;
    public double getPitchVelo()                                    {return pitchVelocity;}
    public DriverAssistStatus setPitchVelo(double pitchVelocity)    {this.pitchVelocity = pitchVelocity; return this;}

    private double estimatedPitch;
    public double getEstimatedPitch()                                   {return estimatedPitch;}
    public DriverAssistStatus setEstimatedPitch(double estimatedPitch)  {this.estimatedPitch = estimatedPitch; return this;}

    private boolean usingProportional;
    public boolean getUsingProportional()                                   {return usingProportional;}
    public DriverAssistStatus setUsingProportional(boolean usingProportional)  {this.usingProportional = usingProportional; return this;}

    @Override
    protected void updateInputs() {
        setAssistCommand(DriverAssist.getInstance().getCommand());
    }

    @Override
    protected void processOutputs(Logger logger, String prefix) {
        logger.recordOutput(prefix + "Command/Target Pose",     assistCommand.getTargetPose());
        logger.recordOutput(prefix + "Calculated Trajectory",   getTrajectory());
        logger.recordOutput(prefix + "Driver Assist State",     getDriverAssistState().name());
        logger.recordOutput(prefix + "Pitch Velocity",          getPitchVelo());
        logger.recordOutput(prefix + "Estimated Pitch",         getEstimatedPitch());
        logger.recordOutput(prefix + "usingProportional",       getUsingProportional());

        if(getDriveCommand() != null)
            getDriveCommand().logCommand(logger, prefix + "Drive Command/");
    }
    
    @Override protected void exportToTable(LogTable table) {}
    @Override protected void importFromTable(LogTable table) {}
    @Override protected void processTable() {}
}
