package frc.robot.subsystems.drive;

import frc.robot.subsystems.framework.SubsystemBase;

public class Drive extends SubsystemBase {
    private static Drive instance;
    public static Drive getInstance(){if(instance == null){instance = new Drive();}return instance;}

    @Override
    public void init() {
        Loop = DriveLoop.getInstance();
        Status = DriveStatus.getInstance();
    }

    private DriveCommand command = DriveCommand.COAST();
    public DriveCommand getDriveCommand() {return command;}
    public Drive setDriveCommand(DriveCommand driveCommand) {this.command = driveCommand; return this;}
}
