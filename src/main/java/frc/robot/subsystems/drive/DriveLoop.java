package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.framework.LoopBase;

public class DriveLoop extends LoopBase {
    private static DriveLoop instance;
    public static DriveLoop getInstance(){if(instance == null){instance = new DriveLoop();}return instance;}

    private final Drive drive;
    private final DriveHAL HAL;
    private final DriveStatus status;

    public static final double kDriveWatchdogTimerThreshold = 0.500;
    private static final double kDisabledCoastTimerThreshold = 5;

    private DriveLoop()
    {
        drive = Drive.getInstance();
        HAL = DriveHAL.getInstance();
        status = DriveStatus.getInstance();
        Subsystem = drive;
    }

    @Override
    public void Enabled() {
        DriveCommand driveCommand = drive.getDriveCommand();
        double currentTime = Timer.getFPGATimestamp();

        if(currentTime - driveCommand.getCommandTime() > kDriveWatchdogTimerThreshold)
            driveCommand = DriveCommand.COAST();

        setMotors(driveCommand);
        setNeutralMode(driveCommand);
        setEncoders(driveCommand);

        status.setCommand(driveCommand);
    }

    private final Timer disabledTimer = new Timer();
    @Override
    public void Disabled() {
        disabledTimer.start();
        if(status.EnabledState.IsInitState)
            disabledTimer.reset();

        DriveCommand disabledCommand = disabledTimer.hasElapsed(kDisabledCoastTimerThreshold) ? DriveCommand.COAST() : DriveCommand.BRAKE();

        setMotors(disabledCommand);
        setNeutralMode(disabledCommand);
        setEncoders(disabledCommand);

        status.setCommand(disabledCommand);
    }

    @Override
    public void Update() {
        
    }

    private void setMotors(DriveCommand command)
    {
        HAL.setMotors(command.getTalonMode(), command.getLeftMotor(), command.getRightMotor());
    }

    private void setNeutralMode(DriveCommand command)
    {
        HAL.setNeutralMode(command.getNeutralMode());
    }

    private void setEncoders(DriveCommand command)
    {
        if(command.getResetEncoders())
            HAL.setEncoders();
    }
}
