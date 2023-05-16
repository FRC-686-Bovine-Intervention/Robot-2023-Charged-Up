package frc.robot.auto.actions;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveStatus;

public class DriveOnChargeStationEdgeAction extends Action {
    private static final double kDrivePercentOutput = 0.3;
    private final boolean reversed;

    private final Drive drive = Drive.getInstance();
    private final DriveStatus driveStatus = DriveStatus.getInstance();

    public DriveOnChargeStationEdgeAction() {this(false);}
    public DriveOnChargeStationEdgeAction(boolean reversed) {
        this.reversed = reversed;
    }

    private double prevPitch;
    private double prevTimestamp;

    @Override
    protected void start() {
        prevPitch = driveStatus.getPitchDeg();
        prevTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    protected void run() {
        drive.setDriveCommand(new DriveCommand(kDrivePercentOutput * (reversed ? -1 : 1), kDrivePercentOutput * (reversed ? -1 : 1)));
        if(getPitchVelo() * (reversed ? -1 : 1) <= -1) {
            setFinished(true);
        }
        Logger.getInstance().recordOutput("DEBUG/pitch velo", getPitchVelo());
        prevPitch = driveStatus.getPitchDeg();
        prevTimestamp = Timer.getFPGATimestamp();
    }

    private double getPitchVelo() {
        return (driveStatus.getPitchDeg() - prevPitch) / (Timer.getFPGATimestamp() - prevTimestamp);
    }

    @Override
    protected void done() {
        drive.setDriveCommand(DriveCommand.COAST());
    }
}
