package frc.robot.auto.actions;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.auto.autoManager.AutoConfiguration.GamePiece;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveStatus;
import frc.robot.subsystems.driverAssist.DriverAssist;
import frc.robot.subsystems.driverAssist.DriverAssistCommand;
import frc.robot.subsystems.driverAssist.DriverAssistStatus.DriverAssistState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeStatus;
import frc.robot.subsystems.intake.IntakeStatus.IntakeState;

public class AutoPickupPieceAction extends Action {
    private final Drive drive = Drive.getInstance();
    private final DriveStatus driveStatus = DriveStatus.getInstance();
    private final DriverAssist driverAssist = DriverAssist.getInstance();
    private final Intake intake = Intake.getInstance();
    private final IntakeStatus intakeStatus = IntakeStatus.getInstance();

    private final GamePiece piece;
    public final double distanceCap;

    public AutoPickupPieceAction(GamePiece piece, double distanceCap) {
        this.piece = piece;
        this.distanceCap = distanceCap;
    }

    private double startDistance;

    @Override
    public void start() {
        startDistance = getDistance();
        intake.setCommand(new IntakeCommand(IntakeState.Grab));
    }

    @Override
    public void run() {
        double driveSetpoint = (intakeStatus.getIntakeCurrent() < 15 ? 0.4 : 0);
        drive.setDriveCommand(new DriveCommand(driveSetpoint, driveSetpoint).setNeutralMode(NeutralMode.Brake));
        driverAssist.setCommand(new DriverAssistCommand(DriverAssistState.AutoIntake).setTargetGamePiece(piece));
        setFinished(intakeStatus.getIntakeState() == IntakeState.Hold || Math.abs(getDistance() - startDistance) > distanceCap);
    }

    @Override
    public void done() {
        drive.setDriveCommand(DriveCommand.BRAKE());
        driverAssist.setCommand(new DriverAssistCommand(DriverAssistState.Disabled));
        if(intakeStatus.getIntakeState() == IntakeState.Grab) {
            intake.setCommand(new IntakeCommand(IntakeState.Defense));
        }
    }

    private double getDistance() {
        return (driveStatus.getLeftDistanceInches() + driveStatus.getRightDistanceInches()) / 2;
    }
}
