package frc.robot.auto.actions;

import frc.robot.auto.autoManager.AutoConfiguration.GamePiece;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.driverAssist.DriverAssist;
import frc.robot.subsystems.driverAssist.DriverAssistCommand;
import frc.robot.subsystems.driverAssist.DriverAssistStatus.DriverAssistState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeStatus;
import frc.robot.subsystems.intake.IntakeStatus.IntakeState;

public class AutoPickupPieceAction extends Action {
    private final Drive drive = Drive.getInstance();
    private final DriverAssist driverAssist = DriverAssist.getInstance();
    private final Intake intake = Intake.getInstance();
    private final IntakeStatus intakeStatus = IntakeStatus.getInstance();

    private final GamePiece piece;

    public AutoPickupPieceAction(GamePiece piece) {
        this.piece = piece;
    }

    @Override
    public void start() {
        intake.setCommand(new IntakeCommand(IntakeState.Grab));
    }

    @Override
    public void run() {
        drive.setDriveCommand(new DriveCommand(0.1,0.1));
        // driverAssist.setCommand(new DriverAssistCommand(DriverAssistState.AutoIntake).setTargetGamePiece(piece));
        setFinished(intakeStatus.getIntakeState() == IntakeState.Hold);
    }

    @Override
    public void done() {
        intake.setCommand(new IntakeCommand(IntakeState.Defense));
    }
}
