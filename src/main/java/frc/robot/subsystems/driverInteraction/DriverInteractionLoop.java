package frc.robot.subsystems.driverInteraction;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmCommand;
import frc.robot.subsystems.arm.ArmStatus;
import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.arm.ArmStatus.NodeEnum;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.driverAssist.DriverAssist;
import frc.robot.subsystems.driverAssist.DriverAssistCommand;
import frc.robot.subsystems.driverAssist.DriverAssistStatus.DriverAssistState;
import frc.robot.subsystems.driverInteraction.DriverInteractionStatus.DriverControlAxes;
import frc.robot.subsystems.driverInteraction.DriverInteractionStatus.DriverControlButtons;
import frc.robot.subsystems.framework.LoopBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeStatus;
import frc.robot.subsystems.intake.IntakeStatus.IntakeState;

public class DriverInteractionLoop extends LoopBase {
    private static DriverInteractionLoop instance;
    public static DriverInteractionLoop getInstance() {if(instance == null){instance = new DriverInteractionLoop();}return instance;}

    private final Drive drive = Drive.getInstance();
    private final DriverAssist driverAssist = DriverAssist.getInstance();
    private final Arm arm = Arm.getInstance();
    private final ArmStatus armStatus = ArmStatus.getInstance();
    private final Intake intake = Intake.getInstance();
    private final IntakeStatus intakeStatus = IntakeStatus.getInstance();

    private DriverInteractionLoop() {Subsystem = DriverInteraction.getInstance();}

    private boolean invertDriveControls = false;

    private DriveCommand generateDriveCommand()
    {
        double turn =       DriverControlAxes.ThrustmasterX.getAxis();
        double throttle =   DriverControlAxes.ThrustmasterY.getAxis();

        turn = 0.8*turn*turn*turn - 0.8*turn + turn;
        throttle = 0.7*throttle*throttle*throttle - 0.7*throttle + throttle;

        turn *= 0.7;

        if(invertDriveControls)
            throttle *= -1;

        double leftPower = throttle+turn;
        double rightPower = throttle-turn;
        return new DriveCommand(leftPower, rightPower);
    }

    @Override
    public void Enabled() {
        if(!DriverStation.isTeleop()) return;

        if(DriverControlButtons.InvertDrive.getRisingEdge())
            invertDriveControls = !invertDriveControls;
        drive.setDriveCommand(generateDriveCommand());

        DriverAssistCommand assistCommand = new DriverAssistCommand();
        if(DriverControlButtons.AutoDrive.getButton())
            assistCommand.setDriverAssistState(DriverAssistState.AutoBalance);
        driverAssist.setCommand(assistCommand);

        IntakeCommand intakeCommand = new IntakeCommand();
        switch(intakeStatus.getIntakeState())
        {
            case Defense:
                if(!armStatus.EnabledState.IsEnabled && armStatus.getArmState() != ArmState.Defense)
                    break;
                if(DriverControlButtons.MainAction.getRisingEdge())
                    intakeCommand.setIntakeState(IntakeState.Grab);
            break;

            case Grab:
                if(!DriverControlButtons.MainAction.getButton())
                    intakeCommand.setIntakeState(IntakeState.Defense);
            break;

            case Hold:
                if(armStatus.EnabledState.IsEnabled)
                    break;
                if(DriverControlButtons.MainAction.getRisingEdge())
                    intakeCommand.setIntakeState(IntakeState.Release);
            break;
            
            case Release:
                if(armStatus.EnabledState.IsEnabled)
                        break;
                if(!DriverControlButtons.MainAction.getButton())
                    intakeCommand.setIntakeState(IntakeState.Defense);
            break;
        }
        intake.setCommand(intakeCommand);

        ArmCommand armCommand = new ArmCommand();

        armCommand.setXAdjustment(DriverControlAxes.ThrustmasterX.getAxis());
        armCommand.setZAdjustment(DriverControlAxes.ThrustmasterY.getAxis());

        if(DriverControlButtons.ButtonBoard1_1.getRisingEdge())
            armCommand.setTargetNode(NodeEnum.BottomLeft);
        else if(DriverControlButtons.ButtonBoard1_2.getRisingEdge())
            armCommand.setTargetNode(NodeEnum.BottomCenter);
        else if(DriverControlButtons.ButtonBoard1_3.getRisingEdge())
            armCommand.setTargetNode(NodeEnum.BottomRight);
        else if(DriverControlButtons.ButtonBoard2_1.getRisingEdge())
            armCommand.setTargetNode(NodeEnum.MiddleLeft);
        else if(DriverControlButtons.ButtonBoard2_2.getRisingEdge())
            armCommand.setTargetNode(NodeEnum.MiddleCenter);
        else if(DriverControlButtons.ButtonBoard2_3.getRisingEdge())
            armCommand.setTargetNode(NodeEnum.MiddleRight);
        else if(DriverControlButtons.ButtonBoard3_1.getRisingEdge())
            armCommand.setTargetNode(NodeEnum.TopLeft);
        else if(DriverControlButtons.ButtonBoard3_2.getRisingEdge())
            armCommand.setTargetNode(NodeEnum.TopCenter);
        else if(DriverControlButtons.ButtonBoard3_3.getRisingEdge())
            armCommand.setTargetNode(NodeEnum.TopRight);

        switch(ArmStatus.getInstance().getArmState())
        {
            case Defense:
                if(DriverControlButtons.Undo.getRisingEdge())
                    armCommand.setArmState(ArmState.SubstationExtend);
            break;

            case IdentifyPiece:
                if(DriverControlButtons.MainAction.getRisingEdge())
                    armCommand.setArmState(ArmState.Grab);
            break;

            case Grab:
                // if(DriverControlButtons.MainAction.getRisingEdge())
                //     armCommand.setArmState(ArmState.Hold);
            break;

            case Hold:
                if(DriverControlButtons.MainAction.getRisingEdge())
                    armCommand.setArmState(ArmState.Align);
                if(DriverControlButtons.Undo.getRisingEdge())
                    armCommand.setArmState(ArmState.Hold); // Locks State
            break;
            
            case Align:
                if(DriverControlButtons.Undo.getRisingEdge())
                    armCommand.setArmState(ArmState.Align); // Locks State
                if(armCommand.getTargetNode() != null)
                    armCommand.setArmState(ArmState.Extend);
            break;

            case Adjust:
                if(DriverControlButtons.MainAction.getRisingEdge() || armCommand.getTargetNode() == armStatus.getTargetNode())
                    armCommand.setArmState(ArmState.Release);
                else if(armCommand.getTargetNode() != null)
                    armCommand.setArmState(ArmState.Extend);
            break;
            
            case SubstationExtend:
                if(DriverControlButtons.MainAction.getRisingEdge())
                    armCommand.setArmState(ArmState.SubstationGrab);
            break;

            default: break;
        }

        arm.setCommand(armCommand);
    }

    @Override public void Disabled() {}
    @Override public void Update() {}
}
