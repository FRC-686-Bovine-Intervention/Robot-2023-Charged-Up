package frc.robot.subsystems.driverInteraction;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    private static final double kExtendedThreshold = Units.degreesToRadians(15);
    private static final double kExtendedDrivePowerMultiplier = 0.5;
    private static final double kAdjustmentDeadband = 0.1;

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
        leftPower *= (armStatus.getShoulderAngleRad() - Math.PI / 2 >= kExtendedThreshold ? kExtendedDrivePowerMultiplier : 1);
        rightPower *= (armStatus.getShoulderAngleRad() - Math.PI / 2 >= kExtendedThreshold ? kExtendedDrivePowerMultiplier : 1);
        return new DriveCommand(leftPower, rightPower);
    }

    private double[] generateAdjustments() {
        double shoulderAdjustment = 0;
        double elbowAdjustment = 0;
        double turretAdjustment = 0;
        if(armStatus.getArmState() == ArmState.Adjust) {
            shoulderAdjustment = -DriverControlAxes.XBoxLeftY.getAxis();
            elbowAdjustment = DriverControlAxes.XBoxRightTrigger.getAxis() - DriverControlAxes.XBoxLeftTrigger.getAxis();
            turretAdjustment = DriverControlAxes.XBoxRightX.getAxis();

            if(Math.abs(shoulderAdjustment) <= kAdjustmentDeadband)
                shoulderAdjustment = 0;
            if(Math.abs(elbowAdjustment) <= kAdjustmentDeadband)
                elbowAdjustment = 0;
            if(Math.abs(turretAdjustment) <= kAdjustmentDeadband)
                turretAdjustment = 0;
        }
        return new double[]{shoulderAdjustment,elbowAdjustment,turretAdjustment};
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
                if(armStatus.EnabledState.IsEnabled && armStatus.getArmState() != ArmState.Defense)
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

        double[] adjustments = generateAdjustments();
        armCommand.setShoulderAdjustment(adjustments[0]);
        armCommand.setElbowAdjustment(adjustments[1]);
        armCommand.setTurretAdjustment(adjustments[2]);

        // Field orient buttons
        if(DriverControlButtons.ButtonBoard1_1.getRisingEdge())
            armCommand.setTargetNode(DriverStation.getAlliance() == Alliance.Red ? NodeEnum.TopWall : NodeEnum.TopLoading);
        else if(DriverControlButtons.ButtonBoard1_2.getRisingEdge())
            armCommand.setTargetNode(NodeEnum.TopCenter);
        else if(DriverControlButtons.ButtonBoard1_3.getRisingEdge())
            armCommand.setTargetNode(DriverStation.getAlliance() == Alliance.Red ? NodeEnum.TopLoading : NodeEnum.TopWall);
        else if(DriverControlButtons.ButtonBoard2_1.getRisingEdge())
            armCommand.setTargetNode(DriverStation.getAlliance() == Alliance.Red ? NodeEnum.MiddleWall : NodeEnum.MiddleLoading);
        else if(DriverControlButtons.ButtonBoard2_2.getRisingEdge())
            armCommand.setTargetNode(NodeEnum.MiddleCenter);
        else if(DriverControlButtons.ButtonBoard2_3.getRisingEdge())
            armCommand.setTargetNode(DriverStation.getAlliance() == Alliance.Red ? NodeEnum.MiddleLoading : NodeEnum.MiddleWall);
        else if(DriverControlButtons.ButtonBoard3_1.getRisingEdge())
            armCommand.setTargetNode(DriverStation.getAlliance() == Alliance.Red ? NodeEnum.BottomWall : NodeEnum.BottomLoading);
        else if(DriverControlButtons.ButtonBoard3_2.getRisingEdge())
            armCommand.setTargetNode(NodeEnum.BottomCenter);
        else if(DriverControlButtons.ButtonBoard3_3.getRisingEdge())
            armCommand.setTargetNode(DriverStation.getAlliance() == Alliance.Red ? NodeEnum.BottomLoading : NodeEnum.BottomWall);

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

            case Hold:
                if(DriverControlButtons.MainAction.getRisingEdge())
                    armCommand.setArmState(ArmState.Align);
                if(DriverControlButtons.Undo.getRisingEdge())
                    armCommand.setArmState(ArmState.Defense);
            break;
            
            case Align:
                if(DriverControlButtons.MainAction.getRisingEdge())
                    armCommand.setArmState(ArmState.Align); // Locks State
                if(DriverControlButtons.Undo.getRisingEdge())
                    armCommand.setArmState(ArmState.Hold);
                if(armCommand.getTargetNode() != null)
                    armCommand.setArmState(ArmState.Extend);
            break;

            case Adjust:
                if(DriverControlButtons.MainAction.getRisingEdge() || DriverControlButtons.Release.getRisingEdge())
                    armCommand.setArmState(ArmState.Release);
                else if(armCommand.getTargetNode() != null && armCommand.getTargetNode() != armStatus.getTargetNode())
                    armCommand.setArmState(ArmState.Extend);
                else if(DriverControlButtons.Undo.getRisingEdge() || DriverControlButtons.SecondUndo.getRisingEdge())
                    armCommand.setArmState(ArmState.Align);
            break;
            
            case SubstationExtend:
                if(DriverControlButtons.MainAction.getRisingEdge())
                    armCommand.setArmState(ArmState.SubstationGrab);
                if(DriverControlButtons.Undo.getRisingEdge())
                    armCommand.setArmState(ArmState.Defense);
            break;

            default: break;
        }

        arm.setCommand(armCommand);
    }

    @Override public void Disabled() {}
    @Override public void Update() {}
}
