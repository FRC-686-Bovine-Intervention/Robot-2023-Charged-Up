package frc.robot.subsystems.driverInteraction;

import com.ctre.phoenix.motorcontrol.NeutralMode;

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
import frc.robot.subsystems.drive.DriveCommand.DriveControlMode;
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
    private static final double kHandoffPowerMultiplier = 0.5;

    private static final double kAdjustmentDeadband = 0.15;

    private final Drive drive = Drive.getInstance();
    private final DriverAssist driverAssist = DriverAssist.getInstance();
    private final Arm arm = Arm.getInstance();
    private final ArmStatus armStatus = ArmStatus.getInstance();
    private final Intake intake = Intake.getInstance();
    private final IntakeStatus intakeStatus = IntakeStatus.getInstance();

    private DriverInteractionLoop() {Subsystem = DriverInteraction.getInstance();}

    private boolean invertDriveControls = false;

    private static final double kTurnPercent = 0.8;
    private static final double kThrottlePercent = 0.7;

    private DriveCommand generateDriveCommand()
    {
        double turn =       DriverControlAxes.ThrustmasterX.getAxis();
        double throttle =   DriverControlAxes.ThrustmasterY.getAxis();

        turn = kTurnPercent*turn*turn*turn - kTurnPercent*turn + turn;
        throttle = kThrottlePercent*throttle*throttle*throttle - kThrottlePercent*throttle + throttle;

        turn *= 0.7;
        turn *= (armStatus.getShoulderAngleRad() + Math.PI / 2 >= kExtendedThreshold ? kExtendedDrivePowerMultiplier : 1);

        if(invertDriveControls)
            throttle *= -1;

        double leftPower = throttle+turn;
        double rightPower = throttle-turn;
        double handoffPowerMultiplier = (armStatus.getArmState() == ArmState.Grab ? kHandoffPowerMultiplier : 1);
        leftPower *= handoffPowerMultiplier;
        rightPower *= handoffPowerMultiplier;
        return new DriveCommand(DriveControlMode.OPEN_LOOP, leftPower, rightPower, (DriverControlButtons.ParkingBrake.getButton() ? NeutralMode.Brake : NeutralMode.Coast));
    }

    private ArmCommand generateAdjustments() {
        ArmCommand command = new ArmCommand();
        command.setShoulderAdjustment(0)
               .setElbowAdjustment(0)
               .setTurretAdjustment(0);
        if(armStatus.getArmState() == ArmState.Adjust || armStatus.getArmState() == ArmState.Emergency || armStatus.getArmState() == ArmState.SubstationExtend) {
            double shoulderAdjustment = -DriverControlAxes.XBoxLeftY.getAxis();
            double elbowAdjustment = DriverControlAxes.XBoxRightY.getAxis();
            double turretAdjustment = DriverControlAxes.XBoxRightTrigger.getAxis() - DriverControlAxes.XBoxLeftTrigger.getAxis();
            boolean elbowRaised = DriverControlButtons.RaiseElbow.getRisingEdge();
            command.setShoulderAdjustment(Math.abs(shoulderAdjustment) > kAdjustmentDeadband ? shoulderAdjustment : 0);
            command.setElbowAdjustment(Math.abs(elbowAdjustment) > kAdjustmentDeadband ? elbowAdjustment : 0);
            command.setTurretAdjustment(Math.abs(turretAdjustment) > kAdjustmentDeadband ? turretAdjustment : 0);
            command.setElbowRaised(armStatus.getElbowRaised() ^ elbowRaised);
        }
        return command;
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

        ArmCommand armCommand = generateAdjustments();

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

            case Grab:
                if(DriverControlButtons.Undo.getRisingEdge())
                    armCommand.setArmState(ArmState.Defense);
            break;

            case Hold:
                if(DriverControlButtons.MainAction.getRisingEdge() || DriverControlButtons.SecondAlign.getRisingEdge())
                    armCommand.setArmState(ArmState.AlignWall);
                if(DriverControlButtons.Undo.getRisingEdge())
                    armCommand.setArmState(ArmState.Defense);
            break;
            
            case AlignWall:
                if(DriverControlButtons.Undo.getRisingEdge())
                    armCommand.setArmState(ArmState.Hold);
                if(DriverControlButtons.MainAction.getRisingEdge()) {
                    armCommand.setArmState(ArmState.AlignNode);
                    invertDriveControls = false;
                }
            break;

            case Adjust:
                if(DriverControlButtons.MainAction.getRisingEdge() || DriverControlButtons.Release.getRisingEdge())
                    armCommand.setArmState(ArmState.Release);
                else if(armCommand.getTargetNode() != null && armCommand.getTargetNode() != armStatus.getTargetNode())
                    armCommand.setArmState(ArmState.Extend);
                else if(DriverControlButtons.Undo.getRisingEdge() || DriverControlButtons.SecondUndo.getRisingEdge())
                    armCommand.setArmState(ArmState.AlignWall);
            break;
            
            case SubstationExtend:
                if(DriverControlButtons.MainAction.getRisingEdge()) {
                    armCommand.setArmState(ArmState.SubstationGrab);
                    invertDriveControls = true;
                }
                if(DriverControlButtons.Undo.getRisingEdge())
                    armCommand.setArmState(ArmState.Defense);
            break;

            default: break;
        }

        if(DriverControlButtons.ReZeroArm.getRisingEdge())
            armCommand.setArmState(ArmState.ZeroDistalUp);
        if(DriverControlButtons.Oopsie.getRisingEdge())
            armCommand.setArmState(armStatus.getArmState() != ArmState.Emergency ? ArmState.Emergency : ArmState.ZeroDistalUp);
        
        arm.setCommand(armCommand);
    }

    @Override public void Disabled() {}
    @Override public void Update() {}
}
