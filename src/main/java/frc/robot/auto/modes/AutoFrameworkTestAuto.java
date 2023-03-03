package frc.robot.auto.modes;

import frc.robot.auto.actions.ArmCommandAction;
import frc.robot.auto.actions.DriveStraightAction;
import frc.robot.auto.actions.IntakeCommandAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.auto.actions.WaitUntilAction;
import frc.robot.subsystems.arm.ArmCommand;
import frc.robot.subsystems.arm.ArmStatus;
import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.arm.ArmStatus.NodeEnum;
import frc.robot.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeStatus.IntakeState;

public class AutoFrameworkTestAuto extends AutoMode {
    public AutoFrameworkTestAuto() {
        addAction(new WaitUntilAction() {
            @Override
            public boolean Condition() {
                return ArmStatus.getInstance().getArmState() == ArmState.Defense;
            }
        });
        addAction(new ArmCommandAction(new ArmCommand(ArmState.Extend).setTargetNode(NodeEnum.MiddleCenter)));
        addAction(new WaitUntilAction() {
            @Override
            public boolean Condition() {
                return ArmStatus.getInstance().getArmState() == ArmState.Adjust;
            }
        });
        addAction(new ArmCommandAction(new ArmCommand(ArmState.Release)));
        addAction(new WaitUntilAction() {
            @Override
            public boolean Condition() {
                return ArmStatus.getInstance().getArmState() == ArmState.Defense;
            }
        });
        addAction(new WaitAction(2).setTimeout(1.5));
        addAction(new ParallelAction(
            new IntakeCommandAction(new IntakeCommand(IntakeState.Grab)), 
            new SeriesAction(
                new DriveStraightAction(0.1, 3).setTimeout(1), 
                new DriveStraightAction(-0.1, 2).setTimeout(2)
            )
        ));
    }
}
