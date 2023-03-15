package frc.robot.auto.modes;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotConfiguration;
import frc.robot.auto.AutoConfiguration;
import frc.robot.auto.AutoTrajectories;
import frc.robot.auto.actions.DriveOnChargeStationEdgeAction;
import frc.robot.auto.actions.DriverAssistCommandAction;
import frc.robot.auto.actions.RamseteFollowerAction;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.driverAssist.DriverAssistCommand;
import frc.robot.subsystems.driverAssist.DriverAssistStatus.DriverAssistState;

public class OnePieceBalanceAuto extends AutoMode {
    public OnePieceBalanceAuto(AutoConfiguration config) {
        Trajectory[] trajectories = new Trajectory[3];

        trajectories[0] = AutoTrajectories.ScoringBackward[DriverStation.getAlliance().ordinal()][config.startingPosition.ordinal()];
        trajectories[1] = AutoTrajectories.PickupForward[DriverStation.getAlliance().ordinal()][config.startingPosition.ordinal()];
        trajectories[2] = AutoTrajectories.StationBackward[DriverStation.getAlliance().ordinal()][config.startingPosition.ordinal()];

        startConfiguration = new RobotConfiguration(trajectories[0].getInitialPose(), ArmPose.Preset.AUTO_START, ArmState.Hold);

        RamseteController ramseteController = new RamseteController(2, 0.7);

        addAction(new RamseteFollowerAction(trajectories[0], ramseteController));
        addAction(new RamseteFollowerAction(trajectories[1], ramseteController));
        addAction(new RamseteFollowerAction(trajectories[2], ramseteController));
        addAction(new DriveOnChargeStationEdgeAction(true));
        addAction(new DriverAssistCommandAction(new DriverAssistCommand(DriverAssistState.AutoBalance)));
    }
}
