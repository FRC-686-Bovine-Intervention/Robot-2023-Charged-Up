package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.IntakeStatus.IntakeState;

public class IntakeCommand {
    private IntakeState intakeState = null;
    public IntakeState getIntakeState()                             {return intakeState;}
    public IntakeCommand setIntakeState(IntakeState intakeState)    {this.intakeState = intakeState; return this;}
}
