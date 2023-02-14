package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.intake.IntakeStatus.IntakeState;

public class IntakeCommand {
    private IntakeState intakeState = null;
    public IntakeState getIntakeState()                             {return intakeState;}
    public IntakeCommand setIntakeState(IntakeState intakeState)    {this.intakeState = intakeState; return this;}

    public void log(Logger logger, String prefix)
    {
        logger.recordOutput(prefix + "/Input State", intakeState == null ? "null" : intakeState.name());
    }
}
