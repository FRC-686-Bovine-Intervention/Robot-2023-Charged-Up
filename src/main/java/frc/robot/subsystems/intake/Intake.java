package frc.robot.subsystems.intake;

import frc.robot.subsystems.framework.SubsystemBase;

public class Intake extends SubsystemBase {
    private static Intake instance;
    public static Intake getInstance(){if(instance == null) instance = new Intake(); return instance;}

    @Override
    public void init()
    {
        Loop = IntakeLoop.getInstance();
        Status = IntakeStatus.getInstance();
    }

    private IntakeCommand command = new IntakeCommand();
    public IntakeCommand getCommand()                       {return command;}
    public Intake setCommand(IntakeCommand intakeCommand)   {this.command = intakeCommand; return this;}
}
