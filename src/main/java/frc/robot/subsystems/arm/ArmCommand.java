package frc.robot.subsystems.arm;

import frc.robot.subsystems.arm.ArmStatus.ArmState;

public class ArmCommand {
    public ArmCommand() {this(null);}
    public ArmCommand(ArmState armState)
    {
        setArmState(armState);
    }

    private ArmState    armState = null;
    public ArmState     getArmState()                   {return armState;}
    public ArmCommand   setArmState(ArmState armState)  {this.armState = armState; return this;}
}
