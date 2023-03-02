package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

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

    public void recordOutputs(Logger logger, String prefix) {
        logger.recordOutput(prefix + "/Arm State", armState != null ? armState.name() : "null");
    }
}
