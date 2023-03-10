package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.arm.ArmStatus.ArmState;
import frc.robot.subsystems.arm.ArmStatus.NodeEnum;

public class ArmCommand {
    public ArmCommand() {this(null);}
    public ArmCommand(ArmState armState)
    {
        setArmState(armState);
    }

    private ArmState    armState = null;
    public ArmState     getArmState()                   {return armState;}
    public ArmCommand   setArmState(ArmState armState)  {this.armState = armState; return this;}

    private NodeEnum    targetNode = null;
    public NodeEnum     getTargetNode()                     {return targetNode;}
    public ArmCommand   setTargetNode(NodeEnum targetNode)  {this.targetNode = targetNode; return this;}

    private Double      xAdjustment = null;
    public Double       getXAdjustment()                    {return xAdjustment;}
    public ArmCommand   setXAdjustment(double xAdjustment)  {this.xAdjustment = xAdjustment; return this;}

    private Double      zAdjustment = null;
    public Double       getZAdjustment()                    {return zAdjustment;}
    public ArmCommand   setZAdjustment(double zAdjustment)  {this.zAdjustment = zAdjustment; return this;}

    public void recordOutputs(Logger logger, String prefix) {
        logger.recordOutput(prefix + "/Arm State", armState != null ? armState.name() : "null");
        logger.recordOutput(prefix + "/Target Node", targetNode != null ? targetNode.name() : "null");
        logger.recordOutput(prefix + "/Adjustment/X", xAdjustment != null ? xAdjustment.toString() : "null");
        logger.recordOutput(prefix + "/Adjustment/Z", zAdjustment != null ? zAdjustment.toString() : "null");
    }
}
