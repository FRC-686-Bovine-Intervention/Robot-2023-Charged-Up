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

    private Double      shoulderAdjustment = null;
    public Double       getShoulderAdjustment()                    {return shoulderAdjustment;}
    public ArmCommand   setShoulderAdjustment(double shoulderAdjustment)  {this.shoulderAdjustment = shoulderAdjustment; return this;}

    private Double      elbowAdjustment = null;
    public Double       getElbowAdjustment()                    {return elbowAdjustment;}
    public ArmCommand   setElbowAdjustment(double elbowAdjustment)  {this.elbowAdjustment = elbowAdjustment; return this;}

    private Boolean     elbowRaised = null;
    public Boolean      getElbowRaised()                    {return elbowRaised;}
    public ArmCommand   setElbowRaised(boolean elbowRaised) {this.elbowRaised = elbowRaised; return this;}

    private Double      turretAdjustment = null;
    public Double       getTurretAdjustment()                           {return turretAdjustment;}
    public ArmCommand   setTurretAdjustment(double turretAdjustment)    {this.turretAdjustment = turretAdjustment; return this;}

    public void recordOutputs(Logger logger, String prefix) {
        logger.recordOutput(prefix + "/Arm State", armState != null ? armState.name() : "null");
        logger.recordOutput(prefix + "/Target Node", targetNode != null ? targetNode.name() : "null");
        logger.recordOutput(prefix + "/Adjustment/Shoulder", shoulderAdjustment != null ? shoulderAdjustment.doubleValue() : 0);
        logger.recordOutput(prefix + "/Adjustment/Elbow", elbowAdjustment != null ? elbowAdjustment.doubleValue() : 0);
        logger.recordOutput(prefix + "/Adjustment/Turret", turretAdjustment != null ? turretAdjustment.doubleValue() : 0);
    }
}
