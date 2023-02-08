package frc.robot.subsystems.arm;

import frc.robot.subsystems.framework.LoopBase;

public class ArmLoop extends LoopBase {
    private static ArmLoop instance;
    public static ArmLoop getInstance(){if(instance == null){instance = new ArmLoop();}return instance;}

    private final Arm arm;
    private final ArmHAL HAL;
    private final ArmStatus status;

    private ArmLoop()
    {
        arm = Arm.getInstance();
        HAL = ArmHAL.getInstance();
        status = ArmStatus.getInstance();
        Subsystem = arm;
    }

    @Override
    public void Enabled() {
        // ArmCommand ArmCommand = Arm.getArmCommand();
        // setMotors(ArmCommand);
        // setNeutralMode(ArmCommand);
        // setEncoders(ArmCommand);

        // status.setCommand(ArmCommand);
    }

    @Override
    public void Disabled() {
        // ArmCommand disabledCommand = ArmCommand.COAST();

        // setMotors(disabledCommand);
        // setNeutralMode(disabledCommand);
        // setEncoders(disabledCommand);

        // status.setCommand(disabledCommand);
    }

    @Override
    public void Update() {
        
    }


}

