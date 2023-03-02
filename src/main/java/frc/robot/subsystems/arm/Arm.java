package frc.robot.subsystems.arm;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.arm.json.ArmConfigJson;
import frc.robot.subsystems.framework.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm instance;
    public static Arm getInstance(){if(instance == null) instance = new Arm(); return instance;}

    private final ArmConfigJson config;
    public ArmConfigJson        getConfig() {return config;}

    private Arm() {
        File configFile = new File(Filesystem.getDeployDirectory(), ArmConfigJson.jsonFilename);
        config = ArmConfigJson.loadJson(configFile);
    }

    @Override
    public void init()
    {
        Loop = ArmLoop.getInstance();
        Status = ArmStatus.getInstance();
    }

    private ArmCommand command = new ArmCommand();
    public ArmCommand getCommand()                  {return command;}
    public Arm setCommand(ArmCommand armCommand)    {this.command = armCommand; return this;}
}
