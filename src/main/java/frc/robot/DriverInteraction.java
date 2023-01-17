package frc.robot;

import frc.robot.controls.Controls;
import frc.robot.controls.Controls.ButtonControlEnum;
import frc.robot.subsystems.drive.Drive;

public class DriverInteraction {
    private static DriverInteraction instance;
    public static DriverInteraction getInstance() {if(instance == null){instance = new DriverInteraction();}return instance;}

    private final Controls controls;

    private boolean invertDriveControls = false;

    private boolean prevInvertButton = false;

    private DriverInteraction()
    {
        controls = Controls.getInstance();
    }
    
    public void run()
    {
        if(controls.getButton(ButtonControlEnum.FORWARD) && ! prevInvertButton)
        {
            invertDriveControls = !invertDriveControls;
        }
        Drive.getInstance().setDriveCommand(controls.getDriveCommand(invertDriveControls));
        prevInvertButton = controls.getButton(ButtonControlEnum.FORWARD);
    }
}
