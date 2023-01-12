package frc.robot;

import frc.robot.controls.Controls;

public class DriverInteraction {
    private static DriverInteraction instance;
    public static DriverInteraction getInstance() {if(instance == null){instance = new DriverInteraction();}return instance;}

    private final Controls controls;

    private DriverInteraction()
    {
        controls = Controls.getInstance();
    }
    
    public void run()
    {

    }
}
