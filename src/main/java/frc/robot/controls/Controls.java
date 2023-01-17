package frc.robot.controls;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveCommand;

public class Controls {
    private static Controls instance;
    public static Controls getInstance() {if(instance == null){instance = new Controls();}return instance;}

    Joystick thrustmaster, buttonboard;

    public Controls()
    {
        thrustmaster =  new Joystick(Constants.kThrustmasterPort);
        buttonboard =   new Joystick(Constants.kButtonboardPort);
    }

    public enum JoystickEnum {THRUSTMASTER, BUTTONBOARD}
    
    public Translation2d getAxis(JoystickEnum joystick)
    {
        switch(joystick)
        {
            case THRUSTMASTER:  default:    return new Translation2d(-thrustmaster.getRawAxis(0),    -thrustmaster.getRawAxis(1));
            case BUTTONBOARD:               return new Translation2d(buttonboard.getRawAxis(0),      buttonboard.getRawAxis(1));
        }
    }

    public int getPOV(JoystickEnum joystick)
    {
        switch(joystick)
        {
            case THRUSTMASTER:  default:    return thrustmaster.getPOV();
            case BUTTONBOARD:               return buttonboard.getPOV();
        }
    }

    public enum ButtonControlEnum {
        FORWARD,
        BACKWARD,
        FAST_FORWARD
    }
    
    public boolean getButton(ButtonControlEnum button)
    {
        switch(button)
        {
            case FORWARD:                   return thrustmaster.getRawButton(Thrustmaster.kTriggerButton);
            case BACKWARD:                  return thrustmaster.getRawButton(Thrustmaster.kBottomThumbButton);
            case FAST_FORWARD:              return thrustmaster.getRawButton(Thrustmaster.kRightThumbButton);
            default:                        return false;
        }
    }

    public DriveCommand getDriveCommand() {return getDriveCommand(false);}
    public DriveCommand getDriveCommand(boolean inverted)
    {
        double x = getAxis(JoystickEnum.THRUSTMASTER).getX();
        double y = getAxis(JoystickEnum.THRUSTMASTER).getY();
        x = 0.8*x*x*x - 0.8*x + x;
        y = 0.7*y*y*y - 0.7*y + y;

        x *= 0.7;

        if(inverted)
        {
            y *= -1;
        }

        double leftPower = y-x;
        double rightPower = y+x;
        return new DriveCommand(leftPower, rightPower);
    }
}
