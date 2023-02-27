package frc.robot.subsystems.driverInteraction;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.joysticks.ButtonBoard3x3;
import frc.robot.joysticks.Thrustmaster;
import frc.robot.subsystems.framework.StatusBase;

public class DriverInteractionStatus extends StatusBase {
    private static DriverInteractionStatus instance;
    public static DriverInteractionStatus getInstance() {if(instance == null){instance = new DriverInteractionStatus();}return instance;}

    public enum Joysticks{
        Thrustmaster(new Joystick(Constants.kThrustmasterPort));

        public final Joystick joystick;
        Joysticks(Joystick joystick)                {this.joystick = joystick;}
        public boolean getRawButton(int buttonID)   {return joystick == null ? false : joystick.getRawButton(buttonID);}
        public double getRawAxis(int axisID)        {return joystick == null ? 0.0   : joystick.getRawAxis(axisID);}
        public int getRawPOV(int POVID)             {return joystick == null ? -1    : joystick.getPOV(POVID);}
        public int getRawPOV()                      {return getRawPOV(0);}
    }

    public enum DriverControlButtons{
        InvertControls  (Thrustmaster.kLeftThumbButton),
        Intake          (Thrustmaster.kTriggerButton),
        AutoBalance     (Thrustmaster.kBottomThumbButton),
        DriverAssist    (Thrustmaster.kRightThumbButton),
        Substation      (Thrustmaster.kTopButton3),
        ButtonBoard1_1  (ButtonBoard3x3.kButton1_1),
        ButtonBoard1_2  (ButtonBoard3x3.kButton1_2),
        ButtonBoard1_3  (ButtonBoard3x3.kButton1_3),
        ButtonBoard2_1  (ButtonBoard3x3.kButton2_1),
        ButtonBoard2_2  (ButtonBoard3x3.kButton2_2),
        ButtonBoard2_3  (ButtonBoard3x3.kButton2_3),
        ButtonBoard3_1  (ButtonBoard3x3.kButton3_1),
        ButtonBoard3_2  (ButtonBoard3x3.kButton3_2),
        ButtonBoard3_3  (ButtonBoard3x3.kButton3_3);

        private final Joysticks joystick;
        private final int buttonID;
        DriverControlButtons(int buttonID)                      {this(Joysticks.Thrustmaster, buttonID);}
        DriverControlButtons(Joysticks joystick, int buttonID)  {this.joystick = joystick; this.buttonID = buttonID;}
        private boolean value = false;
        private boolean prevValue = false;
        public boolean getButton()                              {return value;}
        public boolean getRisingEdge()                          {return value && !prevValue;}
        public boolean getFallingEdge()                         {return !value && prevValue;}
        private DriverControlButtons setValue(boolean value)    {this.value = value; return this;}
        private DriverControlButtons update()                   {return update(joystick.getRawButton(buttonID));}
        private DriverControlButtons update(boolean value)      {this.prevValue = this.value; return setValue(value);}
    }

    public enum DriverControlAxes{
        ThrustmasterX       (Thrustmaster.kXAxisID,         Thrustmaster.kXAxisInvert),
        ThrustmasterY       (Thrustmaster.kYAxisID,         Thrustmaster.kYAxisInvert),
        ThrustmasterRotation(Thrustmaster.kZRotateAxisID,   Thrustmaster.kZRotateAxisInvert),
        ThrustmasterSlider  (Thrustmaster.kSliderAxisID,    Thrustmaster.kSliderAxisInvert);

        private final Joysticks joystick;
        private final int axisID;
        private final boolean invertAxis;
        DriverControlAxes(int axisID)                       {this(Joysticks.Thrustmaster, axisID);}
        DriverControlAxes(int axisID, boolean invertAxis)   {this(Joysticks.Thrustmaster, axisID, invertAxis);}
        DriverControlAxes(Joysticks joystick, int axisID)   {this(joystick, axisID, false);}
        DriverControlAxes(Joysticks joystick, int axisID, boolean invertAxis)   {this.joystick = joystick; this.axisID = axisID; this.invertAxis = invertAxis;}
        private double value = 0.0;
        public double getAxis()                             {return value;}
        private DriverControlAxes setValue(double value)    {this.value = value; return this;}
        private DriverControlAxes update()                  {return update(joystick.getRawAxis(axisID) * (invertAxis ? -1 : 1));}
        private DriverControlAxes update(double value)      {return setValue(value);}
    }

    public enum DriverControlPOVs{
        ThrustmasterPOV (Joysticks.Thrustmaster);

        private final Joysticks joystick;
        private final int POVID;
        DriverControlPOVs(Joysticks joystick)               {this(joystick, 0);}
        DriverControlPOVs(Joysticks joystick, int POVID)    {this.joystick = joystick; this.POVID = POVID;}
        private int value = -1;
        private int prevValue = -1;
        public int getPOV()                             {return value;}
        public int getRisingEdge()                      {return value != prevValue ? value : -1;}
        public int getFallingEdge()                     {return value != prevValue ? prevValue : -1;}
        public boolean getPOV(int checkValue)           {return getPOV() == checkValue;}
        public boolean getRisingEdge(int checkValue)    {return getRisingEdge() == checkValue;}
        public boolean getFallingEdge(int checkValue)   {return getFallingEdge() == checkValue;}
        private DriverControlPOVs setValue(int value)   {this.value = value; return this;}
        private DriverControlPOVs update()              {return update(joystick.getRawPOV(POVID));}
        private DriverControlPOVs update(int value)     {this.prevValue = this.value; return setValue(value);}
    }

    private DriverInteractionStatus() {Subsystem = DriverInteraction.getInstance();}

    @Override
    protected void updateInputs() {
        for(DriverControlButtons button : DriverControlButtons.values())
            button.update();
        for(DriverControlAxes axis : DriverControlAxes.values())
            axis.update();
        for(DriverControlPOVs POV : DriverControlPOVs.values())
            POV.update();
    }

    @Override
    protected void processOutputs(Logger logger, String prefix) {
        for(DriverControlButtons button : DriverControlButtons.values())
            logger.recordOutput(prefix + "Joystick Buttons/" + button.name(), button.getButton());
        for(DriverControlAxes axis : DriverControlAxes.values())
            logger.recordOutput(prefix + "Joystick Axes/" + axis.name(), axis.getAxis());
        for(DriverControlPOVs POV : DriverControlPOVs.values())
            logger.recordOutput(prefix + "Joystick POVs/" + POV.name(), POV.getPOV());
    }

    @Override protected void exportToTable(LogTable table) {}
    @Override protected void importFromTable(LogTable table) {}
    @Override protected void processTable() {}
}
