import static org.junit.jupiter.api.Assertions.*;
// import static org.junit.jupiter.api.Assumptions.*;
import org.junit.jupiter.api.*;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.subsystems.drive.DriveCommand;
import frc.robot.subsystems.drive.DriveStatus;
import frc.robot.subsystems.drive.DriveCommand.DriveControlMode;

public class JUnitTests {
    @Test
    public void DriveSetTalonMode()
    {
        ControlMode setPoint = ControlMode.MusicTone;
        DriveStatus.getInstance().setTalonMode(setPoint);
        assertSame(setPoint,DriveStatus.getInstance().getTalonMode());
    }
    @Test
    public void DriveSetCommand()
    {
        DriveCommand setPoint = new DriveCommand(DriveControlMode.OPEN_LOOP, null);
        DriveStatus.getInstance().setCommand(setPoint);
        assertSame(setPoint,DriveStatus.getInstance().getCommand());
    }
}
