package frc.robot.auto.modes;

import frc.robot.auto.actions.DrivePercentAction;
import frc.robot.auto.actions.DriveVelocityAction;

public class DriveStraightAuto extends AutoMode {
    public DriveStraightAuto() {
        addAction(new DriveVelocityAction(240, 24).setTimeout(11));
        // addAction(new DrivePercentAction(0.5, 15000));
    }
}
