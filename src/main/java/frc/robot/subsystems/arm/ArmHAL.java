package frc.robot.subsystems.arm;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class ArmHAL {
    private static ArmHAL instance;
    public static ArmHAL getInstance(){if(instance == null) instance = new ArmHAL(); return instance;}

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final TalonSRX turretMotor = new TalonSRX(Constants.kTurretMotorID); //TODO: Get Devicenumber

    private static final double kEncoderUnitsToDegrees = 360.0 / 4096.0;

    public double getTargetXOffset(){
        return table.getEntry("tx").getDouble(0.0);
    }    
    
    public double getTargetYOffset(){
        return table.getEntry("ty").getDouble(0.0);
    }    
    
    public double getTargetArea(){
        return table.getEntry("ta").getDouble(0.0);
    }

    public boolean getTargetInView(){
        return table.getEntry("tv").getInteger(0) == 1;
    }

    public int getPipeline(){
        return (int) table.getEntry("getpipe").getInteger(0);
    }

    public void setPipeline(int n){
        table.getEntry("pipeline").setNumber(n);
    }

    public void setTurretPower(double power){
        turretMotor.set(ControlMode.PercentOutput, power);
    }

    public double getTurretPosition(){
        return turretMotor.getSelectedSensorPosition() * 1.0 * kEncoderUnitsToDegrees; // Gear ratio is 1:1 because of worm gear
    }
    
    
}
