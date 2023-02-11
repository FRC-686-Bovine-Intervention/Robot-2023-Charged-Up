package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

public class ArmHAL {
    private static ArmHAL instance;
    public static ArmHAL getInstance(){if(instance == null) instance = new ArmHAL(); return instance;}

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final TalonSRX turretMotor;

    private static final double kEncoderUnitsToDegrees = 360.0 / 4096.0;
    private static final boolean kTurretMotorInverted = true;

    public ArmHAL() {
        if (RobotBase.isReal()) {
            turretMotor = new TalonSRX(Constants.kTurretMotorID);
        } else {
            turretMotor = null;
        }

        if (turretMotor != null) {
            turretMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
            turretMotor.setInverted(kTurretMotorInverted);
        }
    }

    public double getTargetXOffset(){
        return table.getEntry("tx").getDouble(-686);
    }    
    
    public double getTargetYOffset(){
        return table.getEntry("ty").getDouble(-686);
    }    
    
    public double getTargetArea(){
        return table.getEntry("ta").getDouble(-686);
    }

    public boolean getTargetInView(){
        return table.getEntry("tv").getInteger(-686) == 1;
    }


    public enum LimelightPipeline {
        tape(0),
        cone(1),
        cube(2);

        public final int id;
        LimelightPipeline(int id){
            this.id = id;
        }
        public static LimelightPipeline getFromName(String name)
        {
            LimelightPipeline r = null;
            for(LimelightPipeline pipeline : LimelightPipeline.values())
            {
                if(pipeline.name().equals(name))
                {
                    r = pipeline;
                    break;
                }
            }
            return r;
        }
    }

    public LimelightPipeline getPipeline(){
        LimelightPipeline result = null;
        for (LimelightPipeline pipeline : LimelightPipeline.values()) {
            if(pipeline.id == table.getEntry("getpipe").getInteger(-1)){
                result = pipeline;
            }
        }
        return result;
    }

    public void setPipeline(LimelightPipeline p){
        table.getEntry("pipeline").setNumber(p.id);
    }

    public void setTurretPower(double power){
        if (turretMotor != null) {
            turretMotor.set(ControlMode.PercentOutput, power);
        }
    }

    public double getTurretPosition(){
        return turretMotor != null ? turretMotor.getSelectedSensorPosition() * 1.0 * kEncoderUnitsToDegrees : 0; // Gear ratio is 1:1 because of worm gear
    }
    
    
}
