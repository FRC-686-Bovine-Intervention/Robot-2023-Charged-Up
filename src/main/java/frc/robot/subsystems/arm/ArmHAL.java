package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.lib.util.LimelightHelpers;

public class ArmHAL {
    private static ArmHAL instance;
    public static ArmHAL getInstance(){if(instance == null) instance = new ArmHAL(); return instance;}

    private final TalonSRX turretMotor;

    private final String kLimelightName = "limelight";

    private static final double kEncoderUnitsToDegrees = 360.0 / 4096.0;
    private static final boolean kTurretMotorInverted = true;
    private static final boolean kTurretEncoderInverted = true;

    public ArmHAL() {
        if (RobotBase.isReal()) {
            turretMotor = new TalonSRX(Constants.kTurretMotorID);
        } else {
            turretMotor = null;
        }

        if (turretMotor != null) {
            turretMotor.configFactoryDefault();
            turretMotor.getSensorCollection().syncQuadratureWithPulseWidth(0, 0, true);
            turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
            turretMotor.setSensorPhase(kTurretEncoderInverted);
            turretMotor.setInverted(kTurretMotorInverted);
        }
    }

    public double getTargetXOffset(){
        return LimelightHelpers.getTX(kLimelightName);
    }    
    
    public double getTargetYOffset(){
        return LimelightHelpers.getTY(kLimelightName);
    }    
    
    public double getTargetArea(){
        return LimelightHelpers.getTA(kLimelightName);
    }

    public boolean getTargetInView(){
        return LimelightHelpers.getTV(kLimelightName);
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
            if(pipeline.id == LimelightHelpers.getCurrentPipelineIndex(kLimelightName)){
                result = pipeline;
            }
        }
        return result;
    }

    public ArmHAL setPipeline(LimelightPipeline p){
        LimelightHelpers.setPipelineIndex(kLimelightName, p.id);
        return this;
    }

    public ArmHAL setTurretPower(double power){
        if (turretMotor != null) {
            turretMotor.set(ControlMode.PercentOutput, power);
        }
        return this;
    }

    public double getTurretRelative(){
        return turretMotor != null ? turretMotor.getSelectedSensorPosition() * 1.0 * kEncoderUnitsToDegrees : 0; // Gear ratio is 1:1 because of worm gear
    }
    // public double getTurretAbsolute(){
    //     return turretEncoder != null ? turretEncoder.get * 1.0 * kEncoderUnitsToDegrees : 0; // Gear ratio is 1:1 because of worm gear
    // }
}
