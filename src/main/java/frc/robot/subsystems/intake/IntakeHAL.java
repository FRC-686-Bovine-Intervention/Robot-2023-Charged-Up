package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class IntakeHAL {
    private static IntakeHAL instance;
    public static IntakeHAL getInstance(){if(instance == null) instance = new IntakeHAL(); return instance;}

    private final CANSparkMax intakeMotor;
    private final Solenoid deploySolenoid;

    private static final boolean kIntakeMotorInverted = false;
    private static final boolean kDeploySolenoidInverted = false;
    private static final double kOpenLoopRamp = 0.5;
    private static final int kStallCurrentLimit = 20;
    private static final int kFreeCurrentLimit = 20;

    private IntakeHAL()
    {
        if(RobotBase.isReal())
        {
            intakeMotor = new CANSparkMax(Constants.kRollerMotorID, MotorType.kBrushless);
            deploySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.kIntakeSolenoidID);
        }
        else
        {
            intakeMotor = null;
            deploySolenoid = null;
        }
        //TODO: Configure intake HAL
        if(intakeMotor != null)
        {
            intakeMotor.setInverted(kIntakeMotorInverted);
            intakeMotor.setOpenLoopRampRate(kOpenLoopRamp);
            intakeMotor.setSmartCurrentLimit(kStallCurrentLimit, kFreeCurrentLimit);
        }
    }

    public double getIntakeCurrent() {return intakeMotor != null ? intakeMotor.getOutputCurrent() : 0;}

    public IntakeHAL setIntakeMotor(double power)
    {
        if(intakeMotor != null)
            intakeMotor.set(power);
        return this;
    }
    public IntakeHAL setIntakeNeutralMode(IdleMode mode)
    {
        if(intakeMotor != null)
            intakeMotor.setIdleMode(mode);
        return this;
    }
    public IntakeHAL setIntakeStallCurrentLimit(int stallCurrentLimit)
    {
        if(intakeMotor != null)
            intakeMotor.setSmartCurrentLimit(stallCurrentLimit, kFreeCurrentLimit);
        return this;
    }
    public IntakeHAL setDeploySolenoid(boolean intakeDeployed)
    {
        if(deploySolenoid != null)
            deploySolenoid.set(intakeDeployed ^ kDeploySolenoidInverted);
        return this;
    }
}
