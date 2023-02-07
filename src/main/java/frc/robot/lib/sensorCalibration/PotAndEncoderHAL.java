package frc.robot.lib.sensorCalibration;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class PotAndEncoderHAL {

    AnalogPotentiometer pot;
    CANCoder enc;

    public PotAndEncoderHAL(int potAnalogInputPort, int encPort, PotAndEncoderConfig potAndEncoderConfig)
    {
        pot = new AnalogPotentiometer(new AnalogInput(potAnalogInputPort), potAndEncoderConfig.potentiometerNTurns*360.0, 
                potAndEncoderConfig.potentiometerAngleDegAtCalib - potAndEncoderConfig.outputAngleDegAtCalibration);

        enc = new CANCoder(encPort);
        CANCoderConfiguration canConfig = new CANCoderConfiguration();
        enc.configAllSettings(canConfig);  // configure to default settings
    }

    public double getPotentiometerReadingDeg()     {return pot.get();}
    public double getAbsoluteEncoderReadingDeg()   {return enc.getAbsolutePosition();}
    public double getRelativeEncoderReadingDeg()   {return enc.getPosition();}
}
