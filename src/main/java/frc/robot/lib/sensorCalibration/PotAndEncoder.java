package frc.robot.lib.sensorCalibration;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.lib.util.Unwrapper;

public class PotAndEncoder {
    private final Config config;

    private static final Unwrapper absUnwrapper = new Unwrapper(0.0, 360.0);
    private static final Unwrapper relUnwrapper = new Unwrapper(0.0, 360.0);

    private static final double movingThreshold = 2.0 / 4096.0;
    private final CircularBuffer movingBuffer;
    private final CircularBuffer absRelDifferenceBuffer;
    private final CircularBuffer potDifferenceBuffer;

    private final double kAllowableErrorInOutputAngleDeg;

    public PotAndEncoder(Config potAndEncoderConfig)
    {
        this.config = potAndEncoderConfig;
        this.kAllowableErrorInOutputAngleDeg = 0.9 * 360.0 / potAndEncoderConfig.encoderGearRatio;
        this.movingBuffer = new CircularBuffer(potAndEncoderConfig.movingBufferMaxSize);
        this.absRelDifferenceBuffer = new CircularBuffer(potAndEncoderConfig.averagingBufferMaxSize);
        this.potDifferenceBuffer = new CircularBuffer(potAndEncoderConfig.averagingBufferMaxSize);
    }

    private boolean calibrated = false;
    private double firstOffset;
    private double offset;

    public void reset()
    {
        absUnwrapper.reset();
        relUnwrapper.reset();

        movingBuffer.clear();
        absRelDifferenceBuffer.clear();
        potDifferenceBuffer.clear();

        calibrated = false;
    }

    public Status exportToTable(LogTable table, String name)
    {
        Reading reading = config.HAL.getReading();
        reading.exportToTable(table, name);
        return update(reading);
    }

    public Status importFromTable(LogTable table, String name, Reading defaultReading)
    {
        return update(defaultReading.importFromTable(table, name));
    }

    public Status update() {
        if(config.HAL != null)
            return update(config.HAL.getReading());
        throw new NullPointerException("Called a PotAndEncoder auto update without setting a HAL");
    }
    public Status update(Reading reading)
    {
        double averageAbsRelDifference = 0;
        double averagePotDifference = 0;

        double absAngleDegEstimate = 0;
        double absAngleDegEstimateAtCalib = 0;
        double absAngleNumRotationsSinceCalib = 0;

        boolean error = false;
        boolean moving = true;

        boolean offsetReady = false;

        // unwrapped, scaled to output shaft angle
        double potAngleDeg = reading.potAngleDeg / config.potentiometerGearRatio;
        double absAngleDeg = absUnwrapper.unwrap(reading.absAngleDeg) / config.encoderGearRatio;      
        double relAngleDeg = relUnwrapper.unwrap(reading.relAngleDeg) / config.encoderGearRatio;

        // check to see if we are moving
        movingBuffer.addFirst( relAngleDeg );
        if (movingBuffer.size() == config.movingBufferMaxSize)
        {
            double maxVal = Double.NEGATIVE_INFINITY;
            double minVal = Double.POSITIVE_INFINITY;
            for (int k=0; k<config.movingBufferMaxSize; k++)
            {
                double val = movingBuffer.get(k);
                maxVal = Math.max(val, maxVal);
                minVal = Math.min(val, minVal);
            }
            moving = ((maxVal - minVal) > movingThreshold);
        }
        
        if (moving)
        {
            // start averaging over again
            absRelDifferenceBuffer.clear();
            potDifferenceBuffer.clear();
        }
        else 
        {
            absRelDifferenceBuffer.addFirst( absAngleDeg - relAngleDeg );
            potDifferenceBuffer.addFirst( potAngleDeg - config.potentiometerAngleDegAtCalib / config.potentiometerGearRatio);

            offsetReady = (potDifferenceBuffer.size() == config.averagingBufferMaxSize);

            if (offsetReady)
            {
                // calculate the average difference between the absolute and relative encoder values
                // and the average difference between the current potentiometer and that at calibration
                averageAbsRelDifference = 0;
                averagePotDifference = 0;
                for (int k=0; k<config.averagingBufferMaxSize; k++)
                {
                    averageAbsRelDifference += absRelDifferenceBuffer.get(k);
                    averagePotDifference += potDifferenceBuffer.get(k);
                } 
                averageAbsRelDifference = averageAbsRelDifference / config.averagingBufferMaxSize;
                averagePotDifference = averagePotDifference / config.averagingBufferMaxSize;
                
                absAngleDegEstimate = relAngleDeg + averageAbsRelDifference;
                absAngleDegEstimateAtCalib = absAngleDegEstimate - averagePotDifference;
                absAngleNumRotationsSinceCalib = (absAngleDegEstimateAtCalib - config.absoluteEncoderAngleDegAtCalib / config.encoderGearRatio) / (360.0 / config.encoderGearRatio);

                offset = averageAbsRelDifference - (360.0 * absAngleNumRotationsSinceCalib) / config.encoderGearRatio 
                        - config.absoluteEncoderAngleDegAtCalib / config.encoderGearRatio + config.outputAngleDegAtCalibration;

                if (!calibrated)
                {
                    calibrated = true;
                    firstOffset = offset;
                }

                error = (Math.abs(offset - firstOffset) > kAllowableErrorInOutputAngleDeg);
            }
        }

        double position = relAngleDeg + offset;

        return new Status(position, calibrated, moving, reading, config,
                new Debug(
                    averageAbsRelDifference, 
                    averagePotDifference, 
                    absAngleDegEstimate, 
                    absAngleDegEstimateAtCalib, 
                    absAngleNumRotationsSinceCalib, 
                    error, 
                    offsetReady, 
                    offset, 
                    firstOffset));
    }
    public static class Status {

        public final Config config;
        public final Debug debug;
        public final Reading reading;
        public final boolean moving;
        public final boolean calibrated;
        public final double positionDeg;
    
        protected Status(double positionDeg, boolean calibrated, boolean moving, Reading reading, Config config, Debug debug)
        {
            this.positionDeg = positionDeg;
            this.calibrated = calibrated;
            this.moving = moving;
            this.reading = reading;
            this.config = config;
            this.debug = debug;
        }

        public void recordOutputs(Logger logger, String prefix)
        {
            logger.recordOutput(prefix + "/Position (Deg)", positionDeg);
            logger.recordOutput(prefix + "/Calibrated", calibrated);
            logger.recordOutput(prefix + "/Moving", moving);
            reading.recordOutputs(logger, prefix + "/Reading");
        }
    }

    public static class Debug {
        public final double averageAbsRelDifference;
        public final double averagePotDifference;
    
        public final double absAngleDegEstimate;
        public final double absAngleDegEstimateAtCalib;
        public final double absAngleNumRotationsSinceCalib;
    
        public final boolean error;
        
        public final boolean offsetReady;
        public final double offset;
        public final double firstOffset;
    
        protected Debug(double averageAbsRelDifference,
                        double averagePotDifference,
                        double absAngleDegEstimate,
                        double absAngleDegEstimateAtCalib,
                        double absAngleNumRotationsSinceCalib,
                        boolean error,
                        boolean offsetReady,
                        double offset,
                        double firstOffset)
        {
            this.averageAbsRelDifference = averageAbsRelDifference;
            this.averagePotDifference = averagePotDifference;
            this.absAngleDegEstimate = absAngleDegEstimate;
            this.absAngleDegEstimateAtCalib = absAngleDegEstimateAtCalib;
            this.absAngleNumRotationsSinceCalib = absAngleNumRotationsSinceCalib;
            this.error = error;
            this.offsetReady = offsetReady;
            this.offset = offset;
            this.firstOffset = firstOffset;
        }
    }

    public static class HAL {

        AnalogPotentiometer pot;
        CANCoder enc;
    
        public HAL(int potAnalogInputPort, int encPort, double potentiometerNTurns, double potentiometerAngleDegAtCalib, double outputAngleDegAtCalibration)
        {
            pot = new AnalogPotentiometer(new AnalogInput(potAnalogInputPort), potentiometerNTurns*360.0, potentiometerAngleDegAtCalib - outputAngleDegAtCalibration);
            enc = new CANCoder(encPort);
            CANCoderConfiguration canConfig = new CANCoderConfiguration();
            enc.configAllSettings(canConfig);  // configure to default settings
        }
    
        public double getPotentiometerReadingDeg()      {return pot.get();}
        public double getAbsoluteEncoderReadingDeg()    {return enc.getAbsolutePosition();}
        public double getRelativeEncoderReadingDeg()    {return enc.getPosition();}
        public Reading getReading()        {return new Reading(getPotentiometerReadingDeg(), getAbsoluteEncoderReadingDeg(), getRelativeEncoderReadingDeg());}
    }

    public static class Reading {
        public final double potAngleDeg;
        public final double absAngleDeg;
        public final double relAngleDeg;
    
        public Reading(double potAngleDeg, double absAngleDeg, double relAngleDeg)
        {
            this.potAngleDeg = potAngleDeg;
            this.absAngleDeg = absAngleDeg;
            this.relAngleDeg = relAngleDeg;        
        }

        public void recordOutputs(Logger logger, String prefix)
        {
            logger.recordOutput(prefix + "/Potentiometer Angle (Deg)", potAngleDeg);
            logger.recordOutput(prefix + "/Absolute Encoder Angle (Deg)", absAngleDeg);
            logger.recordOutput(prefix + "/Relative Angle (Deg)", relAngleDeg);
        }
        public void exportToTable(LogTable table, String name)
        {
            table.put(name + "/Potentiometer Angle (Deg)", potAngleDeg);
            table.put(name + "/Absolute Encoder Angle (Deg)", absAngleDeg);
            table.put(name + "/Relative Angle (Deg)", relAngleDeg);
        }
        public Reading importFromTable(LogTable table, String name)
        {
            return new Reading(table.getDouble(name + "/Potentiometer Angle (Deg)", potAngleDeg),
                               table.getDouble(name + "/Absolute Encoder Angle (Deg)", absAngleDeg),
                               table.getDouble(name + "/Relative Angle (Deg)", relAngleDeg));
        }
        public static Reading importFromTable(LogTable table, String name, Reading defaultReading)
        {
            return new Reading(table.getDouble(name + "/Potentiometer Angle (Deg)", defaultReading.potAngleDeg),
                               table.getDouble(name + "/Absolute Encoder Angle (Deg)", defaultReading.absAngleDeg),
                               table.getDouble(name + "/Relative Angle (Deg)", defaultReading.relAngleDeg));
        }
    }

    public static class Config {
        public final double potentiometerGearRatio;             // gear ratio: potentiometer rotations / output rotations
        public final double encoderGearRatio;                   // gear ratio: encoder rotations / output rotations
        public final double potentiometerNTurns;                // number of turns in potentiometers full range of motion
        public final double outputAngleDegAtCalibration;        // angle of output at calibration position, in degrees
        public final double potentiometerAngleDegAtCalib;       // potentiometer reading at calibration position, in degrees  
        public final double absoluteEncoderAngleDegAtCalib;     // absolute endoder reading at calibration position, in degrees
        public final HAL HAL;                      // hardware abstraction layer
        public final int movingBufferMaxSize;
        public final int averagingBufferMaxSize;
         
        public Config(double potentiometerGearRatio, double encoderGearRatio, double potentiometerNTurns, 
                double outputAngleDegAtCalibration, double potentiometerAngleDegAtCalib, double absoluteEncoderAngleDegAtCalib, HAL HAL) {
            this(potentiometerGearRatio, encoderGearRatio, potentiometerNTurns, outputAngleDegAtCalibration, potentiometerAngleDegAtCalib, absoluteEncoderAngleDegAtCalib, HAL,
            20, 200);
        }
        public Config(double potentiometerGearRatio, double encoderGearRatio, double potentiometerNTurns, 
                double outputAngleDegAtCalibration, double potentiometerAngleDegAtCalib, double absoluteEncoderAngleDegAtCalib,
                HAL HAL, int movingBufferMaxSize, int averagingBufferMaxSize) {
            this.potentiometerGearRatio = potentiometerGearRatio;
            this.encoderGearRatio = encoderGearRatio;
            this.potentiometerNTurns = potentiometerNTurns;
            this.outputAngleDegAtCalibration = outputAngleDegAtCalibration;
            this.potentiometerAngleDegAtCalib = potentiometerAngleDegAtCalib;
            this.absoluteEncoderAngleDegAtCalib = absoluteEncoderAngleDegAtCalib;
            this.HAL = HAL;
            this.movingBufferMaxSize = movingBufferMaxSize;
            this.averagingBufferMaxSize = averagingBufferMaxSize;
        }
    }
}
