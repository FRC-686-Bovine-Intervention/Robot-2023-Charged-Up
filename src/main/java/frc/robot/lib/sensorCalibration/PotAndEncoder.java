package frc.robot.lib.sensorCalibration;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.lib.util.Unwrapper;

public class PotAndEncoder {
    private final Config config;

    private final Unwrapper absUnwrapper = new Unwrapper(0.0, 360.0);
    private final Unwrapper relUnwrapper = new Unwrapper(0.0, 360.0);

    private static final double movingThreshold = 2.0 / 4096.0;
    private final CircularBuffer movingBuffer;
    private final CircularBuffer absRelDifferenceBuffer;
    private final CircularBuffer potDifferenceBuffer;

    private final double kAllowableErrorInOutputAngleDeg;
    private final double potentiometerAngleDegAtCalib;

    public PotAndEncoder(Config potAndEncoderConfig)
    {
        this.config = potAndEncoderConfig;
        this.kAllowableErrorInOutputAngleDeg = 0.9 * 360.0 / potAndEncoderConfig.encoderGearRatio;
        this.movingBuffer = new CircularBuffer(potAndEncoderConfig.movingBufferMaxSize);
        this.readyCounter = 0;
        this.absRelDifferenceBuffer = new CircularBuffer(potAndEncoderConfig.averagingBufferMaxSize);
        this.potDifferenceBuffer = new CircularBuffer(potAndEncoderConfig.averagingBufferMaxSize);
        this.potentiometerAngleDegAtCalib = potAndEncoderConfig.HAL.getPotAngleFromVoltage(potAndEncoderConfig.potentiometerNormalizedVoltageAtCalib); 
    }

    private int readyCounter = 0;
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

    public Reading getReading()
    {
        Reading r = new Reading(0, 0, 0);
        if(config.HAL != null)
            r = config.HAL.getReading();
        return r;
    }

    public Status update()
    {
        if(config.HAL != null)
            return update(config.HAL.getReading());
        throw new NullPointerException("Called PotAndEncoder auto update without setting a HAL");
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
        
        absRelDifferenceBuffer.addFirst( absAngleDeg - relAngleDeg );
        potDifferenceBuffer.addFirst( potAngleDeg - potentiometerAngleDegAtCalib / config.potentiometerGearRatio );

        // calculate the average difference between the absolute and relative encoder values
        // and the average difference between the current potentiometer and that at calibration
        averageAbsRelDifference = 0;
        averagePotDifference = 0;
        for (int k=0; k<absRelDifferenceBuffer.size(); k++)
        {
            averageAbsRelDifference += absRelDifferenceBuffer.get(k);
            averagePotDifference += potDifferenceBuffer.get(k);
        } 
        averageAbsRelDifference = averageAbsRelDifference / (double)absRelDifferenceBuffer.size();
        averagePotDifference = averagePotDifference / (double)potDifferenceBuffer.size();
        
        absAngleDegEstimate = relAngleDeg + averageAbsRelDifference;
        absAngleDegEstimateAtCalib = absAngleDegEstimate - averagePotDifference;
        absAngleNumRotationsSinceCalib = (absAngleDegEstimateAtCalib - config.absoluteEncoderAngleDegAtCalib / config.encoderGearRatio) / (360.0 / config.encoderGearRatio);

        readyCounter = moving ? 0 : Math.min(readyCounter+1, config.averagingBufferMaxSize);
        offsetReady = (readyCounter >= config.averagingBufferMaxSize);
        if (offsetReady)
        {
            offset = averageAbsRelDifference - (360.0 * absAngleNumRotationsSinceCalib) / config.encoderGearRatio 
                - config.absoluteEncoderAngleDegAtCalib / config.encoderGearRatio + config.outputAngleDegAtCalibration;

            if (!calibrated)
            {
                calibrated = true;
                firstOffset = offset;
            }

            error = (Math.abs(offset - firstOffset) > kAllowableErrorInOutputAngleDeg);
        }

        double position = relAngleDeg + offset;

        return new Status(position, calibrated, moving, reading, config,
                new Debug(
                    potAngleDeg,
                    absAngleDeg,
                    relAngleDeg,
                    averageAbsRelDifference, 
                    averagePotDifference, 
                    absAngleDegEstimate, 
                    absAngleDegEstimateAtCalib, 
                    absAngleNumRotationsSinceCalib, 
                    error, 
                    readyCounter,
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
            config.recordOutputs(logger, prefix + "/Config");
            reading.recordOutputs(logger, prefix + "/Reading");
            debug.recordOutputs(logger, prefix + "/Debug");
        }
    }


    public static class Debug {
        public final double potAngleDeg;
        public final double absAngleDeg;
        public final double relAngleDeg;

        public final double averageAbsRelDifference;
        public final double averagePotDifference;
    
        public final double absAngleDegEstimate;
        public final double absAngleDegEstimateAtCalib;
        public final double absAngleNumRotationsSinceCalib;
    
        public final boolean error;
        
        public final int readyCounter;
        public final boolean offsetReady;
        public final double offset;
        public final double firstOffset;
    
        protected Debug(double potAngleDeg,
                        double absAngleDeg,
                        double relAngleDeg,
                        double averageAbsRelDifference,
                        double averagePotDifference,
                        double absAngleDegEstimate,
                        double absAngleDegEstimateAtCalib,
                        double absAngleNumRotationsSinceCalib,
                        boolean error,
                        int readyCounter,
                        boolean offsetReady,
                        double offset,
                        double firstOffset)
        {
            this.potAngleDeg = potAngleDeg;
            this.absAngleDeg = absAngleDeg;
            this.relAngleDeg = relAngleDeg;
            this.averageAbsRelDifference = averageAbsRelDifference;
            this.averagePotDifference = averagePotDifference;
            this.absAngleDegEstimate = absAngleDegEstimate;
            this.absAngleDegEstimateAtCalib = absAngleDegEstimateAtCalib;
            this.absAngleNumRotationsSinceCalib = absAngleNumRotationsSinceCalib;
            this.error = error;
            this.readyCounter = readyCounter;
            this.offsetReady = offsetReady;
            this.offset = offset;
            this.firstOffset = firstOffset;
        }

        public void recordOutputs(Logger logger, String prefix)
        {        
            logger.recordOutput(prefix + "/potAngleDeg", potAngleDeg);
            logger.recordOutput(prefix + "/absAngleDeg", absAngleDeg);
            logger.recordOutput(prefix + "/relAngleDeg", relAngleDeg);
            logger.recordOutput(prefix + "/averageAbsRelDifference", averageAbsRelDifference);
            logger.recordOutput(prefix + "/averagePotDifference", averagePotDifference);
            logger.recordOutput(prefix + "/absAngleDegEstimate", absAngleDegEstimate);
            logger.recordOutput(prefix + "/absAngleDegEstimateAtCalib", absAngleDegEstimateAtCalib);
            logger.recordOutput(prefix + "/absAngleNumRotationsSinceCalib", absAngleNumRotationsSinceCalib);
            logger.recordOutput(prefix + "/error", error);
            logger.recordOutput(prefix + "/readyCounter", readyCounter);
            logger.recordOutput(prefix + "/offsetReady", offsetReady);
            logger.recordOutput(prefix + "/offset", offset);
            logger.recordOutput(prefix + "/firstOffset", firstOffset);      
        }  
    }


    public static class HAL {

        AnalogPotentiometer pot;
        CANCoder enc;

        double potScale;
        double potOffset;
    
        public HAL(int potAnalogInputPort, int encPort, double potNTurns, double potGearRatio, double potNormalizedVoltageAtCalib, double outputAngleDegAtCalibration, boolean potInverted, boolean encInverted)
        {
            potScale = potInverted ? -potNTurns*360.0 : potNTurns*360.0;
            potOffset = potGearRatio*outputAngleDegAtCalibration - potScale*potNormalizedVoltageAtCalib;
            pot = new AnalogPotentiometer(potAnalogInputPort, potScale, potOffset);

            enc = new CANCoder(encPort);
            enc.configFactoryDefault();
            enc.configSensorDirection(encInverted);  // false: CCW, true: CW while looking from sensor to shaft
        }
    
        public double getPotentiometerReadingDeg()      {return pot.get();}
        public double getAbsoluteEncoderReadingDeg()    {return enc.getAbsolutePosition();}
        public double getRelativeEncoderReadingDeg()    {return enc.getPosition();}
        public Reading getReading()                     {return new Reading(getPotentiometerReadingDeg(), getAbsoluteEncoderReadingDeg(), getRelativeEncoderReadingDeg());}

        public double getPotAngleFromVoltage(double voltage) {
            return potScale*voltage + potOffset;
        }
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
    }

    
    public static class Config {
        public final double potentiometerGearRatio;             // gear ratio: potentiometer rotations / output rotations
        public final double encoderGearRatio;                   // gear ratio: encoder rotations / output rotations
        public final double potentiometerNTurns;                // number of turns in potentiometers full range of motion

        // Pot & Encoder Calibration Setup:
        //      A calibration project is created, with
        //          AnalogPotentiometer pot = new AnalogPotentiometer(potPort); // using default scale = 1.0, offset = 0.0
        //          CANCoder enc = new CANCoder(encPort);
        //          enc.configFactoryDefault();  // using default CCW positive polarity (as viewed from LED)
        //          (Readings from pot & encoder can be averaged to reduce noisiness)
        //      Reference angle is chosen (could be horizontal, vertical, or other)
        //      Joint is set to the calibration angle

        // Pot and Encoder Calibration values are recorded: 
        //      outputAngleDegAtCalibration is recorded as the angle of the joint with respect to the reference, in degrees
        //      potentiometerNormalizedVoltageAtCalib is recorded as the output of pot.get()
        //      absoluteEncoderAngleDegAtCalib is recorded at the output of enc.getAbsolutePosition()
        // No adjustments for polarity, gear ratio, or number of potentiometer turns are made during calibration
        public final double outputAngleDegAtCalibration;        // angle of output at calibration position, in degrees (relative to reference angle of choice)
        public final double potentiometerNormalizedVoltageAtCalib;       // potentiometer reading at calibration position, in degrees (measurement at pot, not modified by gear ratio) 
        public final double absoluteEncoderAngleDegAtCalib;     // absolute endoder reading at calibration position, in degrees (measurement at encoder, not modified by gear ratio)
        
        // in operation, the polarity of the potentiometer and absolute encoder must be configured correctly
        public final boolean potInverted;                       // set true to invert polarity of potentiometer
        public final boolean encInverted;                       // false: CCW, true: CW while looking from sensor to shaft
        
        public final HAL HAL;                      // hardware abstraction layer
        public final int movingBufferMaxSize;
        public final int averagingBufferMaxSize;
         
        public Config(double potentiometerGearRatio, double encoderGearRatio, double potentiometerNTurns, 
                double outputAngleDegAtCalibration, double potentiometerNormalizedVoltageAtCalib, double absoluteEncoderAngleDegAtCalib, 
                boolean potInverted, boolean encInverted, HAL HAL) {
            this(potentiometerGearRatio, encoderGearRatio, potentiometerNTurns, outputAngleDegAtCalibration, potentiometerNormalizedVoltageAtCalib, absoluteEncoderAngleDegAtCalib, 
            potInverted, encInverted, HAL, 20, 200);
        }
        public Config(double potentiometerGearRatio, double encoderGearRatio, double potentiometerNTurns, 
                double outputAngleDegAtCalibration, double potentiometerNormalizedVoltageAtCalib, double absoluteEncoderAngleDegAtCalib,
                boolean potInverted, boolean encInverted, HAL HAL, int movingBufferMaxSize, int averagingBufferMaxSize) {
            this.potentiometerGearRatio = potentiometerGearRatio;
            this.encoderGearRatio = encoderGearRatio;
            this.potentiometerNTurns = potentiometerNTurns;
            this.outputAngleDegAtCalibration = outputAngleDegAtCalibration;
            this.potentiometerNormalizedVoltageAtCalib = potentiometerNormalizedVoltageAtCalib;
            this.absoluteEncoderAngleDegAtCalib = absoluteEncoderAngleDegAtCalib;
            this.HAL = HAL;
            this.potInverted = potInverted;
            this.encInverted = encInverted;
            this.movingBufferMaxSize = movingBufferMaxSize;
            this.averagingBufferMaxSize = averagingBufferMaxSize;
        }

        public void recordOutputs(Logger logger, String prefix)
        {        
            logger.recordOutput(prefix + "/potentiometerGearRatio", potentiometerGearRatio);
            logger.recordOutput(prefix + "/encoderGearRatio", encoderGearRatio);
            logger.recordOutput(prefix + "/potentiometerNTurns", potentiometerNTurns);
            logger.recordOutput(prefix + "/outputAngleDegAtCalibration", outputAngleDegAtCalibration);
            logger.recordOutput(prefix + "/potentiometerNormalizedVoltageAtCalib", potentiometerNormalizedVoltageAtCalib);
            logger.recordOutput(prefix + "/absoluteEncoderAngleDegAtCalib", absoluteEncoderAngleDegAtCalib);
            logger.recordOutput(prefix + "/potInverted", potInverted);
            logger.recordOutput(prefix + "/encInverted", encInverted);
            logger.recordOutput(prefix + "/movingBufferMaxSize", movingBufferMaxSize);
            logger.recordOutput(prefix + "/averagingBufferMaxSize", averagingBufferMaxSize);      
        }
    }
}

