package frc.robot.lib.sensorCalibration;

import edu.wpi.first.util.CircularBuffer;
import frc.robot.lib.util.Unwrapper;

public class PotAndEncoderCalilbration {

    private PotAndEncoderConfig potAndEncoderConfig;

    private Unwrapper absUnwrapper = new Unwrapper(0.0, 360.0);
    private Unwrapper relUnwrapper = new Unwrapper(0.0, 360.0);

    private double potAngleDeg; // unwrapped, scaled to output shaft angle
    private double absAngleDeg; // unwrapped, scaled to output shaft angle
    private double relAngleDeg; // unwrapped, scaled to output shaft angle

    private final int movingBufferMaxSize = 20;
    private int movingBufferSize = 0;
    private CircularBuffer movingBuffer = new CircularBuffer(movingBufferMaxSize);
    private final double movingThreshold = 2.0 / 4096.0;
    private boolean moving;

    private final int averagingBufferMaxSize = 200;
    private int averagingBufferSize = 0;
    private CircularBuffer absRelDifferenceBuffer = new CircularBuffer(averagingBufferMaxSize);
    private CircularBuffer potDifferenceBuffer = new CircularBuffer(averagingBufferMaxSize);

    private double averageAbsRelDifference;
    private double averagePotDifference;

    private double absAngleDegEstimate;
    private double absAngleDegEstimateAtCalib;
    private double absAngleNumRotationsSinceCalib;
    
    private boolean offsetReady = false;
    private double offset;
    private double firstOffset;

    private boolean error;
    private double allowableErrorInOutputAngleDeg;

    private boolean calibrated = false;
    private double position;

    public boolean isMoving() {return moving;}
    public boolean isError() {return error;}
    public boolean isCalibrated() {return calibrated;}
    public double getPosition() {return position;}

    

    public int getMovingBufferMaxSize() {
        return movingBufferMaxSize;
    }
    public int getAveragingBufferMaxSize() {
        return averagingBufferMaxSize;
    }
    public double getPotAngleDeg() {
        return potAngleDeg;
    }
    public double getAbsAngleDeg() {
        return absAngleDeg;
    }
    public double getRelAngleDeg() {
        return relAngleDeg;
    }
    public double getAverageAbsRelDifference() {
        return averageAbsRelDifference;
    }
    public double getAveragePotDifference() {
        return averagePotDifference;
    }
    public double getAbsAngleDegEstimate() {
        return absAngleDegEstimate;
    }
    public double getAbsAngleDegEstimateAtCalib() {
        return absAngleDegEstimateAtCalib;
    }
    public double getAbsAngleNumRotationsSinceCalib() {
        return absAngleNumRotationsSinceCalib;
    }
    public boolean isOffsetReady() {
        return offsetReady;
    }
    public double getOffset() {
        return offset;
    }
    public double getFirstOffset() {
        return firstOffset;
    }





    public PotAndEncoderCalilbration(PotAndEncoderConfig potAndEncoderConfig)
    {
        this.potAndEncoderConfig = potAndEncoderConfig;
        this.allowableErrorInOutputAngleDeg = 0.9 * 360.0 / potAndEncoderConfig.encoderGearRatio;
    }

    public void reset()
    {
        offsetReady = false;
        
        absUnwrapper.reset();
        relUnwrapper.reset();

        movingBufferSize = 0;
        averagingBufferSize = 0;

        error = false;
        calibrated = false;
    }


    public double update(PotAndEncoderReading reading)
    {
        offsetReady = false;

        potAngleDeg = reading.potAngleDeg / potAndEncoderConfig.potentiometerGearRatio;
        absAngleDeg = absUnwrapper.unwrap(reading.absAngleDeg) / potAndEncoderConfig.encoderGearRatio;      
        relAngleDeg = relUnwrapper.unwrap(reading.relAngleDeg) / potAndEncoderConfig.encoderGearRatio;

        // check to see if we are moving
        moving = true;
        movingBuffer.addFirst( relAngleDeg );
        movingBufferSize = Math.min(movingBufferSize+1, movingBufferMaxSize);
        if (movingBufferSize == movingBufferMaxSize)
        {
            double maxVal = Double.NEGATIVE_INFINITY;
            double minVal = Double.POSITIVE_INFINITY;
            for (int k=0; k<movingBufferMaxSize; k++)
            {
                double val = movingBuffer.get(k);
                maxVal = Math.max(val, maxVal);
                minVal = Math.min(val, minVal);
            }
            moving = ((maxVal - minVal) > movingThreshold);
        }
        
        if (moving)
        {
            averagingBufferSize = 0;      // start averaging over again        
        }
        else 
        {
            absRelDifferenceBuffer.addFirst( absAngleDeg - relAngleDeg );
            potDifferenceBuffer.addFirst( potAngleDeg - potAndEncoderConfig.potentiometerAngleDegAtCalib / potAndEncoderConfig.potentiometerGearRatio);
            averagingBufferSize = Math.min(averagingBufferSize+1, averagingBufferMaxSize);

            offsetReady = (averagingBufferSize == averagingBufferMaxSize);

            if (offsetReady)
            {
                // calculate the average difference between the absolute and relative encoder values
                // and the average difference between the current potentiometer and that at calibration
                averageAbsRelDifference = 0;
                averagePotDifference = 0;
                for (int k=0; k<averagingBufferMaxSize; k++)
                {
                    averageAbsRelDifference += absRelDifferenceBuffer.get(k);
                    averagePotDifference += potDifferenceBuffer.get(k);
                } 
                averageAbsRelDifference = averageAbsRelDifference / averagingBufferMaxSize;
                averagePotDifference = averagePotDifference / averagingBufferMaxSize;
                
                absAngleDegEstimate = relAngleDeg + averageAbsRelDifference;
                absAngleDegEstimateAtCalib = absAngleDegEstimate - averagePotDifference;
                absAngleNumRotationsSinceCalib = (absAngleDegEstimateAtCalib - potAndEncoderConfig.absoluteEncoderAngleDegAtCalib / potAndEncoderConfig.encoderGearRatio) / (360.0 / potAndEncoderConfig.encoderGearRatio);

                offset = averageAbsRelDifference - (360.0 * absAngleNumRotationsSinceCalib) / potAndEncoderConfig.encoderGearRatio 
                        - potAndEncoderConfig.absoluteEncoderAngleDegAtCalib / potAndEncoderConfig.encoderGearRatio + potAndEncoderConfig.outputAngleDegAtCalibration;

                if (!calibrated)
                {
                    calibrated = true;
                    firstOffset = offset;
                }

                error = (Math.abs(offset - firstOffset) > allowableErrorInOutputAngleDeg);
            }
        }

        position = relAngleDeg + offset;

        return position;
    }
}
