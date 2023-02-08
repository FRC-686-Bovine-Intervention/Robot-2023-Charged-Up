package frc.robot.lib.sensorCalibration;

public class PotAndEncoderDebug {
    private final int movingBufferMaxSize;
    private final int averagingBufferMaxSize;
    private final double averageAbsRelDifference;
    private final double averagePotDifference;

    private final double absAngleDegEstimate;
    private final double absAngleDegEstimateAtCalib;
    private final double absAngleNumRotationsSinceCalib;
    
    private final boolean offsetReady;
    private final double offset;
    private final double firstOffset;

    protected PotAndEncoderDebug(int movingBufferMaxSize,
                                 int averagingBufferMaxSize,
                                 double averageAbsRelDifference,
                                 double averagePotDifference,
                                 double absAngleDegEstimate,
                                 double absAngleDegEstimateAtCalib,
                                 double absAngleNumRotationsSinceCalib,
                                 boolean offsetReady,
                                 double offset,
                                 double firstOffset)
    {
        this.movingBufferMaxSize = movingBufferMaxSize;
        this.averagingBufferMaxSize = averagingBufferMaxSize;
        this.averageAbsRelDifference = averageAbsRelDifference;
        this.averagePotDifference = averagePotDifference;
        this.absAngleDegEstimate = absAngleDegEstimate;
        this.absAngleDegEstimateAtCalib = absAngleDegEstimateAtCalib;
        this.absAngleNumRotationsSinceCalib = absAngleNumRotationsSinceCalib;
        this.offsetReady = offsetReady;
        this.offset = offset;
        this.firstOffset = firstOffset;
    }

    public int getMovingBufferMaxSize() {return movingBufferMaxSize;}
    public int getAveragingBufferMaxSize() {return averagingBufferMaxSize;}
    public double getAverageAbsRelDifference() {return averageAbsRelDifference;}
    public double getAveragePotDifference() {return averagePotDifference;}
    public double getAbsAngleDegEstimate() {return absAngleDegEstimate;}
    public double getAbsAngleDegEstimateAtCalib() {return absAngleDegEstimateAtCalib;}
    public double getAbsAngleNumRotationsSinceCalib() {return absAngleNumRotationsSinceCalib;}
    public boolean isOffsetReady() {return offsetReady;}
    public double getOffset() {return offset;}
    public double getFirstOffset() {return firstOffset;}
}
