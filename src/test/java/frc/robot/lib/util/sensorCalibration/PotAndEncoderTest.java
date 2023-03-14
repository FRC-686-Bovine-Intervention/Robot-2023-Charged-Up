package frc.robot.lib.util.sensorCalibration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;
import frc.robot.lib.sensorCalibration.PotAndEncoder;

public class PotAndEncoderTest {
    
    static final double kEps = 1e-9;

    double potentiometerGearRatio           = 4.0;
    double encoderGearRatio                 = 4.0;
    double potentiometerNTurns              = 5.0;    
    double outputAngleDegAtCalibration         = -90.0;      
    double potentiometerNormalizedVoltageAtCalib     = 0.4467;     
    double absoluteEncoderAngleDegAtCalib   = 87.5;     
    boolean potInverted = false;
    boolean encInverted = false;
    
    final PotAndEncoder.HAL hal = mock(PotAndEncoder.HAL.class);

    final PotAndEncoder.Config config = new PotAndEncoder.Config(potentiometerGearRatio, encoderGearRatio, potentiometerNTurns, 
                outputAngleDegAtCalibration, potentiometerNormalizedVoltageAtCalib, absoluteEncoderAngleDegAtCalib, 
                potInverted, encInverted, hal);
    
    // simulate hardware inputs
    PotAndEncoder.Reading mockHALReading(double outputAngleDeg)
    {
        double potAngleDeg = potentiometerGearRatio * outputAngleDeg;
        double encAngleDeg = encoderGearRatio * outputAngleDeg;

        double potReadingDeg, absReadingDeg, relReadingDeg;
        
        potReadingDeg = potAngleDeg;

        absReadingDeg = encAngleDeg + (absoluteEncoderAngleDegAtCalib - encoderGearRatio*outputAngleDegAtCalibration);
        absReadingDeg = MathUtil.inputModulus(absReadingDeg, 0.0, 360.0);    // modulo 360

        relReadingDeg = encAngleDeg;
        relReadingDeg = MathUtil.inputModulus(relReadingDeg, 0.0, 360.0);    // modulo 360        

        return new PotAndEncoder.Reading(potReadingDeg, absReadingDeg, relReadingDeg);
    }

    double mockGetPotAngleFromVoltage(double voltage) {
        double potScale = potInverted ? -potentiometerNTurns*360.0 : potentiometerNTurns*360.0;
        double potOffset = potentiometerGearRatio*outputAngleDegAtCalibration - potScale*potentiometerNormalizedVoltageAtCalib;
        return potScale*voltage + potOffset;
    }

    @Test
    public void TestIsMoving() 
    {
        PotAndEncoder potEncoder = new PotAndEncoder(config);
        PotAndEncoder.Status status = null;

        // start out changing the angle every clock
        double outputAngleDeg = 0.0;
        for (int k=0; k<100; k++) {
            outputAngleDeg += 1.0;
            when(hal.getReading()).thenReturn(mockHALReading(outputAngleDeg));
     
            status = potEncoder.update();
            assertTrue(status.moving);
        }

        // should still be declared moving while moving buffer is filling
        outputAngleDeg += 1.0;
        when(hal.getReading()).thenReturn(mockHALReading(outputAngleDeg));  

        for (int k=0; k<config.movingBufferMaxSize-1; k++) {
            status = potEncoder.update();
            assertTrue(status.moving);
        }        

        // shortly thereafter it should detect movement stops
        for (int k=0; k<10; k++) {
            status = potEncoder.update();
        }               
        assertFalse(status.moving);

        // and immediately notice moving once the angle changes again
        for (int k=0; k<10; k++) {
            outputAngleDeg += 1.0;
            when(hal.getReading()).thenReturn(mockHALReading(outputAngleDeg));   
            
            status = potEncoder.update();
            assertTrue(status.moving);
        }
    
    }

    @Test
    public void TestIsCalibrated() 
    {
        PotAndEncoder potEncoder = new PotAndEncoder(config);
        PotAndEncoder.Status status = null;

        // initial movement
        double outputAngleDeg = 0.0;
        for (int k=0; k<100; k++) {
            outputAngleDeg += 1.0;
            when(hal.getReading()).thenReturn(mockHALReading(outputAngleDeg));
     
            status = potEncoder.update();
            assertFalse(status.calibrated);
        }

        // stopped moving, start filling averaging buffers
        outputAngleDeg += 1.0;
        for (int k=0; k<config.movingBufferMaxSize-1+config.averagingBufferMaxSize-1; k++) {
            when(hal.getReading()).thenReturn(mockHALReading(outputAngleDeg));
     
            status = potEncoder.update();
            assertFalse(status.calibrated);
        }        

        // should be calibrated shortly thereafter
        for (int k=0; k<10; k++) {
            status = potEncoder.update();
        }
        assertTrue(status.calibrated);

        // remains calibrated even after movement restarts
        for (int k=0; k<100; k++) {
            outputAngleDeg += 1.0;
            when(hal.getReading()).thenReturn(mockHALReading(outputAngleDeg));
     
            status = potEncoder.update();
            assertTrue(status.calibrated);
        }
    } 


    @Test
    public void TestCalibration() {
        // run a series of dwells, where we move for a while then sit still for long enough to (re-)calculate the offset
        // check the final results for accuracy
        
        when(hal.getPotAngleFromVoltage(potentiometerNormalizedVoltageAtCalib)).thenReturn(mockGetPotAngleFromVoltage(potentiometerNormalizedVoltageAtCalib));

        PotAndEncoder potEncoder = new PotAndEncoder(config);
        PotAndEncoder.Status status = null;

        final long movePeriod = 100;
        final long dwellPeriod = config.averagingBufferMaxSize + config.movingBufferMaxSize + 1;

        double dwellList[] = { -134.2847, 
            148.8153,
             47.6493,
           -144.8855,
            -79.7406,
             16.8773,
            164.7025,
            167.3599,
           -123.2593,
            169.4134,
            outputAngleDegAtCalibration};
        final int numDwells = 10;   // 1 less than the list above

        double outputAngleDeg = dwellList[1];

        for (int dwellCnt = 1; dwellCnt <= numDwells; dwellCnt++)
        {
            // move arm for movePeriod
            double deltaAngle = (dwellList[dwellCnt] - dwellList[dwellCnt-1]) / (double)movePeriod;
            for (int k=0; k<movePeriod; k++)
            {
                outputAngleDeg += deltaAngle;
                when(hal.getReading()).thenReturn(mockHALReading(outputAngleDeg));
     
                status = potEncoder.update();
                assertTrue(status.moving);
            }

            if (dwellCnt == 1) {
                assertFalse(status.calibrated);            
            } else {
                assertTrue(status.calibrated);            
            }

            // stop arm for dwellPeriod
            outputAngleDeg = dwellList[dwellCnt];
            when(hal.getReading()).thenReturn(mockHALReading(outputAngleDeg));
            for (int k=0; k<dwellPeriod; k++)
            {
                status = potEncoder.update();
            }
            assertFalse(status.moving);
            assertTrue(status.calibrated);
        
            assertEquals(outputAngleDeg, status.positionDeg, kEps);
        }

        // final output angle is at the calibration angle
        assertEquals(outputAngleDegAtCalibration, status.positionDeg, kEps);        
    }
}
