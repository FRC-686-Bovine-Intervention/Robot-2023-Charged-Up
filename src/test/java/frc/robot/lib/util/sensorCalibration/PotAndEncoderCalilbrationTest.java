package frc.robot.lib.util.sensorCalibration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.lib.sensorCalibration.PotAndEncoderCalilbration;
import frc.robot.lib.sensorCalibration.PotAndEncoderConfig;
import frc.robot.lib.sensorCalibration.PotAndEncoderDebug;
import frc.robot.lib.sensorCalibration.PotAndEncoderReading;

public class PotAndEncoderCalilbrationTest {
    
    static final double kEps = 1e-9;

    double potentiometerGearRatio           = 3.0;
    double encoderGearRatio                 = 5.0;
    double potentiometerNTurns              = 3.0;    
    double outputAngleAtCalibration         = -37.1;      
    double potentiometerAngleDegAtCalib     = 421.0840;     
    double absoluteEncoderAngleDegAtCalib   = 288.0176;     
    
    PotAndEncoderConfig config = new PotAndEncoderConfig(potentiometerGearRatio, encoderGearRatio, 
                potentiometerNTurns, outputAngleAtCalibration, potentiometerAngleDegAtCalib, absoluteEncoderAngleDegAtCalib);
    
    // simulate hardware inputs
    PotAndEncoderReading simulatePotAndEncoderReadings(double outputAngleDeg)
    {
        double potAngleDeg = potentiometerGearRatio * outputAngleDeg;
        double encAngleDeg = encoderGearRatio * outputAngleDeg;

        double potReadingDeg, absReadingDeg, relReadingDeg;
        
        potReadingDeg = potAngleDeg + (potentiometerAngleDegAtCalib - potentiometerGearRatio*outputAngleAtCalibration);
        potReadingDeg = Math.max(potReadingDeg, 0.0);
        potReadingDeg = Math.min(potReadingDeg, potentiometerNTurns*360.0);

        absReadingDeg = encAngleDeg + (absoluteEncoderAngleDegAtCalib - encoderGearRatio*outputAngleAtCalibration);
        absReadingDeg = absReadingDeg - 360.0 * Math.floor(absReadingDeg / 360.0);    // modulo 360

        relReadingDeg = encAngleDeg;
        relReadingDeg = relReadingDeg - 360.0 * Math.floor(relReadingDeg / 360.0);    // modulo 360        

        return new PotAndEncoderReading(potReadingDeg, absReadingDeg, relReadingDeg);
    }

    @Test
    public void TestIsMoving() 
    {
        PotAndEncoderCalilbration calib = new PotAndEncoderCalilbration(config);

        // start out changing the angle every clock
        double outputAngleDeg = 0.0;
        for (int k=0; k<100; k++) {
            outputAngleDeg += 1.0;
            PotAndEncoderReading reading = simulatePotAndEncoderReadings(outputAngleDeg);
            calib.update(reading);
            assertTrue(calib.isMoving());
        }

        // should still be declared moving while moving buffer is filling
        outputAngleDeg += 1.0;
        for (int k=0; k<calib.getDebug().getMovingBufferMaxSize()-1; k++) {
            PotAndEncoderReading reading = simulatePotAndEncoderReadings(outputAngleDeg);
            calib.update(reading);
            assertTrue(calib.isMoving());
        }        

        // shortly thereafter it should detect movement stops
        for (int k=0; k<10; k++) {
            PotAndEncoderReading reading = simulatePotAndEncoderReadings(outputAngleDeg);
            calib.update(reading);
        }               
        assertFalse(calib.isMoving());

        // and immediately notice moving once the angle changes again
        for (int k=0; k<10; k++) {
            outputAngleDeg += 1.0;
            PotAndEncoderReading reading = simulatePotAndEncoderReadings(outputAngleDeg);
            calib.update(reading);
            assertTrue(calib.isMoving());
        }
    
    }

    @Test
    public void TestIsCalibrated() 
    {
        PotAndEncoderCalilbration calib = new PotAndEncoderCalilbration(config);

        // initial movement
        double outputAngleDeg = 0.0;
        for (int k=0; k<100; k++) {
            outputAngleDeg += 1.0;
            PotAndEncoderReading reading = simulatePotAndEncoderReadings(outputAngleDeg);
            calib.update(reading);
            assertFalse(calib.isCalibrated());
        }

        // stopped moving, start filling averaging buffers
        outputAngleDeg += 1.0;
        for (int k=0; k<calib.getDebug().getMovingBufferMaxSize()-1+calib.getDebug().getAveragingBufferMaxSize()-1; k++) {
            PotAndEncoderReading reading = simulatePotAndEncoderReadings(outputAngleDeg);
            calib.update(reading);
            assertFalse(calib.isCalibrated());
        }        

        // should be calibrated shortly thereafter
        for (int k=0; k<10; k++) {
            PotAndEncoderReading reading = simulatePotAndEncoderReadings(outputAngleDeg);
            calib.update(reading);
        }
        assertTrue(calib.isCalibrated());

        // remains calibrated even after movement restarts
        for (int k=0; k<100; k++) {
            outputAngleDeg += 1.0;
            PotAndEncoderReading reading = simulatePotAndEncoderReadings(outputAngleDeg);
            calib.update(reading);
            assertTrue(calib.isCalibrated());
        }
    } 


    @Test
    public void TestCalibration() {
        // run a series of dwells, where we move for a while then sit still for long enought to (re-)calculate the offset
        // check the final results for accuracy
        
        PotAndEncoderCalilbration calib = new PotAndEncoderCalilbration(config);

        final long movePeriod = 100;
        final long dwellPeriod = Math.round(calib.getDebug().getAveragingBufferMaxSize() * 1.25);

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
            outputAngleAtCalibration};
        final int numDwells = 10;   // 1 less than the list above

        double outputAngleDeg = dwellList[1];

        for (int dwellCnt = 1; dwellCnt <= numDwells; dwellCnt++)
        {
            double deltaAngle = (dwellList[dwellCnt] - dwellList[dwellCnt-1]) / (double)movePeriod;
            for (int k=0; k<movePeriod; k++)
            {
                outputAngleDeg += deltaAngle;
                PotAndEncoderReading reading = simulatePotAndEncoderReadings(outputAngleDeg);
                calib.update(reading);
                assertTrue(calib.isMoving());
            }

            if (dwellCnt == 1) {
                assertFalse(calib.isCalibrated());            
            } else {
                assertTrue(calib.isCalibrated());            
            }

            outputAngleDeg = dwellList[dwellCnt];
            for (int k=0; k<dwellPeriod; k++)
            {
                PotAndEncoderReading reading = simulatePotAndEncoderReadings(outputAngleDeg);
                calib.update(reading);
            }
            assertFalse(calib.isMoving());
            assertTrue(calib.isCalibrated());

            // PotAndEncoderDebug debug = calib.getDebug();
            // System.out.printf("outAngleDeg = %.3f\n", calib.outputAngleDeg);
            // System.out.printf("potAngleDeg = %.3f\n", calib.getPotAngleDeg());
            // System.out.printf("absAngleDeg = %.3f\n", calib.getAbsAngleDeg());
            // System.out.printf("relAngleDeg = %.3f\n", calib.getRelAngleDeg());
            // System.out.printf("averageAbsRelDifference = %.3f\n", debug.getAverageAbsRelDifference());
            // System.out.printf("averagePotDifference = %.3f\n", debug.getAveragePotDifference());
            // System.out.printf("absAngleDegEstimate = %.3f\n", debug.getAbsAngleDegEstimate());
            // System.out.printf("absAngleDegEstimateAtCalib = %.3f\n", debug.getAbsAngleDegEstimateAtCalib());
            // System.out.printf("absAngleNumRotationsSinceCalib = %.3f\n", debug.getAbsAngleNumRotationsSinceCalib());
            // System.out.printf("offset = %.3f\n", debug.getOffset());
            // System.out.printf("dwell=%2d, actual=%.3f, calc=%.3f\n", dwellCnt, dwellList[dwellCnt], calib.getPosition());
        
            assertEquals(outputAngleDeg, calib.getPosition(), kEps);
        }

        // final output angle is at the calibration angle
        assertEquals(outputAngleDeg, outputAngleAtCalibration, kEps);        
    }
}
