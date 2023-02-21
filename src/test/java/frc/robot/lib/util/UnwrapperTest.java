package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

public class UnwrapperTest {
    
    private static final double kEps = 1e-9;

    @Test
    public void TestUnwrap360() {
        
        Unwrapper unwrapDeg = new Unwrapper(0.0, 360.0);
        
        assertEquals(  1.0, unwrapDeg.unwrap(  1.0), kEps);         // small step from initial value
        assertEquals(180.0, unwrapDeg.unwrap(180.0), kEps);     // just under 1/2 period
        assertEquals(270.0, unwrapDeg.unwrap(270.0), kEps);   
        assertEquals(359.0, unwrapDeg.unwrap(359.0), kEps);
        assertEquals(360.0, unwrapDeg.unwrap(  0.0), kEps);       // jump 359 --> 0
        assertEquals(359.0, unwrapDeg.unwrap(359.0), kEps);     // jump 0 --> 359
        assertEquals(360.0+ 90.0, unwrapDeg.unwrap( 90.0), kEps);
        assertEquals(360.0+179.0, unwrapDeg.unwrap(179.0), kEps);
        assertEquals(360.0+181.0, unwrapDeg.unwrap(181.0), kEps); 
        assertEquals(360.0+360.0, unwrapDeg.unwrap(0.0), kEps);             // jump 181 --> 0    
        assertEquals(360.0+181.0, unwrapDeg.unwrap(181.0), kEps);           // jump 0 --> 181
    }

}
