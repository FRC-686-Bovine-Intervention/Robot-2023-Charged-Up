package frc.robot.lib.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.util.PoseBoundingBoxUtil.CompoundBoundingBox;
import frc.robot.lib.util.PoseBoundingBoxUtil.RectangularBoundingBox;

public class PoseBoundingBoxTest {
    @Test
    public void RectangularBoundingBoxTest()
    {
        RectangularBoundingBox box = 
            new RectangularBoundingBox(
                new Translation2d(1,1),
                new Translation2d(2,2)
            );
        Translation2d botLeft =     new Translation2d(0.5,0.5);
        Translation2d botCent =     new Translation2d(1.5,0.5);
        Translation2d botRight =    new Translation2d(2.5,0.5);
        Translation2d cenLeft =     new Translation2d(0.5,1.5);
        Translation2d cenCent =     new Translation2d(1.5,1.5);
        Translation2d cenRight =    new Translation2d(2.5,1.5);
        Translation2d topLeft =     new Translation2d(0.5,2.5);
        Translation2d topCent =     new Translation2d(1.5,2.5);
        Translation2d topRight =    new Translation2d(2.5,2.5);

        Translation2d[] truePoints = new Translation2d[] {
            cenCent
        };

        Translation2d[] falsePoints = new Translation2d[] {
            botLeft,
            cenLeft,
            topLeft,
            topCent,
            topRight,
            cenRight,
            botRight,
            botCent
        };
        
        for (Translation2d pos : truePoints) {
            assertTrue(box.withinBounds(pos));
            assertTrue(box.withinBounds(new Pose2d(pos, new Rotation2d())));
        }

        for (Translation2d pos : falsePoints) {
            assertFalse(box.withinBounds(pos));
            assertFalse(box.withinBounds(new Pose2d(pos, new Rotation2d())));
        }
    }
    @Test
    public void CompoundBoundingBoxTest()
    {
        RectangularBoundingBox box1 = 
        new RectangularBoundingBox(
            new Translation2d(1,0),
            new Translation2d(2,3)
        );
        RectangularBoundingBox box2 = 
            new RectangularBoundingBox(
                new Translation2d(0,1),
                new Translation2d(3,2)
            );
        CompoundBoundingBox cbox = 
        new CompoundBoundingBox(
            box1,
            box2
        );

        Translation2d botLeft =     new Translation2d(0.5,0.5);
        Translation2d botCent =     new Translation2d(1.5,0.5);
        Translation2d botRight =    new Translation2d(2.5,0.5);
        Translation2d cenLeft =     new Translation2d(0.5,1.5);
        Translation2d cenCent =     new Translation2d(1.5,1.5);
        Translation2d cenRight =    new Translation2d(2.5,1.5);
        Translation2d topLeft =     new Translation2d(0.5,2.5);
        Translation2d topCent =     new Translation2d(1.5,2.5);
        Translation2d topRight =    new Translation2d(2.5,2.5);

        Translation2d[] truePoints = new Translation2d[] {
            botCent,
            cenLeft,
            cenCent,
            cenRight,
            topCent
        };

        Translation2d[] falsePoints = new Translation2d[] {
            botLeft,
            botRight,
            topLeft,
            topRight
        };
        
        for (Translation2d pos : truePoints) {
            assertTrue(cbox.withinBounds(pos));
            assertTrue(cbox.withinBounds(new Pose2d(pos, new Rotation2d())));
        }

        for (Translation2d pos : falsePoints) {
            assertFalse(cbox.withinBounds(pos));
            assertFalse(cbox.withinBounds(new Pose2d(pos, new Rotation2d())));
        }
    }
}
