package frc.robot.lib.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PoseBoundingBoxUtil {
    public abstract static class BoundingBox {
        public final boolean withinBounds(Pose2d pose) {return withinBounds(pose.getTranslation());}
        public abstract boolean withinBounds(Translation2d pose);
    }
    public static class RectangularBoundingBox extends BoundingBox {

        public RectangularBoundingBox(Translation2d topLeftCorner, Translation2d bottomRightCorner)
        {
            setTopLeftCorner(topLeftCorner);
            setBottomRightCorner(bottomRightCorner);
        }

        private Translation2d topLeftCorner;
        public Translation2d getTopLeftCorner() {return topLeftCorner;}
        public RectangularBoundingBox setTopLeftCorner(Translation2d topLeftCorner) {this.topLeftCorner = topLeftCorner; return this;}

        private Translation2d bottomRightCorner;
        public Translation2d getBottomRightCorner() {return bottomRightCorner;}
        public RectangularBoundingBox setBottomRightCorner(Translation2d bottomRightCorner) {this.bottomRightCorner = bottomRightCorner; return this;}

        @Override
        public boolean withinBounds(Translation2d pose) {
            return  (pose.getX() >= bottomRightCorner.getX() && pose.getX() <= topLeftCorner.getX())
                                                             &&
                    (pose.getY() >= bottomRightCorner.getY() && pose.getY() <= topLeftCorner.getY());
        }
    }
    public static class CompoundBoundingBox extends BoundingBox {

        private final ArrayList<BoundingBox> positiveBoxes = new ArrayList<BoundingBox>();
        private final ArrayList<BoundingBox> negativeBoxes = new ArrayList<BoundingBox>();

        public CompoundBoundingBox(BoundingBox... boxes)    {this(Arrays.asList(boxes));}
        public CompoundBoundingBox(List<BoundingBox> boxes) {this(boxes, null);}
        public CompoundBoundingBox(List<BoundingBox> positiveBoxes, List<BoundingBox> negativeBoxes)
        {
            this.positiveBoxes.addAll(positiveBoxes);
            this.negativeBoxes.addAll(negativeBoxes);
        }
        
        public CompoundBoundingBox addPositiveBoxes(BoundingBox... boxes)   {return addPositiveBoxes(Arrays.asList(boxes));}
        public CompoundBoundingBox addPositiveBoxes(List<BoundingBox> boxes)
        {
            this.positiveBoxes.addAll(boxes);
            return this;
        }

        public CompoundBoundingBox addNegativeBoxes(BoundingBox... boxes)   {return addNegativeBoxes(Arrays.asList(boxes));}
        public CompoundBoundingBox addNegativeBoxes(List<BoundingBox> boxes)
        {
            this.negativeBoxes.addAll(boxes);
            return this;
        }

        @Override
        public boolean withinBounds(Translation2d pose) {
            boolean r = false;
            for (BoundingBox posBox : positiveBoxes) {
                if(posBox.withinBounds(pose))
                {
                    r = true;
                    break;
                }
            }
            for(BoundingBox negBox : negativeBoxes) {
                if(negBox.withinBounds(pose))
                {
                    r = false;
                    break;
                }
            }
            return r;
        }
    }
}
