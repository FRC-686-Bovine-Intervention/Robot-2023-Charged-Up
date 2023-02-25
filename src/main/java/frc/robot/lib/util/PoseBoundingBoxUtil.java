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

        public RectangularBoundingBox(Translation2d corner1, Translation2d corner2)
        {
            setBottomLeftCorner(new Translation2d(Math.min(corner1.getX(), corner2.getX()), Math.min(corner1.getY(), corner2.getY())));
            setTopRightCorner(new Translation2d(Math.max(corner1.getX(), corner2.getX()), Math.max(corner1.getY(), corner2.getY())));
        }

        private Translation2d bottomLeftCorner;
        public Translation2d getBottomLeftCorner() {return bottomLeftCorner;}
        public RectangularBoundingBox setBottomLeftCorner(Translation2d bottomLeftCorner) {this.bottomLeftCorner = bottomLeftCorner; return this;}

        private Translation2d topRightCorner;
        public Translation2d getTopRightCorner() {return topRightCorner;}
        public RectangularBoundingBox setTopRightCorner(Translation2d topRightCorner) {this.topRightCorner = topRightCorner; return this;}

        @Override
        public boolean withinBounds(Translation2d pose) {
            return  (pose.getX() >= bottomLeftCorner.getX() && pose.getX() <= topRightCorner.getX())
                                                            &&
                    (pose.getY() >= bottomLeftCorner.getY() && pose.getY() <= topRightCorner.getY());
        }
    }
    public static class CompoundBoundingBox extends BoundingBox {

        private final ArrayList<BoundingBox> positiveBoxes = new ArrayList<BoundingBox>();
        private final ArrayList<BoundingBox> negativeBoxes = new ArrayList<BoundingBox>();

        public CompoundBoundingBox(BoundingBox... boxes)    {this(Arrays.asList(boxes));}
        public CompoundBoundingBox(List<BoundingBox> boxes) {this(boxes, null);}
        public CompoundBoundingBox(List<BoundingBox> positiveBoxes, List<BoundingBox> negativeBoxes)
        {
            if(positiveBoxes != null)
                this.positiveBoxes.addAll(positiveBoxes);
            if(negativeBoxes != null)
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
