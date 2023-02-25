package frc.robot;

import frc.robot.lib.util.PoseBoundingBoxUtil.CompoundBoundingBox;
import frc.robot.lib.util.PoseBoundingBoxUtil.RectangularBoundingBox;

public class FieldDimensions {
    private static final RectangularBoundingBox community1 = new RectangularBoundingBox();
    private static final RectangularBoundingBox community2 = new RectangularBoundingBox();
    public static final RectangularBoundingBox chargeStation = new RectangularBoundingBox();

    public static final CompoundBoundingBox community = new CompoundBoundingBox(community1, community2);
    public static final CompoundBoundingBox communityWithoutChargeStation = new CompoundBoundingBox(community1, community2).addNegativeBoxes(chargeStation);
}
