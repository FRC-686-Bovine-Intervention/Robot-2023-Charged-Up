package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.AdvantageUtil;
import frc.robot.lib.sensorCalibration.PotAndEncoder;
import frc.robot.subsystems.arm.ArmPose.Preset;
import frc.robot.subsystems.framework.StatusBase;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.subsystems.odometry.OdometryStatus;

public class ArmStatus extends StatusBase {
    private static ArmStatus instance;
    public static ArmStatus getInstance(){if(instance == null) instance = new ArmStatus(); return instance;}

    private final Arm arm = Arm.getInstance();
    private final ArmHAL HAL = ArmHAL.getInstance();

    public enum ArmState {
        ZeroDistalUp,       // =============================================================================
        ZeroTurret,         // If arm isn't where it's supposed to be on enable,
        ZeroProximal,       // move arm to proper position
        ZeroDistal,         // =============================================================================
        Defense,            // Arm is idling, retracted fully and waiting for the intake to do something
        IdentifyPiece,      // Arm is identifying the piece in the intake
        Grab,               // Arm has identified the location of the piece and arm is grabbing the piece from the intake
        Hold,               // Arm has grabbed the piece and is holding it in the defense position
        Align,              // Robot has entered the community and the turret should align to the alliance wall
        Extend,             // Driver has selected a node to extend to and arm is extending to it
        Adjust,             // Driver and limelight are fudging the position of the turret to align piece on the node
        Release,            // Driver has decided the piece will score on the node and tells the arm to release the piece
        SubstationExtend,   // Driver has decided to grab a piece from the substation
        SubstationGrab;     // Driver is ready to grab piece from the substation

        public static final ArmState DEFAULT = ZeroDistalUp;
    }

    public enum NodeEnum {
        BottomLeft      (0,0,ArmPose.Preset.SCORE_HYBRID),
        BottomCenter    (1,0,ArmPose.Preset.SCORE_HYBRID),
        BottomRight     (2,0,ArmPose.Preset.SCORE_HYBRID),
        MiddleLeft      (0,1,ArmPose.Preset.SCORE_MID_CONE),
        MiddleCenter    (1,1,ArmPose.Preset.SCORE_MID_CUBE),
        MiddleRight     (2,1,ArmPose.Preset.SCORE_MID_CONE),
        TopLeft         (0,2,ArmPose.Preset.SCORE_HIGH_CONE),
        TopCenter       (1,2,ArmPose.Preset.SCORE_HIGH_CUBE),
        TopRight        (2,2,ArmPose.Preset.SCORE_HIGH_CONE);

        public static final NodeEnum DEFAULT = MiddleCenter;

        public final int xPos;
        public final int yPos;
        public final ArmPose.Preset armPreset;
        NodeEnum(int xPos, int yPos, ArmPose.Preset armPreset) {
            this.xPos = xPos;
            this.yPos = yPos;
            this.armPreset = armPreset;
        }
        public NodeEnum fromGridPose(int xPos, int yPos) {
            for (NodeEnum node : NodeEnum.values()) {
                if(node.xPos == xPos && node.yPos == yPos)
                    return node;
            }
            return null;
        }
    }

    private ArmStatus() {Subsystem = arm;}

    // Generic
    private ArmCommand      command = new ArmCommand();
    protected ArmCommand    getCommand()                    {return command;}
    private ArmStatus       setCommand(ArmCommand command)  {this.command = command; return this;}

    private ArmState    armState = ArmState.DEFAULT;
    public ArmState     getArmState()                   {return armState;}
    protected ArmStatus setArmState(ArmState armState)  {this.armState = armState; return this;}

    private NodeEnum    targetNode = NodeEnum.DEFAULT;
    public NodeEnum     getTargetNode()                     {return targetNode;}
    protected ArmStatus setTargetNode(NodeEnum targetNode)  {this.targetNode = targetNode; return this;}

    // Turret
    private double      turretPosition; 
    public double       getTurretPosition() {return turretPosition;}
    private ArmStatus   setTurretPosition(double turretPosition) {this.turretPosition = turretPosition; return this;}

    private double      targetTurretAngle;
    public double       getTargetTurretAngle()              {return targetTurretAngle;}
    protected ArmStatus setTargetTurretAngle(double angle)  {targetTurretAngle = angle; return this;}

    private static final Translation3d robotToTurretTranslation = new Translation3d(); //TODO

    public Transform3d  getRobotToTurret(){
        return new Transform3d(robotToTurretTranslation, new Rotation3d(0, 0, getTurretPosition()));
    };

    public Pose3d getTurretToField() {
        return new Pose3d(OdometryStatus.getInstance().getRobotPose())
                            .transformBy(getRobotToTurret());
    } 

    private double      turretPower;
    public double       getTurretPower()                    {return turretPower;}
    protected ArmStatus setTurretPower(double turretPower)  {this.turretPower = turretPower; return this;}

    public Transform3d  getTurretPose() {return 
        new Transform3d(
            new Translation3d(
                0,
                0,
                0
            ), 
            new Rotation3d(
                0, 
                0, 
                Units.degreesToRadians(getTurretPosition())
            )
        );
    }

    // Arm
    private ArmPose.Preset  targetArmPose = ArmPose.Preset.DEFENSE;
    public ArmPose.Preset   getTargetArmPose()                              {return targetArmPose;}
    protected ArmStatus     setTargetArmPose(ArmPose.Preset targetArmPose)  {this.targetArmPose = targetArmPose; return this;}

    private ArmPose.Preset  currentArmPose = null;
    public ArmPose.Preset   getCurrentArmPose()                                 {return currentArmPose;}
    protected ArmStatus     setCurrentArmPose(ArmPose.Preset currentArmPose)    {this.currentArmPose = currentArmPose; return this;}

    private ArmTrajectory   currentArmTrajectory = null;
    public ArmTrajectory    getCurrentArmTrajectory()                                   {return currentArmTrajectory;}
    protected ArmStatus     setCurrentArmTrajectory(ArmTrajectory currentArmTrajectory) {this.currentArmTrajectory = currentArmTrajectory; return this;}

    private Matrix<N2,N3>   currentTrajState = new MatBuilder<>(Nat.N2(),Nat.N3()).fill(0,0,0,0,0,0);
    public Matrix<N2,N3>    getCurrentTrajState()                                   {return currentTrajState;}
    protected ArmStatus     setCurrentTrajState(Matrix<N2, N3> currentTrajState)    {this.currentTrajState = currentTrajState; return this;}

    private Matrix<N2,N3>   setpointTrajState = new MatBuilder<>(Nat.N2(),Nat.N3()).fill(0,0,0,0,0,0);
    public Matrix<N2,N3>    getSetpointTrajState()                                  {return setpointTrajState;}
    protected ArmStatus     setSetpointTrajState(Matrix<N2, N3> setpointTrajState)  {this.setpointTrajState = setpointTrajState; return this;}

    private boolean         internalDisable = false;
    public boolean          getInternalDisable()                        {return internalDisable;}
    protected ArmStatus     setInternalDisable(boolean internalDisable) {/* this.internalDisable = internalDisable; */ return this;}

    private double          xThrottle;
    public double           getXThrottle()                  {return xThrottle;}
    protected ArmStatus     setXThrottle(double xThrottle)  {this.xThrottle = xThrottle; return this;}

    private double          xAdjustment; // extension from turret center of rotation
    public double           getXAdjustment()                        {return xAdjustment;}
    protected ArmStatus     setXAdjustment(double xAdjustment)      {this.xAdjustment = xAdjustment; return this;}
    protected ArmStatus     incrementXAdjustment(double increment)  {this.xAdjustment += increment; return this;}

    private double          zThrottle;
    public double           getZThrottle()                  {return zThrottle;}
    protected ArmStatus     setZThrottle(double zThrottle)  {this.zThrottle = zThrottle; return this;}

    private double          zAdjustment; // height
    public double           getZAdjustment()                        {return zAdjustment;}
    protected ArmStatus     setZAdjustment(double zAdjustment)      {this.zAdjustment = zAdjustment; return this;}
    protected ArmStatus     incrementZAdjustment(double increment)  {this.zAdjustment += increment; return this;}

    // Shoulder
    private PotAndEncoder.Reading   shoulderPotEncReading = new PotAndEncoder.Reading(0,0,0);
    private ArmStatus               setShoulderPotEncReading(PotAndEncoder.Reading shoulderReading)   {this.shoulderPotEncReading = shoulderReading; return this;}
    
    private PotAndEncoder.Status    shoulderPotEncStatus;
    public PotAndEncoder.Status     getShoulderPotEncStatus()                                     {return shoulderPotEncStatus;}
    private ArmStatus               setShoulderPotEncStatus(PotAndEncoder.Status shoulderStatus)  {this.shoulderPotEncStatus = shoulderStatus; return this;}

    protected final static double kShoulderMotorGearRatio = 4.0 * 4.0 * 72.0/16.0;
    private static final double kShoulderEncoderUnitsPerRev = 2048.0 * kShoulderMotorGearRatio;
    private static final double kShoulderEncoderUnitsPerRad = kShoulderEncoderUnitsPerRev / (2*Math.PI);
  
    public static double shoulderRadiansToSensorUnits(double _radians) { return _radians * kShoulderEncoderUnitsPerRad; }
    public static double shoulderSensorUnitsToRadians(double _units) { return _units / kShoulderEncoderUnitsPerRad; }
    
    private boolean     shoulderFalconCalibrated = false;
    public boolean      getShoulderFalconCalibrated() { return shoulderFalconCalibrated; }
    protected ArmStatus setShoulderFalconCalibrated(boolean shoulderFalconCalibrated) { this.shoulderFalconCalibrated = shoulderFalconCalibrated; return this; }

    private double      shoulderFalconSensorPosition;
    public double       getShoulderFalconSensorPosition() { return shoulderFalconSensorPosition; }
    protected ArmStatus setShoulderFalconSensorPosition(double _units) {shoulderFalconSensorPosition = _units; return this; }

    private double      shoulderCalibAngleRad;
    public double       getShoulderCalibAngleRad() {return shoulderCalibAngleRad;}
    protected ArmStatus setShoulderCalibAngleRad(double shoulderCalibAngleRad) {this.shoulderCalibAngleRad = shoulderCalibAngleRad; return this;}

    private double      shoulderMinAngleRad;
    public double       getShoulderMinAngleRad() { return shoulderMinAngleRad;}
    protected ArmStatus setShoulderMinAngleRad(double shoulderMinAngleRad) {this.shoulderMinAngleRad = shoulderMinAngleRad; return this;}
    
    private double      shoulderMaxAngleRad;
    public double       getShoulderMaxAngleRad() {return shoulderMaxAngleRad; }
    protected ArmStatus setShoulderMaxAngleRad(double shoulderMaxAngleRad) {this.shoulderMaxAngleRad = shoulderMaxAngleRad; return this;}

    // main function to get shoulder angle
    private boolean useShoulderFalconForAngle = false;
    public double   getShoulderAngleRad() { return useShoulderFalconForAngle ? shoulderSensorUnitsToRadians(getShoulderFalconSensorPosition()) : Units.degreesToRadians(getShoulderPotEncStatus().positionDeg); } 

    private double      shoulderPower;
    public double       getShoulderPower()                          {return shoulderPower;}
    public double       getShoulderVoltage()                        {return shoulderPower * 12;}
    protected ArmStatus setShoulderPower(double shoulderPower)      {this.shoulderPower = shoulderMotorSoftLimit(shoulderPower); return this;}
    protected ArmStatus setShoulderVoltage(double shoulderVoltage)  {this.shoulderPower = shoulderMotorSoftLimit(shoulderVoltage / 12); return this;}

    private static final double kShoulderMinPower = 0;//0.055;
    public double shoulderMotorSoftLimit(double _power) {
        double power = _power;

        ArmStatus status = ArmStatus.getInstance();
        double shoulderAngleRad = Units.degreesToRadians(status.getShoulderPotEncStatus().positionDeg);
        double elbowAngleRad = Units.degreesToRadians(status.getElbowPotEncStatus().positionDeg);
        double relativeAngle = elbowAngleRad - shoulderAngleRad;

        // check forward limits
        if ((shoulderAngleRad > shoulderMaxAngleRad) || (relativeAngle > ArmLoop.kRelativeMaxAngleRad)) {
            power = Math.max(power, 0.0);   // still allow movement in reverse direction
        }
        // check reverse limits
        if ((shoulderAngleRad < shoulderMinAngleRad) || (relativeAngle < ArmLoop.kRelativeMinAngleRad)) {
            power = Math.min(power, 0.0);   // still allow movement in forward direction
        }
        if(Math.signum(power) == Math.signum(shoulderAngleRad + Units.degreesToRadians(90))) {
            power += Math.signum(power) * kShoulderMinPower;
        }
        return power;
    }

    private double      shoulderAngleRadSetpoint;
    public double       getShoulderAngleRadSetpoint()                                   {return shoulderAngleRadSetpoint;}
    protected ArmStatus setShoulderAngleRadSetpoint(double shoulderAngleRadSetpoint)    {this.shoulderAngleRadSetpoint = shoulderAngleRadSetpoint; return this;}

    private double      shoulderFeedforward;
    public double       getShoulderFeedforward()                            {return shoulderFeedforward;}
    protected ArmStatus setShoulderFeedforward(double shoulderFeedforward)  {this.shoulderFeedforward = shoulderFeedforward; return this;}

    private double      shoulderPIDOutput;
    public double       getShoulderPIDOutput()                          {return shoulderPIDOutput;}
    protected ArmStatus setShoulderPIDOutput(double shoulderPIDOutput)  {this.shoulderPIDOutput = shoulderPIDOutput; return this;}

    // Elbow
    private PotAndEncoder.Reading   elbowPotEncReading = new PotAndEncoder.Reading(0,0,0);
    private ArmStatus               setElbowPotEncReading(PotAndEncoder.Reading elbowReading) {this.elbowPotEncReading = elbowReading; return this;}
    
    private PotAndEncoder.Status    elbowPotEncStatus;
    public PotAndEncoder.Status     getElbowPotEncStatus()                                    {return elbowPotEncStatus;}
    private ArmStatus               setElbowPotEncStatus(PotAndEncoder.Status elbowStatus)    {this.elbowPotEncStatus = elbowStatus; return this;}

    protected final static double kElbowMotorGearRatio = 4.0 * 4.0 * 64.0/16.0;
    private static final double kElbowEncoderUnitsPerRev = 2048.0 * kElbowMotorGearRatio;
    private static final double kElbowEncoderUnitsPerRad = kElbowEncoderUnitsPerRev / (2*Math.PI);
  
    public static double elbowRadiansToSensorUnits(double _radians) { return _radians * kElbowEncoderUnitsPerRad; }
    public static double elbowSensorUnitsToRadians(double _units) { return _units / kElbowEncoderUnitsPerRad; }
    
    private boolean     elbowFalconCalibrated = false;
    public boolean      getElbowFalconCalibrated() { return elbowFalconCalibrated; }
    protected ArmStatus setElbowFalconCalibrated(boolean elbowFalconCalibrated) { this.elbowFalconCalibrated = elbowFalconCalibrated; return this; }

    private double      elbowFalconSensorPosition;
    public double       getElbowFalconSensorPosition() { return elbowFalconSensorPosition; }
    protected ArmStatus setElbowFalconSensorPosition(double _units) {elbowFalconSensorPosition = _units; return this; }

    private double      elbowCalibAngleRad;
    public double       getElbowCalibAngleRad() {return elbowCalibAngleRad;}
    protected ArmStatus setElbowCalibAngleRad(double elbowCalibAngleRad) {this.elbowCalibAngleRad = elbowCalibAngleRad; return this;}

    private double      elbowMinAngleRad;
    public double       getElbowMinAngleRad() { return elbowMinAngleRad;}
    protected ArmStatus setElbowMinAngleRad(double elbowMinAngleRad) {this.elbowMinAngleRad = elbowMinAngleRad; return this;}
    
    private double      elbowMaxAngleRad;
    public double       getElbowMaxAngleRad() {return elbowMaxAngleRad; }
    protected ArmStatus setElbowMaxAngleRad(double elbowMaxAngleRad) {this.elbowMaxAngleRad = elbowMaxAngleRad; return this;}

    // main function to get elbow angle
    private boolean useElbowFalconForAngle = false;
    public double   getElbowAngleRad() { return useElbowFalconForAngle ? elbowSensorUnitsToRadians(getElbowFalconSensorPosition()) : Units.degreesToRadians(getElbowPotEncStatus().positionDeg); } 

    private double      elbowPower;
    public double       getElbowPower()                       {return elbowPower;}
    public double       getElbowVoltage()                     {return elbowPower * 12;}
    protected ArmStatus setElbowPower(double elbowPower)      {this.elbowPower = elbowMotorSoftLimit(elbowPower); return this;}
    protected ArmStatus setElbowVoltage(double elbowVoltage)  {this.elbowPower = elbowMotorSoftLimit(elbowVoltage / 12); return this;}

    public double elbowMotorSoftLimit(double _power) {
        double power = _power;

        ArmStatus status = ArmStatus.getInstance();
        double shoulderAngleRad = Units.degreesToRadians(status.getShoulderPotEncStatus().positionDeg);
        double elbowAngleRad = Units.degreesToRadians(status.getElbowPotEncStatus().positionDeg);
        double relativeAngle = elbowAngleRad - shoulderAngleRad;

        // check forward limits
        if ((elbowAngleRad > elbowMaxAngleRad) || (relativeAngle > ArmLoop.kRelativeMaxAngleRad)) {
            power = Math.max(power, 0.0);   // still allow movement in reverse direction
        }
        // check reverse limits
        if ((elbowAngleRad < elbowMinAngleRad) || (relativeAngle < ArmLoop.kRelativeMinAngleRad)) {
            power = Math.min(power, 0.0);   // still allow movement in forward direction
        }
        return power;
    }

    private double      elbowAngleRadSetpoint;
    public double       getElbowAngleRadSetpoint()                              {return elbowAngleRadSetpoint;}
    protected ArmStatus setElbowAngleRadSetpoint(double elbowAngleRadSetpoint)  {this.elbowAngleRadSetpoint = elbowAngleRadSetpoint; return this;}

    private double      elbowFeedforward;
    public double       getElbowFeedforward()                           {return elbowFeedforward;}
    protected ArmStatus setElbowFeedforward(double elbowFeedforward)    {this.elbowFeedforward = elbowFeedforward; return this;}

    private double      elbowPIDOutput;
    public double       getElbowPIDOutput()                         {return elbowPIDOutput;}
    protected ArmStatus setElbowPIDOutput(double elbowPIDOutput)    {this.elbowPIDOutput = elbowPIDOutput; return this;}

    // Claw
    private boolean     clawGrabbing;
    public boolean      getClawGrabbing()                       {return clawGrabbing;}
    protected ArmStatus setClawGrabbing(boolean clawGrabbing)   {this.clawGrabbing = clawGrabbing; return this;}

    @Override
    public void updateInputs() {
        setCommand(arm.getCommand());
        setTurretPosition(HAL.getTurretRelative());
        setShoulderPotEncReading(HAL.getShoulderPotEncoder().getReading());
        setElbowPotEncReading(HAL.getElbowPotEncoder().getReading());
        setShoulderFalconSensorPosition(HAL.getShoulderFalconSensorPosition());
        setElbowFalconSensorPosition(HAL.getElbowFalconSensorPosition());
    }

    @Override
    public void exportToTable(LogTable table) {
        table.put("Turret Position", getTurretPosition());
        shoulderPotEncReading.exportToTable(table, "Shoulder Reading");
        elbowPotEncReading.exportToTable(table, "Elbow Reading");     
    }
    
    @Override
    public void importFromTable(LogTable table) {
        setTurretPosition(table.getDouble("Turret Position", turretPosition));
        setShoulderPotEncReading(shoulderPotEncReading.importFromTable(table, "Shoulder Reading"));
        setElbowPotEncReading(elbowPotEncReading.importFromTable(table, "Elbow Reading"));
    }


    @Override
    public void processTable() {
        setShoulderPotEncStatus(HAL.getShoulderPotEncoder().update(shoulderPotEncReading));
        setElbowPotEncStatus(HAL.getElbowPotEncoder().update(elbowPotEncReading));
    }

    boolean oneShotShoulderCalibrationEnabled = true;
    boolean oneShotElbowCalibrationEnabled = true;

    @Override
    public void processOutputs(Logger logger, String prefix) {
        // Generic
        command.recordOutputs(logger, prefix + "Command");
        logger.recordOutput(prefix + "Current Arm State", armState != null ? armState.name() : "null");

        // Turret
        HAL.setTurretPower(turretPower);

        logger.recordOutput(prefix + "Turret/Power",        getTurretPower());
        logger.recordOutput(prefix + "Turret/Position",     getTurretPosition());
        logger.recordOutput(prefix + "Turret/Target Angle", getTargetTurretAngle());

        // Arm
        logger.recordOutput(prefix + "Arm/Target Pose",             targetArmPose != null ? targetArmPose.name() : "null");
        logger.recordOutput(prefix + "Arm/Current Pose",            currentArmPose != null ? currentArmPose.name() : "null");
        logger.recordOutput(prefix + "Arm/Target Node",             targetNode != null ? targetNode.name() : "null");
        logger.recordOutput(prefix + "Arm/Adjustments/Throttle/X",  xThrottle);
        logger.recordOutput(prefix + "Arm/Adjustments/Throttle/Z",  zThrottle);
        logger.recordOutput(prefix + "Arm/Adjustments/X",           xAdjustment);
        logger.recordOutput(prefix + "Arm/Adjustments/Z",           zAdjustment);
        logger.recordOutput(prefix + "Arm/Trajectory/Internal Disable",     internalDisable);
        logger.recordOutput(prefix + "Arm/Trajectory/Current Trajectory",   currentArmTrajectory != null ? "Start pose: " + currentArmTrajectory.getStartString() + " Final pose: " + currentArmTrajectory.getFinalString() : "null");
        AdvantageUtil.recordTrajectoryVector(logger, prefix + "Arm/Trajectory/Current State/Theta1", getCurrentTrajState().extractRowVector(0));
        AdvantageUtil.recordTrajectoryVector(logger, prefix + "Arm/Trajectory/Current State/Theta2", getCurrentTrajState().extractRowVector(1));
        AdvantageUtil.recordTrajectoryVector(logger, prefix + "Arm/Trajectory/Setpoint State/Theta1", getSetpointTrajState().extractRowVector(0));
        AdvantageUtil.recordTrajectoryVector(logger, prefix + "Arm/Trajectory/Setpoint State/Theta2", getSetpointTrajState().extractRowVector(1));

        // Shoulder
        if (getShoulderFalconCalibrated() && oneShotShoulderCalibrationEnabled) {
            HAL.setShoulderFalconSensorPosition(shoulderRadiansToSensorUnits(getShoulderCalibAngleRad()));
            HAL.enableShoulderSoftLimits(shoulderRadiansToSensorUnits(getShoulderMinAngleRad()), shoulderRadiansToSensorUnits(getShoulderMaxAngleRad()));
            oneShotShoulderCalibrationEnabled = false;
        }
        HAL.setShoulderMotorPower(shoulderPower);
        shoulderPotEncStatus.recordOutputs(logger, prefix + "Arm/Shoulder Status");
        logger.recordOutput(prefix + "Arm/Shoulder/Power",          shoulderPower);
        logger.recordOutput(prefix + "Arm/Shoulder/PotEnc Angle (Rad)",    Units.degreesToRadians(getShoulderPotEncStatus().positionDeg));
        logger.recordOutput(prefix + "Arm/Shoulder/Falcon Angle (Rad)",    shoulderSensorUnitsToRadians(getShoulderFalconSensorPosition()));
        logger.recordOutput(prefix + "Arm/Shoulder/Angle (Rad)",    getShoulderAngleRad());
        logger.recordOutput(prefix + "Arm/Shoulder/Setpoint",       getShoulderAngleRadSetpoint());
        logger.recordOutput(prefix + "Arm/Shoulder/Feedforward",    getShoulderFeedforward());
        logger.recordOutput(prefix + "Arm/Shoulder/PID Output",     getShoulderPIDOutput());
        
        // Elbow
        if (getElbowFalconCalibrated() && oneShotElbowCalibrationEnabled) {
            HAL.setElbowFalconSensorPosition(elbowRadiansToSensorUnits(getElbowCalibAngleRad()));
            HAL.enableElbowSoftLimits(elbowRadiansToSensorUnits(getElbowMinAngleRad()), elbowRadiansToSensorUnits(getElbowMaxAngleRad()));
            oneShotElbowCalibrationEnabled = false;
        }        
        HAL.setElbowMotorPower(elbowPower);
        elbowPotEncStatus.recordOutputs(logger, prefix + "Arm/Elbow Status");
        logger.recordOutput(prefix + "Arm/Elbow/Power",         elbowPower);
        logger.recordOutput(prefix + "Arm/Elbow/PotEnc Angle (Rad)",    Units.degreesToRadians(getElbowPotEncStatus().positionDeg));
        logger.recordOutput(prefix + "Arm/Elbow/Falcon Angle (Rad)",    elbowSensorUnitsToRadians(getElbowFalconSensorPosition()));
        logger.recordOutput(prefix + "Arm/Elbow/Angle (Rad)",    getElbowAngleRad());
        logger.recordOutput(prefix + "Arm/Elbow/Setpoint",      getElbowAngleRadSetpoint());
        logger.recordOutput(prefix + "Arm/Elbow/Feedforward",   getElbowFeedforward());
        logger.recordOutput(prefix + "Arm/Elbow/PID Output",    getElbowPIDOutput());
    }
}