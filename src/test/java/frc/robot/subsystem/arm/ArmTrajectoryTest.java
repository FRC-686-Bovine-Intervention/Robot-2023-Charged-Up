package frc.robot.subsystem.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.mockStatic;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedStatic;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.FieldDimensions;
import frc.robot.lib.util.GeomUtil;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmDynamics;
import frc.robot.subsystems.arm.ArmDynamics.JointConfig;
import frc.robot.subsystems.arm.ArmKinematics;
import frc.robot.subsystems.arm.ArmLoop;
import frc.robot.subsystems.arm.ArmPose;
import frc.robot.subsystems.arm.ArmStatus;
import frc.robot.subsystems.arm.ArmTrajectory;
import frc.robot.subsystems.arm.json.ArmConfigJson;
import frc.robot.subsystems.arm.json.ArmPathsJson;
import frc.robot.subsystems.arm.json.ArmPresetsJson;

public class ArmTrajectoryTest {
    
    private static final double kEps = 1e-9;

    // @Test
    // public void TestArmTrajectory() {
        
    //     double w0 = 0.1;
    //     double w1 = 0.2;

    //     List<Vector<N2>> points = new ArrayList<>();
    //     int N = 50; // number of points
    //     for (int k=0; k<N; k++) {
    //         double theta0 = w0 * k;
    //         double theta1 = w1 * k;
    //         points.add(VecBuilder.fill(theta0, theta1));
    //     }

    //     double totalTime = 1.0;
    //     ArmTrajectory traj = new ArmTrajectory("begin", "end", totalTime, points);
    //     traj.setGrannyFactor(1.0);

    //     assertEquals(totalTime, traj.getTotalTime(), kEps);

    //     double grannyFactor = 5.0;
    //     traj.setGrannyFactor(grannyFactor);
    //     assertEquals(totalTime * grannyFactor, traj.getTotalTime(), kEps);

    //     grannyFactor = 1.0;
    //     traj.setGrannyFactor(grannyFactor);
    //     assertEquals(totalTime * grannyFactor, traj.getTotalTime(), kEps);
        
    //     var finalState = traj.getFinalState();
    //     assertEquals((N-1) * w0, finalState.get(0,0), kEps);
    //     assertEquals(0.0, finalState.get(0,1), kEps);
    //     assertEquals(0.0, finalState.get(0,2), kEps);
    //     assertEquals((N-1) * w1, finalState.get(1,0), kEps);
    //     assertEquals(0.0, finalState.get(1,1), kEps);
    //     assertEquals(0.0, finalState.get(1,2), kEps);

    //     var state = traj.sample(totalTime/2.0);
    //     assertEquals((N-1)/2.0 * w0, state.get(0,0), kEps);
    //     assertEquals((N-1)*w0/totalTime, state.get(0,1), kEps);
    //     assertEquals(0.0, state.get(0,2), kEps);
    //     assertEquals((N-1)/2.0 * w1, state.get(1,0), kEps);
    //     assertEquals((N-1)*w1/totalTime, state.get(1,1), kEps);
    //     assertEquals(0.0, state.get(1,2), kEps);
     
    //     state = traj.sample(totalTime);
    //     assertEquals(finalState.get(0,0), state.get(0,0), kEps);
    //     assertEquals(0.0, state.get(0,1), kEps);
    //     assertEquals(finalState.get(1,0), state.get(1,0), kEps);
    //     assertEquals(0.0, state.get(1,1), kEps);
        
        
    //     double start_theta0_actual =  0.01;
    //     double start_theta1_actual = -0.02;

    //     assertTrue(traj.startIsNear(start_theta0_actual, start_theta1_actual, Units.degreesToRadians(10.0)));

    //     start_theta0_actual = 1.0;
    //     start_theta1_actual = 2.0;

    //     assertFalse(traj.startIsNear(start_theta0_actual, start_theta1_actual, Units.degreesToRadians(10.0)));


    //     ArmTrajectory interpTraj = traj.interpolateEndPoints(start_theta0_actual, start_theta1_actual, finalState.get(0,0), finalState.get(1,0));

    //     // interpTraj should end in the same place
    //     finalState = interpTraj.getFinalState();
    //     assertEquals((N-1) * w0, finalState.get(0,0), kEps);
    //     assertEquals(0.0, finalState.get(0,1), kEps);
    //     assertEquals(0.0, finalState.get(0,2), kEps);
    //     assertEquals((N-1) * w1, finalState.get(1,0), kEps);
    //     assertEquals(0.0, finalState.get(1,1), kEps);
    //     assertEquals(0.0, finalState.get(1,2), kEps);

    //     state = interpTraj.sample(0);
    //     assertEquals(start_theta0_actual, state.get(0,0), kEps);
    //     assertEquals(((N-1)*w0-start_theta0_actual)/totalTime, state.get(0,1), kEps);
    //     assertEquals(start_theta1_actual, state.get(1,0), kEps);
    //     assertEquals(((N-1)*w1-start_theta1_actual)/totalTime, state.get(1,1), kEps);
        
    //     state = interpTraj.sample(totalTime/2.0);
    //     assertEquals((start_theta0_actual + (N-1) * w0)/2.0, state.get(0,0), kEps);
    //     assertEquals(((N-1)*w0-start_theta0_actual)/totalTime, state.get(0,1), kEps);
    //     assertEquals((start_theta1_actual + (N-1) * w1)/2.0, state.get(1,0), kEps);
    //     assertEquals(((N-1)*w1-start_theta1_actual)/totalTime, state.get(1,1), kEps);
        
    //     state = interpTraj.sample(totalTime);
    //     assertEquals(finalState.get(0,0), state.get(0,0), kEps);
    //     assertEquals(0.0, state.get(0,1), kEps);
    //     assertEquals(finalState.get(1,0), state.get(1,0), kEps);
    //     assertEquals(0.0, state.get(1,1), kEps);        

    //     double final_theta0_actual = 2.0;
    //     double final_theta1_actual = 1.0;

    //     interpTraj = traj.interpolateEndPoints(traj.sample(0).get(0,0), traj.sample(0).get(1,0), final_theta0_actual, final_theta1_actual);

    //     // interpTraj should finish at the actual
    //     finalState = interpTraj.getFinalState();
    //     assertEquals(final_theta0_actual, finalState.get(0,0), kEps);
    //     assertEquals(0.0, finalState.get(0,1), kEps);
    //     assertEquals(0.0, finalState.get(0,2), kEps);
    //     assertEquals(final_theta1_actual, finalState.get(1,0), kEps);
    //     assertEquals(0.0, finalState.get(1,1), kEps);
    //     assertEquals(0.0, finalState.get(1,2), kEps);

    //     state = interpTraj.sample(0);
    //     assertEquals(0.0, state.get(0,0), kEps);
    //     assertEquals(final_theta0_actual/totalTime, state.get(0,1), kEps);
    //     assertEquals(0.0, state.get(1,0), kEps);
    //     assertEquals(final_theta1_actual/totalTime, state.get(1,1), kEps);
        
    //     state = interpTraj.sample(totalTime/2.0);
    //     assertEquals(final_theta0_actual/2.0, state.get(0,0), kEps);
    //     assertEquals(final_theta0_actual/totalTime, state.get(0,1), kEps);
    //     assertEquals(final_theta1_actual/2.0, state.get(1,0), kEps);
    //     assertEquals(final_theta1_actual/totalTime, state.get(1,1), kEps);
        
    //     state = interpTraj.sample(totalTime);
    //     assertEquals(final_theta0_actual, state.get(0,0), kEps);
    //     assertEquals(0.0, state.get(0,1), kEps);
    //     assertEquals(final_theta1_actual, state.get(1,0), kEps);
    //     assertEquals(0.0, state.get(1,1), kEps);        

    // } 
    
    
    // @Test
    // void TestStartTrajectory() {

    //     ArmTest.setDeployDirectoryDuringTest();
    //     ArmStatus mockStatus = mock(ArmStatus.class);
    //     MockedStatic<ArmStatus> mockStatusStatic = mockStatic(ArmStatus.class);
    //     mockStatusStatic.when(ArmStatus::getInstance).thenReturn(mockStatus);

    //     // Get presets from JSON
    //     File presetFile = new File(Filesystem.getDeployDirectory(), ArmPresetsJson.jsonFilename);
    //     ArmPresetsJson presets = ArmPresetsJson.loadJson(presetFile);
    //     ArmPose.Preset.writePresets(presets);
                
    //     ArmTrajectory[][] armTrajectories = new ArmTrajectory[ArmPose.Preset.values().length+1][ArmPose.Preset.values().length+1];
    //     // Get paths from JSON
    //     // also create trajectories for each path
    //     for (ArmPose.Preset startPos : ArmPose.Preset.values()) {
    //         for (ArmPose.Preset finalPos : ArmPose.Preset.values()) {
    //             int startIdx = startPos.getFileIdx();
    //             int finalIdx = finalPos.getFileIdx();

    //             String pathFilename = String.format(ArmPathsJson.jsonFilename, startIdx, finalIdx);
                
    //             File pathFile = new File(Filesystem.getDeployDirectory(), pathFilename);
    //             var path = ArmPathsJson.loadJson(pathFile);

    //             // create trajectory for each path
    //             List<Vector<N2>> points = new ArrayList<>();
    //             for (int k=0; k<path.theta1().size(); k++) {
    //                 points.add(VecBuilder.fill(path.theta1().get(k), path.theta2().get(k)));
    //             }

    //             armTrajectories[startIdx][finalIdx] = new ArmTrajectory(path.startPos(), path.finalPos(), path.totalTime(), points);
    //         }
    //     }

    //     ArmConfigJson config = Arm.getInstance().getConfig();

    //     JointConfig shoulder = config.shoulder();
    //     JointConfig elbow = ArmDynamics.rigidlyCombineJoints(config.elbow(), config.wrist());

    //     ArmKinematics kinematics = new ArmKinematics(new Translation2d(config.origin().getX(), config.origin().getY()),
    //                                     shoulder.length(), elbow.length(),
    //                                     config.shoulder().minAngle(), config.shoulder().maxAngle(), 
    //                                     elbow.minAngle(), elbow.maxAngle(),
    //                                     ArmLoop.kRelativeMinAngleRad, ArmLoop.kRelativeMaxAngleRad);        

        
    //     // Test High Cone in left-most grid

    //     ArmPose.Preset startPos = ArmPose.Preset.DEFENSE;
    //     ArmPose.Preset finalPos = ArmPose.Preset.SCORE_HIGH_CONE;
    //     ArmStatus.NodeEnum targetNode = ArmStatus.NodeEnum.TopLoading;
    //     Pose2d robotXY = new Pose2d(new Translation2d(1.8,1.0), new Rotation2d(0.0));
    //     double turretToRobotAngleDeg = 0.0;
    //     Pose3d turretXY = getTurretPose(robotXY, turretToRobotAngleDeg);

    //     MockedStatic<DriverStation> mockDS = mockStatic(DriverStation.class);
    //     mockDS.when(DriverStation::getAlliance).thenReturn(Alliance.Blue);

    //     when(mockStatus.getShoulderAngleRad()).thenReturn(startPos.getShoulderAngleRad());
    //     when(mockStatus.getElbowAngleRad()).thenReturn(startPos.getElbowAngleRad());
    //     when(mockStatus.getCurrentArmTrajectory()).thenReturn(armTrajectories[startPos.getFileIdx()][finalPos.getFileIdx()]);       
    //     when(mockStatus.getTargetNode()).thenReturn(targetNode);
    //     when(mockStatus.getTurretToField()).thenReturn(turretXY);
    //     ArmLoop.getInstance().startTrajectory(startPos, finalPos);

    //     ArgumentCaptor<Double> turretCaptor = ArgumentCaptor.forClass(Double.class);
    //     ArgumentCaptor<ArmTrajectory> trajCaptor = ArgumentCaptor.forClass(ArmTrajectory.class);

    //     verify(mockStatus).setTargetTurretAngleDeg(turretCaptor.capture());
    //     double turretAngleToTarget = turretCaptor.getValue();   // relative to robot

    //     // assertEquals(-2.7767, turretAngleToTarget, 0.05);

    //     verify(mockStatus).setCurrentArmTrajectory(trajCaptor.capture());
    //     ArmTrajectory currentArmTraj = trajCaptor.getValue();        
    //     Matrix<N2,N3> finalState = currentArmTraj.getFinalState();

    //     var baseTraj = armTrajectories[startPos.getFileIdx()][finalPos.getFileIdx()];
    //     var basePath = baseTraj.getPoints();
    //     var interpPath = currentArmTraj.getPoints();

    //     System.out.printf("totalTime = %.2f, %.2f\n", baseTraj.getTotalTime(), currentArmTraj.getTotalTime());

    //     for (int k=0; k<interpPath.size(); k++) {
    //         if (k < basePath.size()) {
    //             System.out.printf("%5.2f\t%5.2f\t", basePath.get(k).get(0,0), basePath.get(k).get(1,0));
    //         } else {
    //             System.out.printf("-----\t-----\t");
    //         }
    //         System.out.printf("%5.2f\t%5.2f\n", interpPath.get(k).get(0,0), interpPath.get(k).get(1,0));
    //     }

    //     // Test High Cone in left-most grid

    //     startPos = ArmPose.Preset.SCORE_HIGH_CONE;
    //     finalPos = ArmPose.Preset.DEFENSE;

    //     when(mockStatus.getShoulderAngleRad()).thenReturn(interpPath.get(interpPath.size()-1).get(0,0));
    //     when(mockStatus.getElbowAngleRad()).thenReturn(interpPath.get(interpPath.size()-1).get(1,0));
    //     when(mockStatus.getCurrentArmTrajectory()).thenReturn(armTrajectories[startPos.getFileIdx()][finalPos.getFileIdx()]);       
    //     ArmLoop.getInstance().startTrajectory(startPos, finalPos);

    //     verify(mockStatus, times(2)).setCurrentArmTrajectory(trajCaptor.capture());
    //     currentArmTraj = trajCaptor.getValue();        
    //     finalState = currentArmTraj.getFinalState();

    //     baseTraj = armTrajectories[startPos.getFileIdx()][finalPos.getFileIdx()];
    //     basePath = baseTraj.getPoints();
    //     interpPath = currentArmTraj.getPoints();

    //     System.out.printf("totalTime = %.2f, %.2f\n", baseTraj.getTotalTime(), currentArmTraj.getTotalTime());

    //     for (int k=0; k<interpPath.size(); k++) {
    //         if (k < basePath.size()) {
    //             System.out.printf("%5.2f\t%5.2f\t", basePath.get(k).get(0,0), basePath.get(k).get(1,0));
    //         } else {
    //             System.out.printf("-----\t-----\t");
    //         }
    //         System.out.printf("%5.2f\t%5.2f\n", interpPath.get(k).get(0,0), interpPath.get(k).get(1,0));
    //     }        
    // }

    //     Translation2d turretExtension = kinematics.forward(finalState.get(0,0), finalState.get(1,0));
    //     double d = turretExtension.getX();
    //     double turretAngleRelField = turretAngleToTarget - robotXY.getRotation().getRadians();
    //     Translation2d turretToTarget = new Translation2d(d*Math.cos(turretAngleRelField), d*Math.sin(turretAngleRelField));

    //     Translation2d targetXY = GeomUtil.translation3dTo2dXY(turretXY.getTranslation()).plus(turretToTarget);

    //     assertEquals(FieldDimensions.Grids.highTranslations[0].getX(), targetXY.getX(), 0.1);
    //     assertEquals(FieldDimensions.Grids.highTranslations[0].getY(), targetXY.getY(), 0.1);

    //     // Test High Cone in middle grid

    //     startPos = ArmPose.Preset.DEFENSE;
    //     finalPos = ArmPose.Preset.SCORE_HIGH_CONE;
    //     targetNode = ArmStatus.NodeEnum.TopWall;
    //     robotXY = new Pose2d(new Translation2d(1.8,1.0 + Units.inchesToMeters(3*22.0)), new Rotation2d(0.0));
    //     turretToRobotAngleDeg = 0.0;
    //     turretXY = getTurretPose(robotXY, turretToRobotAngleDeg);

    //     when(mockStatus.getShoulderAngleRad()).thenReturn(startPos.getShoulderAngleRad());
    //     when(mockStatus.getElbowAngleRad()).thenReturn(startPos.getElbowAngleRad());
    //     when(mockStatus.getCurrentArmTrajectory()).thenReturn(armTrajectories[startPos.getFileIdx()][finalPos.getFileIdx()]); 
    //     when(mockStatus.getTargetNode()).thenReturn(targetNode);
    //     when(mockStatus.getTurretToField()).thenReturn(turretXY);              
    //     ArmLoop.getInstance().startTrajectory(startPos, finalPos);

    //     verify(mockStatus, times(2)).setTargetTurretAngleDeg(turretCaptor.capture());
    //     turretAngleToTarget = turretCaptor.getValue();   // relative to robot

    //     assertEquals(-2.7767, turretAngleToTarget, 0.05);

    //     verify(mockStatus, times(2)).setCurrentArmTrajectory(trajCaptor.capture());
    //     currentArmTraj = trajCaptor.getValue();        
    //     finalState = currentArmTraj.getFinalState();

    //     turretExtension = kinematics.forward(finalState.get(0,0), finalState.get(1,0));
    //     d = turretExtension.getX();
    //     turretAngleRelField = turretAngleToTarget - robotXY.getRotation().getRadians();
    //     turretToTarget = new Translation2d(d*Math.cos(turretAngleRelField), d*Math.sin(turretAngleRelField));

    //     targetXY = GeomUtil.translation3dTo2dXY(turretXY.getTranslation()).plus(turretToTarget);

    //     assertEquals(FieldDimensions.Grids.highTranslations[3].getX(), targetXY.getX(), 0.1);
    //     assertEquals(FieldDimensions.Grids.highTranslations[3].getY(), targetXY.getY(), 0.1);

    //     // Test High Cone in top grid

    //     startPos = ArmPose.Preset.DEFENSE;
    //     finalPos = ArmPose.Preset.SCORE_HIGH_CONE;
    //     targetNode = ArmStatus.NodeEnum.TopWall;
    //     robotXY = new Pose2d(new Translation2d(1.8,1.0 + Units.inchesToMeters(6*22.0)), new Rotation2d(0.0));
    //     turretToRobotAngleDeg = 0.0;
    //     turretXY = getTurretPose(robotXY, turretToRobotAngleDeg);

    //     when(mockStatus.getShoulderAngleRad()).thenReturn(startPos.getShoulderAngleRad());
    //     when(mockStatus.getElbowAngleRad()).thenReturn(startPos.getElbowAngleRad());
    //     when(mockStatus.getCurrentArmTrajectory()).thenReturn(armTrajectories[startPos.getFileIdx()][finalPos.getFileIdx()]);   
    //     when(mockStatus.getTargetNode()).thenReturn(targetNode);
    //     when(mockStatus.getTurretToField()).thenReturn(turretXY);            
    //     ArmLoop.getInstance().startTrajectory(startPos, finalPos);

    //     verify(mockStatus, times(3)).setTargetTurretAngleDeg(turretCaptor.capture());
    //     turretAngleToTarget = turretCaptor.getValue();   // relative to robot

    //     assertEquals(-2.7767, turretAngleToTarget, 0.05);

    //     verify(mockStatus, times(3)).setCurrentArmTrajectory(trajCaptor.capture());
    //     currentArmTraj = trajCaptor.getValue();        
    //     finalState = currentArmTraj.getFinalState();

    //     turretExtension = kinematics.forward(finalState.get(0,0), finalState.get(1,0));
    //     d = turretExtension.getX();
    //     turretAngleRelField = turretAngleToTarget - robotXY.getRotation().getRadians();
    //     turretToTarget = new Translation2d(d*Math.cos(turretAngleRelField), d*Math.sin(turretAngleRelField));

    //     targetXY = GeomUtil.translation3dTo2dXY(turretXY.getTranslation()).plus(turretToTarget);

    //     assertEquals(FieldDimensions.Grids.highTranslations[6].getX(), targetXY.getX(), 0.1);
    //     assertEquals(FieldDimensions.Grids.highTranslations[6].getY(), targetXY.getY(), 0.1);        
    // }

    Pose3d getTurretPose(Pose2d robotPose, double turretAngleDeg) {
        return new Pose3d(robotPose).transformBy(new Transform3d(ArmStatus.robotToTurretTranslation, new Rotation3d(0, 0, Units.degreesToRadians(turretAngleDeg))));
}

}
