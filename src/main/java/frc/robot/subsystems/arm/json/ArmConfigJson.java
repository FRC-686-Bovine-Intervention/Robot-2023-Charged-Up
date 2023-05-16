package frc.robot.subsystems.arm.json;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.arm.ArmDynamics;

/** Represents all of the arm config data shared between the robot code and solver. */
public class ArmConfigJson {
    double frame_width_inches;
    double bumper_width_inches;
    Translation2d origin;
    ArmDynamics.JointConfig shoulder;
    ArmDynamics.JointConfig elbow;
    ArmDynamics.JointConfig wrist;
    ArmDynamics.JointConfig cone; 
  
  public double frame_width_inches() {
      return frame_width_inches;
    }

    public double bumper_width_inches() {
      return bumper_width_inches;
    }

    public Translation2d origin() {
      return origin;
    }

    public ArmDynamics.JointConfig shoulder() {
      return shoulder;
    }

    public ArmDynamics.JointConfig elbow() {
      return elbow;
    }

    public ArmDynamics.JointConfig wrist() {
      return wrist;
    }

    public ArmDynamics.JointConfig cone() {
      return cone;
    }


  public static final String jsonFilename = "arm_config.json";

  /** Config fields for solver. */
  public static class SolverConfig {
    int interiorPoints;
    double maxVoltage;
  }

  /** Arbitrary solver constraint. */
  public static class Constraint {
    String type;
    double[] args;
  }

  

  /** Generates a config instance by reading from a JSON file. */
  public static ArmConfigJson loadJson(File source) {
    ArmConfigJson armConfig = new ArmConfigJson();

    ObjectMapper objectMapper  = new ObjectMapper();
    try {
      JsonNode rootNode = objectMapper.readTree(source);

      armConfig.frame_width_inches = rootNode.get("frame_width_inches").asDouble();
      armConfig.bumper_width_inches = rootNode.get("bumper_width_inches").asDouble();
      armConfig.origin = new Translation2d(rootNode.get("origin").get(0).asDouble(), rootNode.get("origin").get(1).asDouble());
      armConfig.shoulder = parseJointConfig(rootNode.get("shoulder"));
      armConfig.elbow = parseJointConfig(rootNode.get("elbow"));
      armConfig.wrist = parseJointConfig(rootNode.get("wrist"));
      armConfig.cone = parseJointConfig(rootNode.get("cone"));
  
    } catch (IOException e) {
      throw new RuntimeException("Failed to parse arm config JSON");
    }   
    
    return armConfig;
  }

  private static ArmDynamics.JointConfig parseJointConfig(JsonNode node) {
    double mass, length, moi, cgRadius, minAngle=0, maxAngle=0;

    mass = node.get("mass").asDouble();
    length = node.get("length").asDouble();
    moi = node.get("moi").asDouble();
    cgRadius = node.get("cgRadius").asDouble();
    if (node.has("minAngle")) {
      minAngle = node.get("minAngle").asDouble();
    }
    if (node.has("maxAngle")) {
      maxAngle = node.get("maxAngle").asDouble();
    }

    DCMotor motor = null;
    int count = 1;
    double reduction = 1;

    if (node.has("motor")) {
      JsonNode motorNode = node.get("motor");
      String type = motorNode.get("type").asText();
      count = motorNode.get("count").asInt();
      reduction = motorNode.get("reduction").asDouble();

      switch (type) {
        case "neo":
          motor = DCMotor.getNEO(count).withReduction(reduction);
          break;
          case "neo550":
          motor = DCMotor.getNeo550(count).withReduction(reduction);        
          break;
          case "falcon":
          motor = DCMotor.getFalcon500(count).withReduction(reduction);
          break;
        default:
          motor = null;
      }        
    }
    return new ArmDynamics.JointConfig(mass, length, moi, cgRadius, minAngle, maxAngle, reduction, motor);
  }
}
