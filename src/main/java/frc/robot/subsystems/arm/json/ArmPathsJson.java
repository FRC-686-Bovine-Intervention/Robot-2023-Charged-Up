package frc.robot.subsystems.arm.json;

import java.io.File;
import java.io.IOException;
import java.util.List;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

public class ArmPathsJson {
  String startPos;
  String finalPos;
  double totalTime;                 // total path time
  double grannyFactor;              // per-path total time multiplier
  List<Double> theta1;      // angle of shoulder in radians
  List<Double> theta2;      // angle of elbow in radians
  
  public static final String jsonFilename = "paths/arm_path_%d_%d.json";

  public String startPos() {
        return startPos;
    }

    public String finalPos() {
        return finalPos;
    }

    public double totalTime() {
      return totalTime;
  }

  public double grannyFactor() {
    return grannyFactor;
}

    public List<Double> theta1() {
        return theta1;
    }

    public List<Double> theta2() {
        return theta2;
    }

/** Generates a config instance by reading from a JSON file. */
  public static ArmPathsJson loadJson(File source) {
       // Set up object mapper
    ObjectMapper objectMapper = new ObjectMapper();
    ArmPathsJson path = new ArmPathsJson();

    try {
      JsonNode rootNode = objectMapper.readTree(source);

      path.startPos = rootNode.get("startPos").asText();
      path.finalPos = rootNode.get("finalPos").asText();
      path.totalTime = rootNode.get("totalTime").asDouble();
      path.grannyFactor = rootNode.get("grannyFactor").asDouble();
      path.theta1 = objectMapper.convertValue(rootNode.get("theta1"), new TypeReference<List<Double>>() {});
      path.theta2 = objectMapper.convertValue(rootNode.get("theta2"), new TypeReference<List<Double>>() {});      

    } catch (IOException e) {
      throw new RuntimeException("Failed to parse " + source.getName());
    }
    
    // Return result
    return path;
  }

}
