package frc.robot.subsystems.arm.json;

import java.io.File;
import java.io.IOException;
import java.util.List;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.module.SimpleModule;

public record ArmPathsJson(
  String startPos,
  String finalPos,
  double totalTime,                 // total path time
  List<Double> theta1,      // angle of proximal arm in radians
  List<Double> theta2)      // angle of distal arm in radians
  {     
  
  public static final String jsonFilename = "paths/arm_paths_%d_%d.json";

  public String startPos() {
        return startPos;
    }

    public String finalPos() {
        return finalPos;
    }

    public double totalTime() {
        return totalTime;
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
    ObjectMapper mapper = new ObjectMapper();
    mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    SimpleModule module = new SimpleModule();
    mapper.registerModule(module);

    // Read config data
    ArmPathsJson paths;
    try {
        paths = mapper.readValue(source, ArmPathsJson.class);
    } catch (IOException e) {
      throw new RuntimeException("Failed to parse " + source.getName());
    }

    // Return result
    return paths;
  }

}
