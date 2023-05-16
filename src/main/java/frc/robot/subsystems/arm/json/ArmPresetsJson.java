package frc.robot.subsystems.arm.json;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Translation2d;

public class ArmPresetsJson {
  PresetEntry defense;
  PresetEntry intake;
  PresetEntry double_substation;
  PresetEntry score_hybrid;
  PresetEntry score_mid_cube;
  PresetEntry score_high_cube;
  PresetEntry score_mid_cone;
  PresetEntry score_high_cone;
  PresetEntry auto_start;
  PresetEntry hold;

  public static final String jsonFilename = "arm_preset_poses.json";

  public static class PresetEntry {
      int fileIdx;
      Translation2d xy;
      double theta1;
      double theta2; 
      
    public PresetEntry(int fileIdx, Translation2d xy, double theta1, double theta2) {
        this.fileIdx = fileIdx;
        this.xy = xy;
        this.theta1 = theta1;
        this.theta2 = theta2;
      }

    public int getFileIdx() {
      return fileIdx;
    }

    public Translation2d getXy() {
      return xy;
    }
        
    public double getX() { return xy.getX(); }
    public double getY() { return xy.getY(); }
    public double getTheta1() { return theta1; }
    public double getTheta2() { return theta2; }    
  }
  


  public PresetEntry defense() {
    return defense;
  }

  public PresetEntry intake() {
    return intake;
  }

  public PresetEntry double_substation() {
    return double_substation;
  }

  public PresetEntry score_hybrid() {
    return score_hybrid;
  }

  public PresetEntry score_mid_cube() {
    return score_mid_cube;
  }

  public PresetEntry score_high_cube() {
    return score_high_cube;
  }

  public PresetEntry score_mid_cone() {
    return score_mid_cone;
  }

  public PresetEntry score_high_cone() {
    return score_high_cone;
  }

  public PresetEntry auto_start() {
    return auto_start;
  }

  public PresetEntry hold() {
    return hold;
  }

  
  /** Generates a config instance by reading from a JSON file. */
  public static ArmPresetsJson loadJson(File source) {
    // Set up object mapper
    ObjectMapper objectMapper = new ObjectMapper();

    ArmPresetsJson presets = new ArmPresetsJson();

    try {
      JsonNode rootNode = objectMapper.readTree(source); 
      
      presets.defense = parsePresetEntry(rootNode.get("defense"));
      presets.intake = parsePresetEntry(rootNode.get("intake"));
      presets.double_substation = parsePresetEntry(rootNode.get("double_substation"));
      presets.score_hybrid = parsePresetEntry(rootNode.get("score_hybrid"));
      presets.score_mid_cube = parsePresetEntry(rootNode.get("score_mid_cube"));
      presets.score_high_cube = parsePresetEntry(rootNode.get("score_high_cube"));
      presets.score_mid_cone = parsePresetEntry(rootNode.get("score_mid_cone"));
      presets.score_high_cone = parsePresetEntry(rootNode.get("score_high_cone"));
      presets.auto_start = parsePresetEntry(rootNode.get("auto_start"));
      presets.hold = parsePresetEntry(rootNode.get("hold"));

    } catch (IOException e) {
      throw new RuntimeException("Failed to parse " + source.getName());
    }

    // Return result
    return presets;
  }

  private static PresetEntry parsePresetEntry(JsonNode node) {
    int fileIdx;
    double theta1, theta2;
    Translation2d xy;

    fileIdx = node.get("fileIdx").asInt();
    theta1 = node.get("theta1").asDouble();
    theta2 = node.get("theta2").asDouble();
    xy = new Translation2d(node.get("xy").get(0).asDouble(), node.get("xy").get(1).asDouble());

    return new PresetEntry(fileIdx, xy,  theta1, theta2);
}  

}
