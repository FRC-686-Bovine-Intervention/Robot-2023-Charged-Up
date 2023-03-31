package frc.robot.subsystems.arm.json;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;
import com.fasterxml.jackson.databind.module.SimpleModule;
import com.fasterxml.jackson.databind.node.ArrayNode;

import edu.wpi.first.math.geometry.Translation2d;

public record ArmPresetsJson(
  PresetEntry defense,
  PresetEntry intake,
  PresetEntry double_substation,
  PresetEntry score_hybrid,
  PresetEntry score_mid_cube,
  PresetEntry score_high_cube,
  PresetEntry score_mid_cone,
  PresetEntry score_high_cone,
  PresetEntry auto_start,
  PresetEntry hold) {

  public static final String jsonFilename = "arm_preset_poses.json";

  public static record PresetEntry(
      int fileIdx,
      Translation2d xy,
      double theta1,
      double theta2) {

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
  


  /** Converts double array to Translation2d instance. */
  private static class Translation2dDeserializer extends StdDeserializer<Translation2d> {
    public Translation2dDeserializer() {
      this(null);
    }

    public Translation2dDeserializer(Class<?> vc) {
      super(vc);
    }

    @Override
    public Translation2d deserialize(JsonParser jp, DeserializationContext ctxt)
        throws IOException, JsonProcessingException {
      ArrayNode node = jp.getCodec().readTree(jp);
      double x = (Double) node.get(0).numberValue();
      double y = (Double) node.get(1).numberValue();
      return new Translation2d(x, y);
    }
  }
    
  /** Generates a config instance by reading from a JSON file. */
  public static ArmPresetsJson loadJson(File source) {
    // Set up object mapper
    ObjectMapper mapper = new ObjectMapper();
    mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    SimpleModule module = new SimpleModule();
    module.addDeserializer(Translation2d.class, new Translation2dDeserializer());
    mapper.registerModule(module);

    // Read config data
    ArmPresetsJson presets;
    try {
      presets = mapper.readValue(source, ArmPresetsJson.class);
    } catch (IOException e) {
      throw new RuntimeException("Failed to parse " + source.getName());
    }

    // Return result
    return presets;
  }

}
