import json
from unyt import meter, inch, lb, kg

def main():

    arm_config = {"frame_width_inches": 24.0,
                  "bumper_width_inches": 30.5,
                  "origin": [0.343, 1.270],
                  "shoulder": {
                      # "mass": ((3.52 * lb).to("kg")).value.tolist(),
                      "mass": ((2.0 * lb).to("kg")).value.tolist(),
                      "length": ((20.5 * inch).to("meter")).value.tolist(),
                      "moi": 0.153,
                      "cgRadius": ((4.875 * inch).to("meter")).value.tolist(),
                      "minAngle": -3.0,
                      "maxAngle": 0,
                      "motor": {
                          "type": "falcon",
                          "count": 1,
                          "reduction": 4.0 * 4.0 * 72.0 / 16.0
                      }
                  },
                  "elbow": {
                      # "mass": ((1.21 * lb).to("kg")).value.tolist(),
                      "mass": ((2.5 * lb).to("kg")).value.tolist(),
                      "length": ((21.0 * inch).to("meter")).value.tolist(),
                      "moi": 0.013,
                      "cgRadius": ((16.0 * inch).to("meter")).value.tolist(),
                      "minAngle": -1.57,
                      "maxAngle": 3.00,
                      "motor": {
                          "type": "falcon",
                          "count": 1,
                          "reduction": 4.0 * 4.0 * 64.0 / 16.0
                      }
                  },
                  "wrist": {
                      "mass": ((2.8 * lb).to("kg")).value.tolist(),
                      "length": (((29.0 - 21.0) * inch).to("meter")).value.tolist(),
                      "moi": 0.8,
                      "cgRadius": ((1.5 * inch).to("meter")).value.tolist()
                  },
                  "cone": {
                      "mass": (((1 + 7/16) * lb).to("kg")).value.tolist(),
                      "length": ((0.0 * inch).to("meter")).value.tolist(),
                      "moi": 0.019,
                      "cgRadius": ((0.0 * inch).to("meter")).value.tolist()
                  }}


    with open("src/main/deploy/arm_config.json", "w") as outfile:
        json.dump(arm_config, outfile, indent=2)

if __name__ == '__main__':
    main()
