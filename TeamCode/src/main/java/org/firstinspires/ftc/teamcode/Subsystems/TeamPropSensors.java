package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

public class TeamPropSensors {
    private final ColorRangeSensor left, right;

    public static double LIGHT_SENSITIVITY, LEFT_BOUND, RIGHT_BOUND = 30, 145, 170;

    public enum TeamPropLocation {
        LEFT,
        RIGHT,
        FRONT,
    }

    public TeamPropSensors(HardwareMap hwMap) {
        left = hwMap.get(ColorRangeSensor.class, "Left Sensor");
        right = hwMap.get(ColorRangeSensor.class, "Right Sensor");

        left.setGain(2);
        right.setGain(2);
    }

    public TeamPropLocation CheckSensors() {
        TeamPropLocation ret = TeamPropLocation.FRONT;
        int left_count = 0;
        int right_count = 0;

        for (int i = 0; i < 50; i++) {
            double left_light = left.getRawLightDetected();
            double right_light = right.getRawLightDetected();

            if (left_light >= LEFT_BOUND) {
                left_count++;
            } else {
                left_count = 0;
            }

            if (right_light >= RIGHT_BOUND) {
                right_count++;
            } else {
                right_count = 0;
            }

            if (left_count >= LIGHT_SENSITIVITY) {
                ret = TeamPropLocation.LEFT;
            } else if (right_count >= LIGHT_SENSITIVITY) {
                ret = TeamPropLocation.RIGHT;
            }
        }

        return ret;
    }
}
>>>>>>> f4b53614836423d23b13a22c44ae89d829631ec9