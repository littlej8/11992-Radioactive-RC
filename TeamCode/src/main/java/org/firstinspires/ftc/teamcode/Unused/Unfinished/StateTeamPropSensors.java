package org.firstinspires.ftc.teamcode.Unused.Unfinished;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.acmerobotics.dashboard.config.Config;

@Config
public class StateTeamPropSensors {
    private final ColorRangeSensor left, right;

    public static double LIGHT_SENSITIVITY = 30, LEFT_BOUND = 145, RIGHT_BOUND = 170;

    private String side = "front";

    public StateTeamPropSensors(HardwareMap hwMap) {
        left = hwMap.get(ColorRangeSensor.class, "Left Sensor");
        right = hwMap.get(ColorRangeSensor.class, "Right Sensor");

        left.setGain(2);
        right.setGain(2);
    }

    public String getSide() {
        return side;
    }

    public boolean front() {
        return side.equals("front");
    }

    public boolean left() {
        return side.equals("left");
    }

    public boolean right() {
        return side.equals("right");
    }

    public void CheckSensors() {
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
                side = "left";
            } else if (right_count >= LIGHT_SENSITIVITY) {
                side = "right";
            }
        }
    }
}