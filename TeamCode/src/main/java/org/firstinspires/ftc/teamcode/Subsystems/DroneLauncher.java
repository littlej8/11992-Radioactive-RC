package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DroneLauncher {
    private final Servo drone;

    public static double LAUNCH = 1.0, PRIMED = 0.0;

    public DroneLauncher(HardwareMap hwMap) {
        drone = hwMap.get(Servo.class, "DroneLauncher");

        drone.setDirection(Servo.Direction.FORWARD);
    }

    public void Launch() {
        drone.setPosition(LAUNCH);
    }

    public void Reset() {
        drone.setPosition(PRIMED);
    }

    public void UpdateTeleOp(Gamepad controller) {
        if (controller.left_bumper && controller.right_bumper && controller.left_trigger > 0 && controller.right_trigger > 0) {
            Launch();
        }
    }
}
