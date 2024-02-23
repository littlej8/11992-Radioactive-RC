package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Trapdoor {
    private final Servo trap;

    public static double OPENED, CLOSED = 0.0, 0.5;

    public Trapdoor(HardwareMap hwMap) {
        trap = hwMap.get(Servo.class, "Trapdoor");
    }

    public void Open() {
        trap.setPosition(OPENED);
    }

    public void Close() {
        trap.setPosition(CLOSED);
    }
}
