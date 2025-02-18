package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Trapdoor {
    private final Servo trap;

    public static double OPENED = 1.0, CLOSED = 0.75;

    public Trapdoor(HardwareMap hwMap) {
        trap = hwMap.get(Servo.class, "Trapdoor");
        trap.setDirection(Servo.Direction.REVERSE);
        trap.setPosition(CLOSED);
    }

    public void Open() {
        trap.setPosition(OPENED);
    }

    public void Close() {
        trap.setPosition(CLOSED);
    }
}
