package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private final Servo claw;

    public static double OPENED = 1.0, CLOSED = 0.0;

    public Claw(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "Claw");
    }

    public void Open() {
        claw.setPosition(OPENED);
    }

    public void Close() {
        claw.setPosition(CLOSED);
    }
}
