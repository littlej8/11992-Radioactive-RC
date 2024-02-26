package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Claw {
    private final CRServo claw;
    private final DcMotor arm;

    public static double OPENED = 1.0, CLOSED = 0.0;

    public Claw(HardwareMap hwMap) {
        claw = hwMap.get(CRServo.class, "Claw");
        arm = hwMap.get(DcMotor.class, "Arm");
    }

    public void Open() {
        claw.setPower(OPENED);
    }

    public void Close() {
        claw.setPower(CLOSED);
    }
}
