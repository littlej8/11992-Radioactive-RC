package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.config.Config;

@Config
public class OldClaw {
    private final CRServo claw;
    private final DcMotor arm;

    public static double ARM_POWER = 0.1;
    public static double DROP = -0.01, GRIP = 0.05;
    public static double PULLED_IN = 0, GRABBING = 88;

    public OldClaw(HardwareMap hwMap) {
        claw = hwMap.get(CRServo.class, "Claw");
        arm = hwMap.get(DcMotor.class, "Arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
    }

    public void PullArmIn() {
        arm.setTargetPosition(PULLED_IN);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_POWER);
    }

    public void MoveArmToGrab() {
        arm.setTargetPosition(GRABBING);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_POWER);
    }

    public void GripClaw() {
        claw.setPower(GRIP);
    }

    public void DropClaw() {
        claw.setPower(DROP);
    }
}
