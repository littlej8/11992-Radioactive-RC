package org.firstinspires.ftc.teamcode.StateSubsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Claw {
    private final CRServo claw;
    private final DcMotor arm;

    public static double ARM_POWER = 0.1, TELEOP_ARM_SPEED = 2;
    public static double DROP = -0.01, GRIP = 0.05;
    public static int PULLED_IN = 0, GRABBING = 88;

    public int current_arm_target = PULLED_IN;

    public Claw(HardwareMap hwMap) {
        claw = hwMap.get(CRServo.class, "Claw");
        arm = hwMap.get(DcMotor.class, "Arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(current_arm_target);
    }

    public void UpdateTeleOp(Gamepad controller) {
        double arm_move = -controller.left_stick_y * TELEOP_ARM_SPEED;
        current_arm_target += arm_move;

        if (controller.left_trigger > 0) {
            DropClaw();
        } else if (controller.right_trigger > 0) {
            GripClaw();
        }

        periodic();
    }

    public void periodic() {
        arm.setTargetPosition(current_arm_target);
    }

    public void PullArmIn() {
        current_arm_target = PULLED_IN;
    }

    public void MoveArmToGrab() {
        current_arm_target = GRABBING;
    }

    public void GripClaw() {
        claw.setPower(GRIP);
    }

    public void DropClaw() {
        claw.setPower(DROP);
    }
}
