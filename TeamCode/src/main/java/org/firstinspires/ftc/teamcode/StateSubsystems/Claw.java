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

    public static double ARM_POWER = 1.0, TELEOP_ARM_SPEED = 2;
    public static double DROP = -0.1, GRIP = 0.1;
    public static int PULLED_IN = 0, GRABBING = 88;

    private int current_arm_target = PULLED_IN;
    private boolean claw_gripping = false;

    public Claw(HardwareMap hwMap) {
        claw = hwMap.get(CRServo.class, "Claw");
        arm = hwMap.get(DcMotor.class, "Arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);

        arm.setTargetPosition(current_arm_target);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_POWER);
    }

    public void UpdateTeleOp(Gamepad controller) {
        double arm_move = -controller.left_stick_y * TELEOP_ARM_SPEED;
        current_arm_target += arm_move;

        if (controller.left_trigger > 0) {
            DropClaw();
        } else if (controller.right_trigger > 0) {
            GripClaw();
        } else if (!claw_gripping) {
            StopClaw();
        }

        if (-controller.left_stick_y > 0) {
            current_arm_target += TELEOP_ARM_SPEED;
            periodic();
        } else if (-controller.left_stick_y < 0) {
            current_arm_target -= TELEOP_ARM_SPEED;
            periodic();
        }
    }

    public void periodic() {
        arm.setTargetPosition(current_arm_target);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(ARM_POWER);
    }

    public void PullArmIn() {
        current_arm_target = PULLED_IN;
        periodic();
    }

    public void MoveArmToGrab() {
        current_arm_target = GRABBING;
        periodic();
    }

    public void GripClaw() {
        claw.setPower(GRIP);
        claw_gripping = true;
    }

    public void DropClaw() {
        claw.setPower(DROP);
        claw_gripping = false;
    }

    public void StopClaw() {
        claw.setPower(0.0);
        claw_gripping = false;
    }
}
