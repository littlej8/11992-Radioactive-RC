package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Lift {
    private final DcMotor left, right;

    public Lift(HardwareMap hwMap) {
        left = hwMap.get(Servo.class, "LifterArm1");
        right = hwMap.get(Servo.class, "LifterArm2");

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setPower(0.0);
        right.setPower(0.0);
    }

    public void StartLifting() {
        left.setPower(Environment.TeleOp.LIFT_POWER);
        right.setPower(Environment.TeleOp.LIFT_POWER);
    }

    public void PullLegsUp() {
        left.setPower(-Environment.TeleOp.LIFT_POWER);
        right.setPower(-Environment.TeleOp.LIFT_POWER);
    }

    public void StopLifting() {
        left.setPower(0.0);
        right.setPower(0.0);
    }

    public void UpdateTeleOp(Gamepad controller) {
        if (controller.dpad_up) {
            StartLifting();
        } else if (controller.dpad_down) {
            PullLegsUp();
        } else {
            StopLifting();
        }
    }
}
