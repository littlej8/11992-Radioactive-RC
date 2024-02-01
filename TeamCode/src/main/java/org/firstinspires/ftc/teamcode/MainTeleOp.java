package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MainTeleOp extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    private DcMotor Lift;
    private DcMotor Arm;
    private DcMotor LifterArm1;
    private DcMotor LifterArm2;

    private Servo ClawWrist;
    private Servo ClawGrabber;
    private Servo DroneLauncher;

    @Override
    public void runOpMode() throws InterruptedException {
        init();

        waitForStart();

        while (opModeIsActive()) {
            loop();
        }
    }

    public void init() {
        FrontLeft = hardwareMap.get(DcMotor.class, "Frontleft");
        FrontRight = hardwareMap.get(DcMotor.class, "Frontright");
        BackLeft = hardwareMap.get(DcMotor.class, "Backleft");
        BackRight = hardwareMap.get(DcMotor.class, "Backright");

        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        LifterArm1 = hardwareMap.get(DcMotor.class, "LifterArm1");
        LifterArm2 = hardwareMap.get(DcMotor.class, "LifterArm2");

        ClawWrist = hardwareMap.get(Servo.class, "ClawX");
        ClawGrabber = hardwareMap.get(Servo.class, "ClawY");
        DroneLauncher = hardwareMap.get(Servo.class, "Airplane Launcher");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

        LifterArm1.setDirection(DcMotor.Direction.REVERSE);
        LifterArm2.setDirection(DcMotor.Direction.FORWARD);

        ClawWrist.setDirection(Servo.Direction.FORWARD);
        ClawGrabber.setDirection(Servo.Direction.FORWARD);
        DroneLauncher.setDirection(Servo.Direction.FORWARD);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LifterArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LifterArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LifterArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LifterArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft.setPower(0.0);
        FrontRight.setPower(0.0);
        BackLeft.setPower(0.0);
        BackRight.setPower(0.0);

        Lift.setPower(0.0);
        Arm.setPower(0.0);
        LifterArm1.setPower(0.0);
        LifterArm2.setPower(0.0);

        ClawWrist.scaleRange(Environment.TeleOp.CLAW_WRIST_MIN, Environment.TeleOp.CLAW_WRIST_MAX);
        ClawGrabber.scaleRange(Environment.TeleOp.CLAW_GRABBER_MIN, Environment.TeleOp.CLAW_GRABBER_MAX);
    }

    public void loop() {
        UpdateWheels();
        UpdateArm();
        UpdateClaw();
        UpdateLift();

        if (gamepad2.triangle) {
            RobotRoutine(); // routine yayyyyy
        }

        telemetry.update();
    }

    private void UpdateWheels() {
        double horizontal = -gamepad1.left_stick_x;
        double vertical = gamepad1.left_stick_y;
        double pivot = gamepad1.right_stick_x / 2; // half turn speed

        FrontLeft.setPower((-pivot + (vertical + horizontal)) * Environment.TeleOp.WHEEL_POWER);
        FrontRight.setPower((pivot + (vertical + horizontal)) * Environment.TeleOp.WHEEL_POWER);
        BackLeft.setPower((-pivot + (vertical - horizontal)) * Environment.TeleOp.WHEEL_POWER);
        BackRight.setPower((pivot + (vertical - horizontal)) * Environment.TeleOp.WHEEL_POWER);
    }

    private void UpdateArm() {
        double left_y = gamepad2.left_stick_y;
        double right_y = gamepad1.right_stick_y;

        if (left_y != 0) {
            Arm.setPower(left_y * Environment.TeleOp.ARM_POWER);
        } else {
            Arm.setPower(0.0);
        }

        if (right_y != 0) {
            Lift.setPower(right_y * Environment.TeleOp.ARM_POWER);
        } else {
            Lift.setPower(0.0);
        }
    }

    private void UpdateClaw() {
        if (gamepad2.right_trigger > 0) {
            ClawWrist.setPosition(ClawWrist.getPosition() + Environment.TeleOp.CLAW_WRIST_SPEED);
        } else if (gamepad2.left_trigger > 0) {
            ClawWrist.setPosition(ClawWrist.getPosition() - Environment.TeleOp.CLAW_WRIST_SPEED);
        }

        if (gamepad2.left_bumper) {
            ClawGrabber.setPosition(ClawGrabber.getPosition() + Environment.TeleOp.CLAW_GRABBER_SPEED);
        } else if (gamepad2.right_bumper) {
            ClawGrabber.setPosition(ClawGrabber.getPosition() + Environment.TeleOp.CLAW_GRABBER_SPEED);
        }
    }

    private void UpdateDrone() {
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
            DroneLauncher.setPosition(1.0);
            sleep(1000);
            DroneLauncher.setPosition(0.0);
        }
    }

    private void UpdateLift() {
        if (gamepad2.dpad_up) {
            LifterArm1.setPower(Environment.TeleOp.LIFT_POWER);
            LifterArm2.setPower(Environment.TeleOp.LIFT_POWER);
        } else if (gamepad2.dpad_down) {
            LifterArm1.setPower(-Environment.TeleOp.LIFT_POWER);
            LifterArm2.setPower(-Environment.TeleOp.LIFT_POWER);
        } else {
            LifterArm1.setPower(0.0);
            LifterArm2.setPower(0.0);
        }
    }

    private void RobotRoutine() {
        ClawWrist.setPosition(0.5);
        ClawGrabber.setPosition(0.0);

        WheelMoveTime(15);
        sleep(500);

        ClawWrist.setPosition(0.0);
        sleep(500);

        ClawGrabber.setPosition(1.0);
    }

    private void WheelMoveTime(long time) {
        FrontLeft.setPower(Environment.TeleOp.WHEEL_POWER);
        FrontRight.setPower(Environment.TeleOp.WHEEL_POWER);
        BackLeft.setPower(Environment.TeleOp.WHEEL_POWER);
        BackRight.setPower(Environment.TeleOp.WHEEL_POWER);

        sleep(time);

        FrontLeft.setPower(0.0);
        FrontRight.setPower(0.0);
        BackLeft.setPower(0.0);
        BackRight.setPower(0.0);
    }
}