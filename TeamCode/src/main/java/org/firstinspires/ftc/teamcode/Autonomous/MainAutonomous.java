package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Environment;

/**
 *    Concept for autonomous structure
 */
public abstract class MainAutonomous extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    private Servo ClawWrist;
    private Servo ClawGrabber;

    private ColorSensor LeftSensor;
    private ColorSensor RightSensor;

    private double MoveUp;
    private double MoveBack;
    private double ParkMove1;
    private double ParkMove2;
    private double ParkMove3;

    abstract public void on_init();

    public void runOpMode() throws InterruptedException {
        on_init();

        initialize();

        waitForStart();

        String side = check_sensors();
        if (side == "left") {
            turn_left();
            drive_straight();
        }
    }

    public void init_vars(double move_up, double move_back, double park_move_1, double park_move_2, double park_move_3) {
        MoveUp = move_up;
        MoveBack = move_back;
        ParkMove1 = park_move_1;
        ParkMove2 = park_move_2;
        ParkMove3 = park_move_3;
    }

    public void initialize() {
        FrontLeft = hardwareMap.get(DcMotor.class, "Frontleft");
        FrontRight = hardwareMap.get(DcMotor.class, "Frontleft");
        BackLeft = hardwareMap.get(DcMotor.class, "Frontleft");
        BackRight = hardwareMap.get(DcMotor.class, "Frontleft");

        ClawWrist = hardwareMap.get(Servo.class, "ClawX");
        ClawGrabber = hardwareMap.get(Servo.class, "ClawY");

        LeftSensor = hardwareMap.get(ColorSensor.class, "Left Sensor");
        RightSensor = hardwareMap.get(ColorSensor.class, "Right Sensor");

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        ClawWrist.setDirection(Servo.Direction.FORWARD);
        ClawGrabber.setDirection(Servo.Direction.FORWARD);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ClawWrist.scaleRange(0.3, 0.75);
        ClawGrabber.scaleRange(0.25, 0.75);

        ClawGrabber.setPosition(1.0);
        ClawWrist.setPosition(1.0);
    }

    public String check_sensors() {
        LeftSensor.Gain = 2;
        RightSensor.Gain = 2;

        String ret = "front";
        int left_count = 0;
        int right_count = 0;

        for (int i = 0; i < 50; i++) {
            double left_light = LeftSensor.RawLightDetected;
            double right_light = RightSensor.RawLightDetected;

            if (left_light >= Environment.Auto.LEFT_BOUND) {
                left_count++;
            } else {
                left_count = 0;
            }

            if (right_light >= Environment.Auto.RIGHT_BOUND) {
                right_count++;
            } else {
                right_count = 0;
            }

            if (left_count >= Environment.Auto.LIGHT_SENSITIVITY) {
                ret = "left";
            } else if (right_count >= Environment.Auto.LIGHT_SENSITIVITY) {
                ret = "right";
            }
        }

        return ret;
    }

    public void drive_straight(double amount) {
        drive(amount, amount, amount, amount);
    }

    public void drive_backwards(double amount) {
        drive(-amount, -amount, -amount, -amount);
    }

    public void strafe_right(double amount) {
        drive(amount, amount, -amount, -amount);
    }

    public void strafe_left(double amount) {
        drive(-amount, -amount, amount, amount);
    }

    public void turn_right() {
        drive(Environment.Auto.MovementCounts.DEG90, -Environment.MovementCounts.Auto.DEG90, Environment.MovementCounts.Auto.DEG90, -Environment.MovementCounts.Auto.DEG90);
    }

    public void turn_left() {
        drive(-Environment.Auto.MovementCounts.DEG90, Environment.MovementCounts.Auto.DEG90, -Environment.MovementCounts.Auto.DEG90, Environment.MovementCounts.Auto.DEG90);
    }

    public void drive(double fl, double fr, double bl, double br) {
        double fl_target = FrontLeft.getCurrentPosition() + fl;
        double fr_target = FrontRight.getCurrentPosition() + fr;
        double bl_target = BackLeft.getCurrentPosition() + bl;
        double br_target = BackRight.getCurrentPosition() + br;

        FrontLeft.setTargetPosition(fl_target);
        FrontRight.setTargetPosition(fr_target);
        BackLeft.setTargetPosition(bl_target);
        BackRight.setTargetPosition(br_target);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setPower(Environment.Auto.WHEEL_POWER);
        FrontRight.setPower(Environment.Auto.WHEEL_POWER);
        BackLeft.setPower(Environment.Auto.WHEEL_POWER);
        BackRight.setPower(Environment.Auto.WHEEL_POWER);

        telemetry.update();

        while (opModeIsActive() && (FrontLeft.isBusy() || FrontRight.isBusy() || BackLeft.isBusy() || BackRight.isBusy())) {
            idle();
        }

        sleep(50);

        FrontLeft.setPower(0.0);
        FrontRight.setPower(0.0);
        BackLeft.setPower(0.0);
        BackRight.setPower(0.0);

        telemetry.update();
    }
}