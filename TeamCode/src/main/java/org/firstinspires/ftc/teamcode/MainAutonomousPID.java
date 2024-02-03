package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Environment;

/**
 *    Concept for autonomous structure
 */
public abstract class MainAutonomousPID extends LinearOpMode {
    private PIDController flPID;
    private PIDController frPID;
    private PIDController blPID;
    private PIDController brPID;

    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    private Servo ClawWrist;
    private Servo ClawGrabber;

    private ColorRangeSensor LeftSensor;
    private ColorRangeSensor RightSensor;

    private double MoveUp;
    private double LeftAdjust;
    private double RightAdjust;
    private double FrontAdjust1;
    private double FrontAdjust2;
    private double MoveBackStraight;
    private double MoveBackLeft;
    private double MoveBackRight;
    private double ParkMove1Straight;
    private double ParkMove1Strafe;
    private double ParkMove2;
    private double ParkMove3;

    abstract public void on_init();

    @Override
    public void runOpMode() throws InterruptedException {
        on_init();

        initialize();

        waitForStart();

        drive_straight(MoveUp);

        String side = check_sensors();
        if (side.equals("left")) {
            turn_left();
            sleep(500);
            drive_straight(LeftAdjust);
        } else if (side.equals("right")) {
            turn_right();
            sleep(500);
            drive_straight(RightAdjust);
        } else {
            strafe_left(FrontAdjust1);
            sleep(500);
            strafe_right(FrontAdjust2);
        }

        ClawWrist.setPosition(0.0);
        sleep(500);
        ClawGrabber.setPosition(0.0);

        sleep(500);

        if (side.equals("left")) {
            strafe_left(MoveBackLeft);
        } else if (side.equals("right")) {
            strafe_right(MoveBackRight);
        } else {
            drive_backwards(MoveBackStraight);
        }

        sleep(500);

        if (side.equals("left")) {
            drive_straight(ParkMove1Straight);
        } else if (side.equals("right")) {
            drive_backwards(ParkMove1Straight);
        } else {
            strafe_right(ParkMove1Strafe);
        }


    }

    public void init_vars(double move_up, double left_adjust, double right_adjust, double front_adjust_1, double front_adjust_2, double move_back_straight, double move_back_left, double move_back_right, double park_move_1_straight, double park_move_1_strafe, double park_move_2, double park_move_3) {
        MoveUp = move_up;
        LeftAdjust = left_adjust;
        RightAdjust = right_adjust;
        FrontAdjust1 = front_adjust_1;
        FrontAdjust2 = front_adjust_2;
        MoveBackStraight = move_back_straight;
        MoveBackLeft = move_back_left;
        MoveBackRight = move_back_right;
        ParkMove1Straight = park_move_1_straight;
        ParkMove1Strafe = park_move_1_strafe;
        ParkMove2 = park_move_2;
        ParkMove3 = park_move_3;
    }

    public void initialize() {
        FrontLeft = hardwareMap.get(DcMotor.class, "Frontleft");
        FrontRight = hardwareMap.get(DcMotor.class, "Frontleft");
        BackLeft = hardwareMap.get(DcMotor.class, "Frontleft");
        BackRight = hardwareMap.get(DcMotor.class, "Frontleft");

        flPID = new PIDController(Environment.Auto.PID_KP, Environment.Auto.PID_KI, Environment.Auto.PID_KD);
        frPID = new PIDController(Environment.Auto.PID_KP, Environment.Auto.PID_KI, Environment.Auto.PID_KD);
        blPID = new PIDController(Environment.Auto.PID_KP, Environment.Auto.PID_KI, Environment.Auto.PID_KD);
        brPID = new PIDController(Environment.Auto.PID_KP, Environment.Auto.PID_KI, Environment.Auto.PID_KD);

        ClawWrist = hardwareMap.get(Servo.class, "ClawX");
        ClawGrabber = hardwareMap.get(Servo.class, "ClawY");

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

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ClawWrist.scaleRange(0.3, 0.75);
        ClawGrabber.scaleRange(0.25, 0.75);

        ClawGrabber.setPosition(1.0);
        ClawWrist.setPosition(1.0);
    }

    public String check_sensors() {
        LeftSensor.setGain(2);
        RightSensor.setGain(2);

        String ret = "front";
        int left_count = 0;
        int right_count = 0;

        for (int i = 0; i < 50; i++) {
            double left_light = LeftSensor.getRawLightDetected();
            double right_light = RightSensor.getRawLightDetected();

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
        drive(Environment.Auto.MovementCounts.DEG90, -Environment.Auto.MovementCounts.DEG90, Environment.Auto.MovementCounts.DEG90, -Environment.Auto.MovementCounts.DEG90);
    }

    public void turn_left() {
        drive(-Environment.Auto.MovementCounts.DEG90, Environment.Auto.MovementCounts.DEG90, -Environment.Auto.MovementCounts.DEG90, Environment.Auto.MovementCounts.DEG90);
    }

    public void drive(double fl, double fr, double bl, double br) {
        double fl_target = FrontLeft.getCurrentPosition() + fl;
        double fr_target = FrontRight.getCurrentPosition() + fr;
        double bl_target = BackLeft.getCurrentPosition() + bl;
        double br_target = BackRight.getCurrentPosition() + br;

        flPID.reset();
        frPID.reset();
        blPID.reset();
        brPID.reset();

        double max_power = Double.POSITIVE_INFINITY;

        while (opModeIsActive() && max_power > Environment.Auto.PID_TOLERANCE) {
            double fl_power = flPID.update(fl_target, FrontLeft.getCurrentPosition());
            double fr_power = frPID.update(fr_target, FrontRight.getCurrentPosition());
            double bl_power = blPID.update(bl_target, BackLeft.getCurrentPosition());
            double br_power = brPID.update(br_target, BackRight.getCurrentPosition());

            max_power = Math.max(Math.max(fl_power, fr_power), Math.max(bl_power, br_power));

            FrontLeft.setPower(fl_power);
            FrontRight.setPower(fr_power);
            BackLeft.setPower(bl_power);
            BackRight.setPower(br_power);
        }
    }
}