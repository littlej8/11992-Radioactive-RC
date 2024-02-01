package org.firstinspires.ftc.teamcode;

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

    private PIDController flPID;
    private PIDController frPID;
    private PIDController blPID;
    private PIDController brPID;

    private Servo ClawWrist;
    private Servo ClawGrabber;

    private double MoveUp;
    private double MoveBack;
    private double ParkMove1;
    private double ParkMove2;
    private double ParkMove3;

    abstract public void on_init();

    public void runOpMode() throws InterruptedException {
        on_init();

        init();

        waitForStart();
    }

    public void init_vars(double move_up, double move_back, double park_move_1, double park_move_2, double park_move_3) {
        MoveUp = move_up;
        MoveBack = move_back;
        ParkMove1 = park_move;
        ParkMove2 = park_move_2;
        ParkMove3 = park_move_3;
    }

    public void init() {
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

    public void drive(double fl, double fr, double bl, double br) {
        double fl_target = FrontLeft.getPosition() + fl;
        double fr_target = FrontRight.getPosition() + fr;
        double bl_target = BackLeft.getPosition() + bl;
        double br_target = BackRight.getPosition() + br;

        flPID.reset();
        frPID.reset();
        blPID.reset();
        brPID.reset();

        double max_power = Double.POSITIVE_INFINITY;

        while (opModeIsActive() && max_power > Environment.Auto.PID_TOLERANCE) {
            double fl_power = flPID.update(fl_target, FrontLeft.getPosition());
            double fr_power = frPID.update(fr_target, FrontRight.getPosition());
            double bl_power = blPID.update(bl_target, BackLeft.getPosition());
            double br_power = brPID.update(br_target, BackRight.getPosition());

            max_power = Math.max(Math.max(fl_power, fr_power), Math.max(bl_power, br_power));

            FrontLeft.setPower(fl_power);
            FrontRight.setPower(fr_power);
            BackLeft.setPower(bl_power);
            BackRight.setPower(br_power);
        }
    }
}