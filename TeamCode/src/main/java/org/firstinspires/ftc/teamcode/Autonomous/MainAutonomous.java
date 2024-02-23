package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.Util.Environment;
import org.firstinspires.ftc.teamcode.Util.MecanumOdometryController;

public abstract class MainAutonomous extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    private Servo ClawWrist;
    private Servo ClawGrabber;

    private ColorRangeSensor LeftSensor;
    private ColorRangeSensor RightSensor;
    
    private IMU imu;

    private MecanumOdometryController RobotPosition;

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
    private double ParkMove2Straight = 0.0;
    private double ParkMove2Strafe = 0.0;
    private double ParkMove3Straight = 0.0;
    private double ParkMove3Strafe = 0.0;

    abstract public void on_init();

    @Override
    public void runOpMode() {
        on_init();

        initialize();

        waitForStart();
        
        drive_straight(MoveUp);
        
        sleep(500);

        String side = check_sensors();
        
        telemetry.addData("side: ", side);
        telemetry.update();
        
        sleep(500);
        
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
            drive_backwards(FrontAdjust2);
        }
        
        sleep(500);
        ClawWrist.setPosition(0.0);
        sleep(1000);
        ClawGrabber.setPosition(0.0);
        sleep(1000);
        ClawWrist.setPosition(0.75);
        sleep(500);
        ClawGrabber.setPosition(1.0);
        sleep(500);

        if (side.equals("left")) {
            strafe_left(MoveBackLeft);
        } else if (side.equals("right")) {
            strafe_right(MoveBackRight);
        } else {
            drive_backwards(MoveBackStraight);
        }
        
        /*
        if (ParkMove2Straight == 0) {
            sleep(500);
        } else {
            sleep(10000);
        }

        if (side.equals("left")) {
            drive_backwards(ParkMove1Straight);
        } else if (side.equals("right")) {
            drive_straight(ParkMove1Straight);
        } else {
            strafe_right(ParkMove1Strafe);
        }

        if (ParkMove2Straight == 0) {
            return;
        }
        
        sleep(500);

        if (side.equals("left")) {
            strafe_right(ParkMove2Strafe);
        } else if (side.equals("right")) {
            strafe_left(ParkMove2Strafe);
        } else {
            drive_straight(ParkMove2Straight);
        }
        
        sleep(500);

        if (side.equals("left")) {
            drive_backwards(ParkMove3Straight);
        } else if (side.equals("right")) {
            drive_straight(ParkMove3Straight);
        } else {
            strafe_right(ParkMove3Strafe);
        }
        */
    }

    public void init_vars(double move_up, double left_adjust, double right_adjust, double front_adjust_1, double front_adjust_2, double move_back_straight, double move_back_left, double move_back_right, double park_move_1_straight, double park_move_1_strafe) {
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
    }

    public void init_vars(double move_up, double left_adjust, double right_adjust, double front_adjust_1, double front_adjust_2, double move_back_straight, double move_back_left, double move_back_right, double middle_move_1_straight, double middle_move_1_strafe, double middle_move_2_straight, double middle_move_2_strafe, double middle_move_3_straight, double middle_move_3_strafe) {
        MoveUp = move_up;
        LeftAdjust = left_adjust;
        RightAdjust = right_adjust;
        FrontAdjust1 = front_adjust_1;
        FrontAdjust2 = front_adjust_2;
        MoveBackStraight = move_back_straight;
        MoveBackLeft = move_back_left;
        MoveBackRight = move_back_right;
        ParkMove1Straight = middle_move_1_straight;
        ParkMove1Strafe = middle_move_1_strafe;
        ParkMove2Straight = middle_move_2_straight;
        ParkMove2Strafe = middle_move_2_strafe;
        ParkMove3Straight = middle_move_3_straight;
        ParkMove3Strafe = middle_move_3_strafe;
    }

    public void initialize() {
        FrontLeft = hardwareMap.get(DcMotor.class, "Frontleft");
        FrontRight = hardwareMap.get(DcMotor.class, "Frontright");
        BackLeft = hardwareMap.get(DcMotor.class, "Backleft");
        BackRight = hardwareMap.get(DcMotor.class, "Backright");

        ClawWrist = hardwareMap.get(Servo.class, "ClawX");
        ClawGrabber = hardwareMap.get(Servo.class, "ClawY");

        LeftSensor = hardwareMap.get(ColorRangeSensor.class, "Left Sensor");
        RightSensor = hardwareMap.get(ColorRangeSensor.class, "Right Sensor");
        
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

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

        ClawGrabber.setPosition(1.0);
        sleep(1000);
        ClawWrist.setPosition(1.0);
    }

    public String check_sensors() {
        LeftSensor.setGain(2);
        RightSensor.setGain(2);

        String ret = "front";
        int left_count = 0;
        int right_count = 0;

        for (int i = 0; i < 50 && opModeIsActive(); i++) {
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
            
            sleep(10);
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
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double cur = orientation.getYaw(AngleUnit.DEGREES);
        double target = cur - 90.0;
        
        double error = target - cur;
        
        FrontLeft.setPower(Environment.Auto.TURN_POWER);
        FrontRight.setPower(-Environment.Auto.TURN_POWER);
        BackLeft.setPower(Environment.Auto.TURN_POWER);
        BackRight.setPower(-Environment.Auto.TURN_POWER);
        
        telemetry.update();
        
        while (opModeIsActive() && Math.abs(error) > Environment.Auto.TURN_TOLERANCE) {
            orientation = imu.getRobotYawPitchRollAngles();
            cur = orientation.getYaw(AngleUnit.DEGREES);
            error = target - cur;
            idle();
        }
        
        FrontLeft.setPower(0.0);
        FrontRight.setPower(0.0);
        BackLeft.setPower(0.0);
        BackRight.setPower(0.0);
        
        telemetry.update();
    }

    public void turn_left() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double cur = orientation.getYaw(AngleUnit.DEGREES);
        double target = cur + 90.0;
        
        double error = target - cur;
        
        FrontLeft.setPower(-Environment.Auto.TURN_POWER);
        FrontRight.setPower(Environment.Auto.TURN_POWER);
        BackLeft.setPower(-Environment.Auto.TURN_POWER);
        BackRight.setPower(Environment.Auto.TURN_POWER);
        
        telemetry.update();
        
        while (opModeIsActive() && Math.abs(error) > Environment.Auto.TURN_TOLERANCE) {
            orientation = imu.getRobotYawPitchRollAngles();
            cur = orientation.getYaw(AngleUnit.DEGREES);
            error = target - cur;
            idle();
        }
        
        FrontLeft.setPower(0.0);
        FrontRight.setPower(0.0);
        BackLeft.setPower(0.0);
        BackRight.setPower(0.0);
        
        telemetry.update();
    }

    public void drive(double fl, double fr, double bl, double br) {
        int fl_target = (int)(FrontLeft.getCurrentPosition() + fl);
        int fr_target = (int)(FrontRight.getCurrentPosition() + fr);
        int bl_target = (int)(BackLeft.getCurrentPosition() + bl);
        int br_target = (int)(BackRight.getCurrentPosition() + br);

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

        while (opModeIsActive() && (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy())) {
            idle();
        }

        FrontLeft.setPower(0.0);
        FrontRight.setPower(0.0);
        BackLeft.setPower(0.0);
        BackRight.setPower(0.0);
        
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();
    }
}