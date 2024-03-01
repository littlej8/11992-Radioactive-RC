package org.firstinspires.ftc.teamcode.Unused.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Environment;
import org.firstinspires.ftc.teamcode.Unused.Unfinished.MecanumOdometryController;

@Autonomous
public class OdometryTest extends LinearOpMode {
    private MecanumOdometryController Robot;

    public void runOpMode() {
        initialize();

        waitForStart();

        Robot.DriveTo(new MecanumOdometryController.FieldCoord(0.0, 24.0, 0.0));
        sleep(1000);
        Robot.TurnTo(90.0);
        sleep(1000);
        Robot.TurnTo(0.0);
        sleep(1000);
        Robot.DriveTo(new MecanumOdometryController.FieldCoord(0.0, 3.0, 0.0));
        sleep(1000);
    }

    public void initialize() {
        DcMotor FrontLeft = hardwareMap.get(DcMotor.class, "Frontleft");
        DcMotor FrontRight = hardwareMap.get(DcMotor.class, "Frontright");
        DcMotor BackLeft = hardwareMap.get(DcMotor.class, "Backleft");
        DcMotor BackRight = hardwareMap.get(DcMotor.class, "Backright");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        Robot = new MecanumOdometryController(this, FrontLeft, FrontRight, BackLeft, BackRight, imu);
    }
}