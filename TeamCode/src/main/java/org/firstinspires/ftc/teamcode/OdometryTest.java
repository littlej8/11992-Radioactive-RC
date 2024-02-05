package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Environment;
import org.firstinspires.ftc.teamcode.Util.MecanumOdometryController;

@Autonomous
public class PIDTest extends LinearOpMode {
    private MecanumOdometryController Robot;

    public void runOpMode() {
        initialize();

        waitForStart();
        imu.resetYaw();

        Robot.DriveTo(0.0, 24.0, 0.0);
        sleep(1000);
        Robot.TurnTo(90.0);
        sleep(1000);
        Robot.TurnTo(0.0);
        sleep(1000);
        Robot.DriveTo(0.0, 3.0, 0.0);
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

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Robot = new MecanumOdometryController(this, FrontLeft, FrontRight, BackLeft, BackRight, imu);
    }
}