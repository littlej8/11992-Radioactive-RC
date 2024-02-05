package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Environment;
import org.firstinspires.ftc.teamcode.Util.PID;
import org.firstinspires.ftc.teamcode.Util.MecanumOdometryController;

@Autonomous
public class PIDTest extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    private IMU imu;

    private MecanumOdometryController Robot;

    private PID xPID = new PID(Environment.Auto.PID_KP, Environment.Auto.PID_KI, Environment.Auto.PID_KD, Environment.Auto.MAX_ACCELERATION, Environment.Auto.MAX_VELOCITY);
    private PID yPID = new PID(Environment.Auto.PID_KP, Environment.Auto.PID_KI, Environment.Auto.PID_KD, Environment.Auto.MAX_ACCELERATION, Environment.Auto.MAX_VELOCITY);
    private PID thetaPID = new PID(Environment.Auto.PID_KP, Environment.Auto.PID_KI, Environment.Auto.PID_KD, Environment.Auto.MAX_TURN_ACCELERATION, Environment.Auto.MAX_TURN_VELOCITY);

    public void runOpMode() {
        initialize();

        waitForStart();
        imu.resetYaw();


    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }

    public void initialize() {
        FrontLeft = hardwareMap.get(DcMotor.class, "Frontleft");
        FrontRight = hardwareMap.get(DcMotor.class, "Frontright");
        BackLeft = hardwareMap.get(DcMotor.class, "Backleft");
        BackRight = hardwareMap.get(DcMotor.class, "Backright");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Robot = new MecanumOdometryController(this, FrontLeft, FrontRight, BackLeft, BackRight, imu);
    }

    public void arrived(double x, double y, double heading) {
        MecanumOdometryController.FieldCoord cur = Robot.getPosition();

        double max_error = Math.max(x - cur.X, y - cur.Y);
        return (max_error < Environment.Auto.ODOMETRY_TOLERANCE) && (heading - cur.Heading < Environment.Auto.TURN_TOLERANCE);
    }

    public void DriveTo(double x, double y, double heading) {
        xPID.reset();
        yPID.reset();
        thetaPID.reset();

        while (opModeIsActive() && !(arrived(x, y, heading))) {
            MecanumOdometryController.FieldCoord cur = Robot.getPosition();

            double xPower = xPID.update(x, cur.X, false);
            double yPower = yPID.update(y, cur.Y, false);
            double tPower = thetaPID.update(heading, cur.Heading, true);

            double angle = Math.toRadians(cur.Heading);
            double xRotated = xPower * Math.cos(angle) - yPower * Math.sin(angle);
            double yRotated = xPower * Math.sin(angle) + yPower * Math.cos(angle);

            FrontLeft.setPower(xRotated + yRotated + tPower);
            FrontRight.setPower(xRotated - yRotated - tPower);
            BackLeft.setPower(xRotated - yRotated + tPower);
            BackRight.setPower(xRotated + yRotated - tPower);

            telemetry.update();
        }

        FrontLeft.setPower(0.0);
        FrontRight.setPower(0.0);
        BackLeft.setPower(0.0);
        BackRight.setPower(0.0);

        telemetry.update();
    }
}