package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Environment;
import org.firstinspires.ftc.teamcode.Util.PID;

@Autonomous
public class PIDTest extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    private PID FLPID = new PID(Environment.Auto.PID_KP, Environment.Auto.PID_KI, Environment.Auto.PID_KD, Environment.Auto.MAX_ACCELERATION, Environment.Auto.MAX_VELOCITY);
    private PID FRPID = new PID(Environment.Auto.PID_KP, Environment.Auto.PID_KI, Environment.Auto.PID_KD, Environment.Auto.MAX_ACCELERATION, Environment.Auto.MAX_VELOCITY);
    private PID BLPID = new PID(Environment.Auto.PID_KP, Environment.Auto.PID_KI, Environment.Auto.PID_KD, Environment.Auto.MAX_ACCELERATION, Environment.Auto.MAX_VELOCITY);
    private PID BRPID = new PID(Environment.Auto.PID_KP, Environment.Auto.PID_KI, Environment.Auto.PID_KD, Environment.Auto.MAX_ACCELERATION, Environment.Auto.MAX_VELOCITY);

    private IMU imu;
    private PID imuPID = new PID(Environment.Auto.PID_KP, Environment.Auto.PID_KI, Environment.Auto.PID_KD, Environment.Auto.MAX_TURN_ACCELERATION, Environment.Auto.MAX_TURN_VELOCITY);

    public void runOpMode() {
        initialize();

        waitForStart();
        imu.resetYaw();

        drive_forward(Environment.COUNTS_PER_INCH * 24.0);
        sleep(1000);
        turn_to(90.0);
        sleep(1000);
        turn_to(0.0)
        sleep(1000);
        drive_backwards(Environment.COUNTS_PER_INCH * 21.0);
        sleep(1000);
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

    public boolean at_position(double fl, double fr, double bl, double br) {
        double max_error = Math.max(Math.max(fl - FrontLeft.getCurrentPosition(), fr - FrontRight.getCurrentPosition()), Math.max(bl - BackLeft.getCurrentPosition(), br - BackRight.getCurrentPosition));
        return max_error <= Environment.Auto.PID_TOLERANCE;
    }

    public void turn_to(double deg) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double cur = orientation.getYaw(AngleUnit.DEGREES);
        double target = deg;
        
        double error = Math.toDegrees(angleWrap(Math.toRadians(target - cur)));
        
        while (opModeIsActive() && Math.abs(error) > Environment.Auto.TURN_TOLERANCE) {
            orientation = imu.getRobotYawPitchRollAngles();
            cur = orientation.getYaw(AngleUnit.DEGREES);
            error = Math.toDegrees(angleWrap(Math.toRadians(target - cur)));
            
            double turn_power = imuPID.update(target, cur, true);

            FrontLeft.setPower(turn_power);
            FrontRight.setPower(-turn_power);
            BackLeft.setPower(turn_power);
            BackRight.setPower(-turn_power);
        }
        
        FrontLeft.setPower(0.0);
        FrontRight.setPower(0.0);
        BackLeft.setPower(0.0);
        BackRight.setPower(0.0);
        
        telemetry.update();
    }

    public void drive_forward(double amount) {
        drive(amount, amount, amount, amount);
    }

    public void drive_backwards(double amount) {
        drive(-amount, -amount, -amount, -amount);
    }

    public void drive(double fl, double fr, double bl, double br) {
        FLPID.reset();
        FRPID.reset();
        BLPID.reset();
        BRPID.reset();

        while (opModeIsActive() && !at_position(fl, fr, bl, br)) {
            FrontLeft.setPower(FLPID.update(fl, FrontLeft.getCurrentPosition(), false));
            FrontRight.setPower(FRPID.update(fr, FrontRight.getCurrentPosition(), false));
            BackLeft.setPower(BLPID.update(bl, BackLeft.getCurrentPosition(), false));
            BackRight.setPower(BRPID.update(br, BackRight.getCurrentPosition(), false));

            telemetry.update();
        }

        FrontLeft.setPower(0.0);
        FrontRight.setPower(0.0);
        BackLeft.setPower(0.0);
        BackRight.setPower(0.0);

        telemetry.update()
    }
}