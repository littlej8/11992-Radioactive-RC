package org.firstinspires.ftc.teamcode.StateSubsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class DriveTrain {
    private final LinearOpMode opMode;
    private final DcMotor fl, fr, bl, br;
    private final IMU imu;
    private final Telemetry telemetry;
    public static double DRIVE_SPEED = 0.2, TURN_SPEED = 0.1, TURN_TOLERANCE = 5.0;
    public static double TELEOP_SPEED = 0.5, TELEOP_TURN_MULT = 0.5;

    public DriveTrain(LinearOpMode opMode, HardwareMap hwMap, Telemetry telemetry) {
        this.opMode = opMode;
        this.telemetry = telemetry;

        this.fl = hwMap.get(DcMotor.class, "Frontleft");
        this.fr = hwMap.get(DcMotor.class, "Frontright");
        this.bl = hwMap.get(DcMotor.class, "Backleft");
        this.br = hwMap.get(DcMotor.class, "Backright");

        this.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.fl.setDirection(DcMotor.Direction.REVERSE);
        this.fr.setDirection(DcMotor.Direction.REVERSE);
        this.bl.setDirection(DcMotor.Direction.FORWARD);
        this.br.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        this.imu = hwMap.get(IMU.class, "imu");
        this.imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void DriveTeleOp(Gamepad driver) {
        double x = -driver.left_stick_x;
        double y = -driver.left_stick_y;
        double turn = -driver.right_stick_x * TELEOP_TURN_MULT;

        this.fl.setPower((-turn + (y + x)) * TELEOP_SPEED);
        this.fr.setPower((turn + (y + x)) * TELEOP_SPEED);
        this.bl.setPower((-turn + (y - x)) * TELEOP_SPEED);
        this.br.setPower((turn + (y - x)) * TELEOP_SPEED);
    }

    public void DriveForward(double amount) {
        drive(amount, amount, amount, amount);
    }

    public void DriveBackward(double amount) {
        drive(-amount, -amount, -amount, -amount);
    }

    public void DriveLeft(double amount) {
        drive(amount, amount, -amount, -amount);
    }

    public void DriveRight(double amount) {
        drive(-amount, -amount, amount, amount);
    }

    public void TurnTo(double deg_target) {
        double target = Math.toRadians(deg_target);

        double current = angleWrap(Math.toRadians(GetOrientation()));
        double error = target - current;

        if (error > 0) {
            this.fl.setPower(-TURN_SPEED);
            this.fr.setPower(TURN_SPEED);
            this.bl.setPower(-TURN_SPEED);
            this.br.setPower(TURN_SPEED);
        } else if (error < 0) {
            this.fl.setPower(TURN_SPEED);
            this.fr.setPower(-TURN_SPEED);
            this.bl.setPower(TURN_SPEED);
            this.br.setPower(-TURN_SPEED);
        } else {
            return;
        }

        while (opMode.opModeIsActive() && Math.toDegrees(error) > TURN_TOLERANCE) {
            current = angleWrap(Math.toRadians(GetOrientation()));
            error = target - current;

            telemetry.addData("TurnTo Target: ", Math.toDegrees(target));
            telemetry.addData("TurnTo Current: ", Math.toDegrees(current));
            telemetry.addData("TurnTo Error: ", Math.toDegrees(error));
            telemetry.update();

            opMode.idle();
        }

        this.fl.setPower(0.0);
        this.fr.setPower(0.0);
        this.bl.setPower(0.0);
        this.br.setPower(0.0);
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }

    private void drive(double fl, double fr, double bl, double br) {
        int fl_target = (int)(this.fl.getCurrentPosition() + fl);
        int fr_target = (int)(this.fr.getCurrentPosition() + fr);
        int bl_target = (int)(this.bl.getCurrentPosition() + bl);
        int br_target = (int)(this.br.getCurrentPosition() + br);

        this.fl.setTargetPosition(fl_target);
        this.fr.setTargetPosition(fr_target);
        this.bl.setTargetPosition(bl_target);
        this.br.setTargetPosition(br_target);

        this.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.fl.setPower(DRIVE_SPEED);
        this.fr.setPower(DRIVE_SPEED);
        this.bl.setPower(DRIVE_SPEED);
        this.br.setPower(DRIVE_SPEED);

        while (opMode.opModeIsActive() && (this.fl.isBusy() && this.fr.isBusy() && this.bl.isBusy() && this.br.isBusy())) {
            opMode.idle();
        }

        this.fl.setPower(0.0);
        this.fr.setPower(0.0);
        this.bl.setPower(0.0);
        this.br.setPower(0.0);
        
        this.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ResetOrientation() {
        imu.resetYaw();
    }

    public double GetOrientation() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}