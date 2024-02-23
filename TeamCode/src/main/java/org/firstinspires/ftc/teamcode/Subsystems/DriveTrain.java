package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {
    private final LinearOpMode opMode;
    private final DcMotor fl, fr, bl, br;
    private final IMU imu;
    public static double DRIVE_SPEED = 0.2, TURN_SPEED = 0.1, TURN_TOLERANCE = 5.0;

    public DriveTrain(LinearOpMode opMode, HardwareMap hwMap) {
        this.opMode = opMode;

        this.fl = hwMap.get(DcMotor.class, "Frontleft");
        this.fr = hwMap.get(DcMotor.class, "Frontright");
        this.bl = hwMap.get(DcMotor.class, "Backleft");
        this.br = hwMap.get(DcMotor.class, "Backright");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        this.imu = hwMap.get(IMU.class, "imu");
        this.imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void DriveTeleOp(Gamepad driver) {
        double x = -driver.left_stick_x;
        double y = driver.left_stick_y;
        double turn = driver.right_stick_x / 2; // half turn speed

        this.fl.setPower((-turn + (y + x)) * DRIVE_SPEED);
        this.fr.setPower((turn + (y + x)) * DRIVE_SPEED);
        this.bl.setPower((-turn + (y - x)) * DRIVE_SPEED);
        this.br.setPower((turn + (y - x)) * DRIVE_SPEED);
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

    public void TurnTo(double target) {
        target = Math.toRadians(target);

        double current = Math.toRadians(angleWrap(GetOrientation()));
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

        while (opMode.opModeIsActive() && error > TURN_TOLERANCE) {
            current = Math.toRadians(angleWrap(GetOrientation()));
            error = target - current;
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

        while (opMode.opModeIsActive() && (this.fl.isBusy() || this.fr.isBusy() || this.bl.isBusy() || this.br.isBusy())) {
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