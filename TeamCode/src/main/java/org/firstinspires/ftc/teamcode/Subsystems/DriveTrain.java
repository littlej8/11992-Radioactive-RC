package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DriveTrain {
    private final DcMotor fl, fr, bl, br;
    private final IMU imu;

    public static double DRIVE_SPEED, TURN_SPEED, TURN_TOLERANCE = 0.2, 0.1, 5.0;

    public DriveTrain(HardwareMap hwMap) {
        this.telemetry = telemetry;

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

    public void TurnTo(double deg) {
        double current = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = Math.abs(deg - current);

        this.fl.setPower(-TURN_SPEED);
        this.fr.setPower(TURN_SPEED);
        this.bl.setPower(-TURN_SPEED);
        this.br.setPower(TURN_SPEED);
                
        while (opModeIsActive() && error > TURN_TOLERANCE) {
            current = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = deg - currrent;
            idle();
        }
        
        this.fl.setPower(0.0);
        this.fr.setPower(0.0);
        this.bl.setPower(0.0);
        this.br.setPower(0.0);
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

        while (opModeIsActive() && (this.fl.isBusy() || this.ffr.isBusy() || this.bl.isBusy() || this.br.isBusy())) {
            idle();
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