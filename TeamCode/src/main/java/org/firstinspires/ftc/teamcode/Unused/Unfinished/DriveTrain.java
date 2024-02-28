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

    private double turn_target;
    private int fl_move_target;
    private int fr_move_target;
    private int bl_move_target;
    private int br_move_target;

    public DriveTrain(LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        HardwareMap hwMap = opMode.hardwareMap;

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

    private double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }

    private void UpdateTurn() {
        double current = angleWrap(Math.toRadians(GetOrientation()));
        double error = turn_target - current;

        telemetry.addData("TurnTo Target: ", Math.toDegrees(turn_target));
        telemetry.addData("TurnTo Current: ", Math.toDegrees(current));
        telemetry.addData("TurnTo Error: ", Math.toDegrees(error));
        telemetry.update();

        if (Math.toDegrees(error) > TURN_TOLERANCE) {
            this.fl.setPower(-TURN_SPEED);
            this.fr.setPower(TURN_SPEED);
            this.bl.setPower(-TURN_SPEED);
            this.br.setPower(TURN_SPEED);
        } else if (Math.toDegrees(error) < TURN_TOLERANCE) {
            this.fl.setPower(TURN_SPEED);
            this.fr.setPower(-TURN_SPEED);
            this.bl.setPower(TURN_SPEED);
            this.br.setPower(-TURN_SPEED);
        } else {
            this.fl.setPower(0);
            this.fr.setPower(0);
            this.bl.setPower(0);
            this.br.setPower(0);

            turn_target = 0;
        }
    }

    private void UpdateMove() {
        if (this.fl.isBusy() && this.fr.isBusy() && this.bl.isBusy() && this.br.isBusy()) {
            this.fl.setTargetPosition(fl_move_target);
            this.fr.setTargetPosition(fr_move_target);
            this.bl.setTargetPosition(bl_move_target);
            this.br.setTargetPosition(br_move_target);

            this.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            this.fl.setPower(DRIVE_SPEED);
            this.fr.setPower(DRIVE_SPEED);
            this.bl.setPower(DRIVE_SPEED);
            this.br.setPower(DRIVE_SPEED);
        } else {
            this.fl.setPower(0.0);
            this.fr.setPower(0.0);
            this.bl.setPower(0.0);
            this.br.setPower(0.0);

            this.fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            fl_move_target = 0;
            fr_move_target = 0;
            bl_move_target = 0;
            br_move_target = 0;
        }
    }

    public boolean isActive() {
        return (turn_target == 0) &&
                (fl_move_target == 0) &&
                (fr_move_target == 0) &&
                (bl_move_target == 0) &&
                (br_move_target == 0);
    }

    public void periodic() {
        if (turn_target == 0) {
            UpdateMove();
        } else {
            UpdateTurn();
        }
    }

    public void ScheduleDriveForward(double amount) {
        ScheduleDrive(amount, amount, amount, amount);
    }

    public void ScheduleDriveBackward(double amount) {
        ScheduleDrive(-amount, -amount, -amount, -amount);
    }

    public void ScheduleDriveLeft(double amount) {
        ScheduleDrive(amount, amount, -amount, -amount);
    }

    public void ScheduleDriveRight(double amount) {
        ScheduleDrive(-amount, -amount, amount, amount);
    }

    public void ScheduleTurnTo(double deg_target) {
        turn_target = Math.toRadians(deg_target);
    }

    private void ScheduleDrive(double fl, double fr, double bl, double br) {
        fl_move_target = (int)(this.fl.getCurrentPosition() + fl);
        fr_move_target = (int)(this.fr.getCurrentPosition() + fr);
        bl_move_target = (int)(this.bl.getCurrentPosition() + bl);
        br_move_target = (int)(this.br.getCurrentPosition() + br);
    }

    public void ResetOrientation() {
        imu.resetYaw();
    }

    public double GetOrientation() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}