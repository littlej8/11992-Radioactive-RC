package org.firstinspires.ftc.teamcode.Unused.Unfinished;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// https://www.reddit.com/r/FTC/comments/10mzfaf/how_can_odometry_be_performed_on_mechanum_wheels/

/* field centric model (relative to robot start position)
     ------------ blue
   ↑ |  r    r  | robots facing ↓
   | |          |
   | |  r    r  | robots facing ↑
   y ------------ red
     x -------→
   positions = (x, y)
 */

/* robot centric model (forward is +x and left is +y)
    --0------0--
  ↑ |          | front
  | |    *     | ---->
  | |          |
  y --0------0--
   x -------→
 */

public class MecanumOdometryController {
    private final LinearOpMode Opmode;
    private final DcMotorEx FrontLeft;
    private final DcMotorEx FrontRight;
    private final DcMotorEx BackLeft;
    private final DcMotorEx BackRight;
    private final IMU Imu;

    private double LastFL;
    private double LastFR;
    private double LastBL;
    private double LastBR;
    private double LastHeading;

    /**
     * A coordinate on the field in inches relative to the robot's starting position
     */
    public static class FieldCoord {
        public double X;
        public double Y;
        public double Heading;

        public FieldCoord(double x, double y, double heading) {
            X = x;
            Y = y;
            Heading = heading;
        }
    }

    private final FieldCoord RobotPos;

    public MecanumOdometryController(LinearOpMode op_mode, DcMotor front_left, DcMotor front_right, DcMotor back_left, DcMotor back_right, IMU imu) {
        Opmode = op_mode;
        FrontLeft = (DcMotorEx) front_left;
        FrontRight = (DcMotorEx) front_right;
        BackLeft = (DcMotorEx) back_left;
        BackRight = (DcMotorEx) back_right;
        Imu = imu;

        LastFL = FrontLeft.getCurrentPosition();
        LastFR = FrontRight.getCurrentPosition();
        LastBL = BackLeft.getCurrentPosition();
        LastBR = BackRight.getCurrentPosition();
        LastHeading = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        RobotPos = new FieldCoord(0.0, 0.0, 0.0);
    }

    public FieldCoord getPosition() {
        return RobotPos;
    }

    /**
     * Updates the robot position to current wheel positions
     */
    public void update() {
        double curFL = FrontLeft.getCurrentPosition();
        double curFR = FrontRight.getCurrentPosition();
        double curBL = BackLeft.getCurrentPosition();
        double curBR = BackRight.getCurrentPosition();
        double curHeading = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double dFL = curFL - LastFL;
        double dFR = curFR - LastFR;
        double dBL = curBL - LastBL;
        double dBR = curBR - LastBR;

        // change relative to robot
        double dRobotY = (dFL + dFR + dBL + dBR) / 4.0;
        double dRobotX = (dFL - dFR - dBL + dBR) / 4.0;
        double dHeading = curHeading - LastHeading;

        // change on field (accounts for moving straight at angles)
        double dY = dRobotY * Math.sin(Math.toRadians(curHeading));
        double dX = dRobotX * Math.cos(Math.toRadians(curHeading));

        RobotPos.X += dX / Environment.Auto.COUNTS_PER_INCH;
        RobotPos.Y += dY / Environment.Auto.COUNTS_PER_INCH;
        RobotPos.Heading += dHeading;

        LastFL = curFL;
        LastFR = curFR;
        LastBL = curBL;
        LastBR = curBR;
        LastHeading = curHeading;
    }

    /**
     * Drives the robot to the specified field coordinates
     * @param pos position for the robot to move to
     */
    public void DriveTo(FieldCoord pos) {
        double relativeX = pos.X - RobotPos.X;
        double relativeY = pos.Y - RobotPos.Y;

        // Rotate coordinates based on current heading
        double rotatedX = relativeX * Math.cos(Math.toRadians(RobotPos.Heading)) - relativeY * Math.sin(Math.toRadians(RobotPos.Heading));
        double rotatedY = relativeX * Math.sin(Math.toRadians(RobotPos.Heading)) + relativeY * Math.cos(Math.toRadians(RobotPos.Heading));

        // Calculate target position in encoder ticks for each wheel
        double ticksFrontLeft = (rotatedX + rotatedY) * Environment.Auto.COUNTS_PER_INCH;
        double ticksFrontRight = (rotatedX - rotatedY) * Environment.Auto.COUNTS_PER_INCH;
        double ticksRearLeft = (rotatedX - rotatedY) * Environment.Auto.COUNTS_PER_INCH;
        double ticksRearRight = (rotatedX + rotatedY) * Environment.Auto.COUNTS_PER_INCH;

        // Set target positions for each motor
        FrontLeft.setTargetPosition((int) ticksFrontLeft);
        FrontRight.setTargetPosition((int) ticksFrontRight);
        BackLeft.setTargetPosition((int) ticksRearLeft);
        BackRight.setTargetPosition((int) ticksRearRight);

        // Set power to motors to start moving
        FrontLeft.setPower(1.0);
        FrontRight.setPower(1.0);
        BackLeft.setPower(1.0);
        BackRight.setPower(1.0);

        Opmode.telemetry.update();

        // Wait until all motors reach their target positions
        while (Opmode.opModeIsActive() && (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy())) {
            update();
            Opmode.idle();
        }

        // Stop motors after reaching the target positions
        FrontLeft.setPower(0.0);
        FrontRight.setPower(0.0);
        BackLeft.setPower(0.0);
        BackRight.setPower(0.0);

        Opmode.telemetry.update();

        update();
    }

    /**
     * Turns the robot to the specified heading
     * @param heading heading to turn to in degrees
     */
    public void TurnTo(double heading) {
        YawPitchRollAngles orientation = Imu.getRobotYawPitchRollAngles();
        double cur = orientation.getYaw(AngleUnit.DEGREES);

        double error = Math.toDegrees(angleWrap(Math.toRadians(heading - cur)));

        FrontLeft.setPower(-Environment.Auto.TURN_POWER);
        FrontRight.setPower(Environment.Auto.TURN_POWER);
        BackLeft.setPower(-Environment.Auto.TURN_POWER);
        BackRight.setPower(Environment.Auto.TURN_POWER);

        Opmode.telemetry.update();

        while (Opmode.opModeIsActive() && Math.abs(error) > Environment.Auto.TURN_TOLERANCE) {
            orientation = Imu.getRobotYawPitchRollAngles();
            cur = orientation.getYaw(AngleUnit.DEGREES);
            error = Math.toDegrees(angleWrap(Math.toRadians(heading - cur)));

            update();

            Opmode.idle();
        }

        FrontLeft.setPower(0.0);
        FrontRight.setPower(0.0);
        BackLeft.setPower(0.0);
        BackRight.setPower(0.0);

        Opmode.telemetry.update();

        update();
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
}
