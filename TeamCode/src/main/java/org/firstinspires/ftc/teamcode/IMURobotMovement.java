package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.env;

@Autonomous(name="IMU Robot Movement", group="Robot")
public class IMURobotMovement extends LinearOpMode {
    private DcMotor         frontLeftDrive   = null;
    private DcMotor         frontRightDrive  = null;
    private DcMotor         backLeftDrive = null;
    private DcMotor         backRightDrive = null
    private IMU             imu         = null;      // Control/Expansion Hub IMU

    private double          headingError  = 0;

    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;

    private double  frontLeftSpeed     = 0;
    private double  frontRightSpeed    = 0;
    private double  backLeftSpeed     = 0;
    private double  backRightSpeed    = 0;

    private int     frontLeftTarget    = 0;
    private int     frontRightTarget   = 0;
    private int     backLeftTarget    = 0;
    private int     backRightTarget   = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 145.1 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRight");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        driveStraight(DRIVE_SPEED, env.MOVE_UP, 0.0);
        turnToHeading(TURN_SPEED, 90.0);

        sleep(1000);
        turnToHeading(TURN_SPEED, 0.0)

        driveStraight(DRIVE_SPEED, env.MOVE_BACK, 0.0);

        sleep(1000);
    }

    public void driveStraight(double maxDriveSpeed, double distance, double heading) {
        if (opModeIsActive()) {
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = frontLeftDrive.getCurrentPosition() + moveCounts;
            frontRightTarget = frontRightDrive.getCurrentPosition() + moveCounts;
            backLeftTarget = backLeftDrive.getCurrentPosition() + moveCounts;
            backRightTarget = backRightDrive.getCurrentPosition() + moveCounts;

            frontLeftDrive.setTargetPosition(frontLeftTarget);
            frontRightDrive.setTargetPosition(frontRightTarget);
            backLeftDrive.setTargetPosition(backLeftTarget);
            backRightDrive.setTargetPosition(backRightTarget);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            while (opModeIsActive() && (
                    frontLeftDrive.isBusy() ||
                    frontRightDrive.isBusy() ||
                    backLeftDrive.isBusy() ||
                    backRightDrive.isBusy())) {
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                
                if (distance < 0)
                    turnSpeed *= -1.0;
                
                moveRobot(driveSpeed, turnSpeed);

                sendTelemetry(true);
            }

            moveRobot(0, 0);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0, turnSpeed);

            sendTelemetry(false;)
        }

        moveRobot(0, 0);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0, turnSpeed);

            sendTelemetry(false);
        }

        moveRobot(0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;

        headingError = targetHeading - getHeading();

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed = turn;

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        frontLeftDrive.setPower(leftSpeed);
        frontRightDrive.setPower(rightSpeed);
        backLeftDrive.setPower(leftSpeed);
        backRightDrive.setPower(rightSpeed);
    }

    private void sendTelemetry(boolean straight) {
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos FL:FR BL:BR", "%7d:%7d %7d:%7d",
                                frontLeftTarget,
                                frontRightTarget,
                                backLeftTarget,
                                backRightTarget);
            telemetry.addData("Actual Pos FL:FR BL:BR", "%7d:%7d %7d:%7d",
                                frontLeftDrive.getCurrentPosition(),
                                frontRightDrive.getCurrentPosition(),
                                backLeftDrive.getCurrentPosition(),
                                backRightDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.2f", targetHeading, getHeading());
        telemetry.addData("Error : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds FL:FR BL:BR", "%5.2f:%5.2f 5.2f:%5.2f",
                            frontLeftSpeed,
                            frontRightSpeed,
                            backLeftSpeed,
                            backRightSpeed);
        telemetry.update();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}