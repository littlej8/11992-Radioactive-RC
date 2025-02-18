package org.firstinspires.ftc.teamcode.Unused.Unfinished;

import java.util.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Class for controlling robot movement and initialization
 */
public class Robot {
    /**
     * Type of robot drive (Mecanum, Tank, etc.)
     */
    public enum DriveTrain {
        Mecanum,
        Tank,
    }

    /**
     * Motors, Servos, etc that you need to move
     */
    public enum Component {
        Motor,
        Servo,
    }

    private LinearOpMode OpMode;
    private HardwareMap hwMap;

    private Map<String, DcMotor> motors;
    private Map<String, Servo> servos;

    private Map<String, Double> LastPositions;

    RevHubOrientationOnRobot.LogoFacingDirection LogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection UsbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    private IMU imu;

    private double RobotX = 0.0;
    private double RobotY = 0.0;
    private double RobotHeading = 0.0;

    public Robot(LinearOpMode opmode, HardwareMap hardware) {
        SetOpMode(opmode);
        SetHardwareMap(hardware);
        SetDriveTrain(DriveTrain.Mecanum);
    }

    public Robot(LinearOpMode opmode, HardwareMap hardware, double x_pos, double y_pos, double heading) {
        SetOpMode(opmode);
        SetHardwareMap(hardware);
        SetPosition(x_pos, y_pos, heading);
        SetDriveTrain(DriveTrain.Mecanum);
    }

    public Robot(LinearOpMode opmode, HardwareMap hardware, DriveTrain drive) {
        SetOpMode(opmode);
        SetHardwareMap(hardware);
        SetDriveTrain(drive);
    }

    public Robot(LinearOpMode opmode, HardwareMap hardware, DriveTrain drive, RevHubOrientationOnRobot.LogoFacingDirection logo_dir, RevHubOrientationOnRobot.UsbFacingDirection usb_dir) {
        SetOpMode(opmode);
        SetHardwareMap(hardware);
        SetDriveTrain(drive);
        SetHubOrientation(logo_dir, usb_dir);
    }

    public void SetOpMode(LinearOpMode opmode) {
        OpMode = opmode;
    }

    public void SetHardwareMap(HardwareMap hardware) {
        hwMap = hardware;
    }

    public void SetDriveTrain(DriveTrain drive) {
        switch(drive) {
            case Mecanum:
                motors.put("Frontleft", null);
                motors.put("Frontright", null);
                motors.put("Backleft", null);
                motors.put("Backright", null);
                break;
            case Tank:
                System.out.println("Tank Drive is not implemented yet.");
                break;
            default:
                System.out.println("Uh Oh");
                break;
        }
    }

    public void SetHubOrientation(RevHubOrientationOnRobot.LogoFacingDirection logo_dir, RevHubOrientationOnRobot.UsbFacingDirection usb_dir) {
        LogoDirection = logo_dir;
        UsbDirection = usb_dir;
    }

    public void SetPosition(double x_pos, double y_pos, double heading) {
        RobotX = x_pos;
        RobotY = y_pos;
        RobotHeading = heading;
    }

    /**
     * Add component to the robot before initialization
     */
    public void AddComponent(Component component, String name) {
        switch(component) {
            case Motor:
                motors.put(name, null);
                break;
            case Servo:
                motors.put(name, null);
                break;
            default:
                System.out.println("Uh Oh");
                break;
        }
    }

    /**
     * Initializes the robot
     */
    public void Init() {
        for (String name : motors.keySet()) {
            motors.put(name, hwMap.get(DcMotor.class, name));
        }

        for (String name : servos.keySet()) {
            servos.put(name, hwMap.get(Servo.class, name));
        }

        LastPositions.put("Frontleft", (double)GetMotor("Frontleft").getCurrentPosition());
        LastPositions.put("Frontright", (double)GetMotor("Frontright").getCurrentPosition());
        LastPositions.put("Backleft", (double)GetMotor("Backleft").getCurrentPosition());
        LastPositions.put("Backright", (double)GetMotor("Backright").getCurrentPosition());

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LogoDirection, UsbDirection)));
    }

    public DcMotor GetMotor(String name) {
        return motors.get(name);
    }

    public Servo GetServo(String name) {
        return servos.get(name);
    }

    /*
        http://web.mit.edu/6.186/2007/tutorials/odomtutorial/odomtutorial.pdf
    */
    public void UpdateRobotPosition() {
        DcMotor fl = GetMotor("Frontleft");
        DcMotor fr = GetMotor("Frontright");
        DcMotor bl = GetMotor("Backleft");
        DcMotor br = GetMotor("Backright");

        double dFL = fl.getCurrentPosition() - LastPositions.get("Frontleft");
        double dFR = fr.getCurrentPosition() - LastPositions.get("Frontright");
        double dBL = bl.getCurrentPosition() - LastPositions.get("Backleft");
        double dBR = br.getCurrentPosition() - LastPositions.get("Backright");

        LastPositions.put("Frontleft", (double)fl.getCurrentPosition());
        LastPositions.put("Frontright", (double)fr.getCurrentPosition());
        LastPositions.put("Backleft", (double)bl.getCurrentPosition());
        LastPositions.put("Backright", (double)br.getCurrentPosition());

        double heading = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        double dRobotX = (dFL + dFR + dBL + dBR) / 4;
        double dRobotY = (dFL - dFR - dBL + dBR) / 4;

        double dFieldX = dRobotX * Math.cos(heading);
        double dFieldY = dRobotY * Math.sin(heading);

        RobotX += dFieldX;
        RobotY += dFieldY;
        RobotHeading = heading;
    }

    public void DriveTo(double x, double y, double heading) {
        
    }
}