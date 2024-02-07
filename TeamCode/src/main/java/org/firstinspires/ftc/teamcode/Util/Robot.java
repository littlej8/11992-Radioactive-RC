package org.firstinspires.ftc.teamcode.Util;

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

    public enum Component {
        Motor,
        Servo,
    }

    private HardwareMap hwMap;

    private Map<String, DcMotor> motors;
    private Map<String, Servo> servos;

    RevHubOrientationOnRobot.LogoFacingDirection LogoDirection;
    RevHubOrientationOnRobot.UsbFacingDirection UsbDirection;
    
    private IMU imu;

    /**
     * Uses default names for motors (Frontleft, Backright, etc.)
     * Uses default orientation for IMU (Forward & Up)
     */
    public Robot(DriveTrain drive) {
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

    public Robot(DriveTrain drive, RevHubOrientationOnRobot.LogoFacingDirection logo_dir, RevHubOrientationOnRobot.UsbFacingDirection usb_dir) {
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

        LogoDirection = logo_dir;
        UsbDirection = usb_dir;
    }

    public void SetHubOrientation(RevHubOrientationOnRobot.LogoFacingDirection logo_dir, RevHubOrientationOnRobot.UsbFacingDirection usb_dir) {
        LogoDirection = logo_dir;
        UsbDirection = usb_dir;
    }

    /**
     * Add component to the robot for initialization
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

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
}