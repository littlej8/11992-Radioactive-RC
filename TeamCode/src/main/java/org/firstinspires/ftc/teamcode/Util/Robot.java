package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.robot.Robot;

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

    private Map<String, DcMotor> motors;
    private Map<String, Servo> servos;

    private IMU imu;

    /**
     * Uses default names for motors (Frontleft, Backright, etc.)
     */
    public Robot(DriveTrain drive) {
        switch(drive) {
            case DriveTrain.Mecanum:
                motors.set("Frontleft", null);
                motors.set("Frontright", null);
                motors.set("Backleft", null);
                motors.set("Backright", null);
                break;
            case DriveTrain.Tank:
                System.out.println("Tank Drive is not implemented yet.");
                break;
            default:
                System.out.println("Uh Oh");
                break;
        }
    }

    /**
     * Add component to the robot for initialization
     */
    public void AddComponent(Component component, String name) {
        switch(component) {
            case Component.Motor:
                motors.set(name, null);
                break;
            case Component.Servo:
                motors.set(name, null);
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

    }
}