package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *    Concept for autonomous structure
 */
public abstract class MainAutonomous extends LinearOpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    private Servo ClawWrist;
    private Servo ClawGrabber;

    private double MOVE_UP;
    private double MOVE_BACK;
    private double PARK_MOVE;

    abstract public void on_init();

    public void runOpMode() throws InterruptedException {
        on_init();

        init();
    }

    public void init_vars(double move_up, double move_back, double park_move) {
        MOVE_UP = move_up;
        MOVE_BACK = move_back;
        PARK_MOVE = park_move;
    }

    public void init() {

    }
}