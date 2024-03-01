package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.StateSubsystems.Claw;

@Config
public abstract class SplitAutonomous extends LinearOpMode {
    private DriveTrain drive;
    private Trapdoor trap;
    private Claw claw;
    private TeamPropSensors sensors;

    public static boolean PIDSmoothing = false;
    public static long FarWaitTime = 0;

    private boolean FarSide;
    private boolean Red;
    private double MoveUp;
    private double LeftAdjust;
    private double RightAdjust;
    private double FrontAdjust1;
    private double FrontAdjust2;
    private double MoveBackStraight;
    private double MoveBackLeft;
    private double MoveBackRight;
    private double ParkMove1Straight;
    private double ParkMove1Strafe;
    private double ParkMove2Straight = 0.0;
    private double ParkMove2Strafe = 0.0;
    private double ParkMove3Straight = 0.0;
    private double ParkMove3Strafe = 0.0;

    abstract public void on_init();

    @Override
    public void runOpMode() {
        on_init();

        initialize();

        waitForStart();

        // go up to team prop
        drive.DriveForward(MoveUp, PIDSmoothing);
        sleep(500);

        // check color sensors for where team prop is
        String side = sensors.CheckSensors();
        telemetry.addData("Side: ", side);
        telemetry.update();
        sleep(1000);
        
        // turn to pixel and adjust
        if (side.equals("left")) {
            drive.TurnTo(90);
            sleep(500);

            drive.DriveForward(LeftAdjust, PIDSmoothing);
        } else if (side.equals("right")) {
            drive.TurnTo(-90);
            sleep(500);

            drive.DriveForward(RightAdjust, PIDSmoothing);
        } else {
            drive.DriveLeft(FrontAdjust1, PIDSmoothing);
            sleep(500);

            drive.DriveBackward(FrontAdjust2, PIDSmoothing);
        }
        sleep(500);

        // drop pixel and push onto line
        trap.Open();
        drive.DriveForward(25, PIDSmoothing);
        drive.DriveBackward(25, PIDSmoothing);
        sleep(1000);

        // close trap turn back straight
        trap.Close();
        drive.TurnTo(0);
        sleep(500);

        // go back
        drive.DriveBackward(MoveBackStraight, PIDSmoothing);
        sleep(500);

        // turn to face board
        drive.TurnTo((Red) ? -90 : 90);

        // wait for alliance partner if far side
        if (!FarSide) {
            sleep(500);
        } else {
            sleep(FarWaitTime);
        }

        // move up to board
        drive.DriveForward(-ParkMove1Straight, PIDSmoothing);
        sleep(500);

        // line up with board
        if (Red) {
            drive.DriveLeft(ParkMove2Strafe / 2, PIDSmoothing);
        } else {
            drive.DriveRight(ParkMove2Strafe / 2, PIDSmoothing);
        }
        sleep(500);

        // make sure still turned correctly
        drive.TurnTo((Red) ? -90 : 90); 
        sleep(500);

        // move really slow to not ram board
        drive.DriveForward(150, 0.05);
        sleep(500);

        // put arm on board
        claw.MoveArmToGrab();
        sleep(500);

        // drop pixel
        claw.DropClaw();
        sleep(2000);

        // retract arm
        claw.PullArmIn();
        sleep(2000);
    }

    public void init_vars(boolean far_side, double move_up, double left_adjust, double right_adjust, double front_adjust_1, double front_adjust_2, double move_back_straight, double move_back_left, double move_back_right, double park_move_1_straight, double park_move_1_strafe) {
        FarSide = far_side;
        MoveUp = move_up;
        LeftAdjust = left_adjust;
        RightAdjust = right_adjust;
        FrontAdjust1 = front_adjust_1;
        FrontAdjust2 = front_adjust_2;
        MoveBackStraight = move_back_straight;
        MoveBackLeft = move_back_left;
        MoveBackRight = move_back_right;
        ParkMove1Straight = park_move_1_straight;
        ParkMove1Strafe = park_move_1_strafe;

        Red = (ParkMove1Straight > 0);
    }

    public void init_vars(boolean far_side, double move_up, double left_adjust, double right_adjust, double front_adjust_1, double front_adjust_2, double move_back_straight, double move_back_left, double move_back_right, double middle_move_1_straight, double middle_move_1_strafe, double middle_move_2_straight, double middle_move_2_strafe, double middle_move_3_straight, double middle_move_3_strafe) {
        FarSide = far_side;
        MoveUp = move_up;
        LeftAdjust = left_adjust;
        RightAdjust = right_adjust;
        FrontAdjust1 = front_adjust_1;
        FrontAdjust2 = front_adjust_2;
        MoveBackStraight = move_back_straight;
        MoveBackLeft = move_back_left;
        MoveBackRight = move_back_right;
        ParkMove1Straight = middle_move_1_straight;
        ParkMove1Strafe = middle_move_1_strafe;
        ParkMove2Straight = middle_move_2_straight;
        ParkMove2Strafe = middle_move_2_strafe;
        ParkMove3Straight = middle_move_3_straight;
        ParkMove3Strafe = middle_move_3_strafe;

        Red = (ParkMove1Straight > 0);
    }

    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new DriveTrain(this, hardwareMap, telemetry);
        trap = new Trapdoor(hardwareMap);
        claw = new Claw(hardwareMap);
        sensors = new TeamPropSensors(hardwareMap);

        claw.GripClaw();
    }
}