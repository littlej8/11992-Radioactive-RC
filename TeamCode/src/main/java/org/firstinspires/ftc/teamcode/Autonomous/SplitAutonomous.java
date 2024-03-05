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
    public static long PropPause = 1000;
    public static long ArmPause = 1000;
    public static long MovePause = 250;

    public static int Drop1 = 305;
    public static int Drop2 = 360;
    public static int Drop3 = 425;

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
        sleep(MovePause);

        // check color sensors for where team prop is
        String side = sensors.CheckSensors();
        telemetry.addData("Side: ", side);
        telemetry.update();
        sleep(PropPause);
        
        // turn to pixel and adjust
        if (side.equals("left")) {
            drive.TurnTo(90);
            sleep(MovePause);

            drive.DriveForward(LeftAdjust, PIDSmoothing);
        } else if (side.equals("right")) {
            drive.TurnTo(-90);
            sleep(MovePause);

            drive.DriveForward(RightAdjust, PIDSmoothing);
        } else {
            drive.DriveLeft(FrontAdjust1, PIDSmoothing);
            sleep(MovePause);

            drive.DriveBackward(FrontAdjust2, PIDSmoothing);
        }
        sleep(MovePause);

        // drop pixel and push onto line
        trap.Open();
        drive.DriveForward(35, PIDSmoothing);
        sleep(100);
        drive.DriveBackward(35, PIDSmoothing);
        sleep(MovePause);

        // close trap turn back straight
        trap.Close();
        drive.TurnTo(0);
        sleep(MovePause);

        // go back
        drive.DriveBackward(MoveBackStraight, PIDSmoothing);
        sleep(MovePause);

        // turn to face board
        drive.TurnTo((Red) ? -90 : 90);

        // wait for alliance partner if far side
        if (!FarSide) {
            sleep(MovePause);
        } else {
            sleep(FarWaitTime);
        }

        // move up to board
        drive.DriveForward((Red) ? ParkMove1Straight : -ParkMove1Straight, PIDSmoothing);
        sleep(MovePause);

        int strafe_amount = Drop1;

        if (side.equals("left")) {
            strafe_amount = (Red) ? Drop3 : Drop1;
        } else if (side.equals("right")) {
            strafe_amount = (Red) ? Drop1 : Drop3;
        } else {
            strafe_amount = Drop2;
        }

        // line up with board
        if (Red) {
            drive.DriveLeft(strafe_amount, PIDSmoothing);
        } else {
            drive.DriveRight(strafe_amount, PIDSmoothing);
        }
        sleep(MovePause);

        // make sure still turned correctly
        drive.TurnTo((Red) ? -90 : 90); 
        sleep(MovePause);

        // move really slow to not ram board
        drive.DriveForward(200, 0.05);
        sleep(MovePause);

        // put arm on board
        claw.MoveArmToGrab();
        sleep(ArmPause);

        // drop pixel
        claw.DropClaw();
        sleep(ArmPause);
        claw.StopClaw();

        // retract arm
        claw.PullArmIn();
        sleep(ArmPause * 4);
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