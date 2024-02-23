package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.*;

public abstract class SplitAutonomous extends LinearOpMode {
    private DriveTrain drive;
    private Trapdoor trap;
    private Claw claw;
    private TeamPropSensors sensors;

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

        drive.DriveForward(MoveUp);
        sleep(500);

        drive.TurnTo(0);
        sleep(500);

        TeamPropSensors.TeamPropLocation side = sensors.CheckSensors();
        
        if (side == TeamPropSensors.TeamPropLocation.LEFT) {
            drive.TurnTo(90);
            sleep(500);

            drive.DriveForward(LeftAdjust);
        } else if (side == TeamPropSensors.TeamPropLocation.RIGHT) {
            drive.TurnTo(-90);
            sleep(500);

            drive.DriveForward(RightAdjust);
        } else {
            drive.DriveLeft(FrontAdjust1);
            sleep(500);

            drive.DriveBackward(FrontAdjust2);
        }
        sleep(500);

        trap.Open();
        sleep(500);

        trap.Close();
        sleep(500);

        if (side == TeamPropSensors.TeamPropLocation.LEFT) {
            drive.DriveLeft(MoveBackLeft);
        } else if (side == TeamPropSensors.TeamPropLocation.RIGHT) {
            drive.DriveRight(MoveBackRight);
        } else {
            drive.DriveBackward(MoveBackStraight);
        }
        sleep(500);

        drive.TurnTo(0);

        if (ParkMove2Straight == 0) {
            sleep(500);
        } else {
            sleep(10000);
        }

        if (side == TeamPropSensors.TeamPropLocation.LEFT) {
            drive.DriveBackward(ParkMove1Straight);
        } else if (side == TeamPropSensors.TeamPropLocation.RIGHT) {
            drive.DriveForward(ParkMove1Straight);
        } else {
            drive.DriveRight(ParkMove1Strafe);
        }
        sleep(500);

        if (ParkMove2Straight == 0) {
            return;
        }

        if (side == TeamPropSensors.TeamPropLocation.LEFT) {
            drive.DriveRight(ParkMove2Strafe);
        } else if (side == TeamPropSensors.TeamPropLocation.RIGHT) {
            drive.DriveLeft(ParkMove2Strafe);
        } else {
            drive.DriveForward(ParkMove2Straight);
        }
        sleep(500);

        if (side == TeamPropSensors.TeamPropLocation.LEFT) {
            drive.DriveBackward(ParkMove3Straight);
        } else if (side == TeamPropSensors.TeamPropLocation.RIGHT) {
            drive.DriveForward(ParkMove3Straight);
        } else {
            drive.DriveRight(ParkMove3Strafe);
        }
    }

    public void init_vars(double move_up, double left_adjust, double right_adjust, double front_adjust_1, double front_adjust_2, double move_back_straight, double move_back_left, double move_back_right, double park_move_1_straight, double park_move_1_strafe) {
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
    }

    public void init_vars(double move_up, double left_adjust, double right_adjust, double front_adjust_1, double front_adjust_2, double move_back_straight, double move_back_left, double move_back_right, double middle_move_1_straight, double middle_move_1_strafe, double middle_move_2_straight, double middle_move_2_strafe, double middle_move_3_straight, double middle_move_3_strafe) {
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
    }

    public void initialize() {
        drive = new DriveTrain(this, hardwareMap);
        trap = new Trapdoor(hardwareMap);
        claw = new Claw(hardwareMap);
        sensors = new TeamPropSensors(hardwareMap);
    }
}