package org.firstinspires.ftc.teamcode.Unused.Unfinished;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Trapdoor;
import org.firstinspires.ftc.teamcode.StateSubsystems.TeamPropSensors;
import org.firstinspires.ftc.teamcode.StateSubsystems.Claw;
import org.firstinspires.ftc.teamcode.StateSubsystems.DriveTrain;

public abstract class StateAutonomous extends LinearOpMode {
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

    public enum AutoState {
        MOVE_FORWARD,
        CHECK_PROP,
        PLACE_PROP,
        GO_BACK,
        PARK1,
        PARK2,
        PARK3
    };

    private int state = 0;
    private boolean state_began = false;
    private final int max_state = 10;

    abstract public void on_init();

    @Override
    public void runOpMode() {
        on_init();

        initialize();

        waitForStart();

        while (opModeIsActive() && state <= max_state) {
            drive.periodic();
            claw.periodic();

            UpdateState();
        }
    }

    public void UpdateState() {
        switch(state) {
            case 0:
                if (!state_began) {
                    drive.ScheduleDriveForward(MoveUp);
                    state_began = true;
                } else if (state_began && !drive.isActive()) {
                    state++;
                    state_began = false;
                }
                break;
            case 1:
                if (!state_began) {
                    sensors.CheckSensors();
                    state_began = true;
                } else if (state_began && !drive.isActive()) {
                    state++;
                    state_began = false;
                }
                break;
            case 2:
                if (!state_began) {
                    if (sensors.left()) {
                        drive.ScheduleTurnTo(90);
                    } else if (sensors.right()) {
                        drive.ScheduleTurnTo(-90);
                    } else {
                        drive.ScheduleDriveLeft(FrontAdjust1);
                    }
                } else if (state_began && !drive.isActive()) {
                    state++;
                    state_began = false;
                }
                break;
            case 3:
                if (!state_began) {
                    if (sensors.left()) {
                        drive.ScheduleDriveForward(LeftAdjust);
                    } else if (sensors.right()) {
                        drive.ScheduleDriveForward(RightAdjust);
                    } else {
                        drive.ScheduleDriveBackward(FrontAdjust2);
                    }
                } else if (state_began && !drive.isActive()) {
                    state++;
                    state_began = false;
                }
                break;
            case 4:
                if (!state_began) {
                    if (sensors.left()) {
                        drive.ScheduleDriveForward(LeftAdjust);
                    } else if (sensors.right()) {
                        drive.ScheduleDriveForward(RightAdjust);
                    } else {
                        drive.ScheduleDriveBackward(FrontAdjust2);
                    }
                } else if (state_began && !drive.isActive()) {
                    state++;
                    state_began = false;
                }
                break;
            case 5:

                break;
            case 6:

                break;
        }
    }

    public boolean MoveForward() {
        
    }

    public boolean PlaceProp() {

    }

    public boolean GoBack() {

    }

    public boolean Park1() {

    }

    public boolean Park2() {
        
    }

    public boolean Park3() {

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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new DriveTrain(this, hardwareMap, telemetry);
        trap = new Trapdoor(hardwareMap);
        claw = new Claw(hardwareMap);
        sensors = new TeamPropSensors(hardwareMap);
    }
}