package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.Environment;

@Autonomous(preselectTeleOp="Junior's New Groove")
public class AutonomousBlueRightCorner extends SplitAutonomous {
    public void on_init() {
        super.init_vars(Environment.Auto.MovementCounts.MOVE_UP,
                        Environment.Auto.MovementCounts.LEFT_ADJUST,
                        Environment.Auto.MovementCounts.RIGHT_ADJUST,
                        Environment.Auto.MovementCounts.FRONT_ADJUST_1,
                        Environment.Auto.MovementCounts.FRONT_ADJUST_2,
                        Environment.Auto.MovementCounts.MOVE_BACK_STRAIGHT,
                        Environment.Auto.MovementCounts.MOVE_BACK_LEFT,
                        Environment.Auto.MovementCounts.MOVE_BACK_RIGHT,
                        -Environment.Auto.MovementCounts.PARK_CORNER_FAR_STRAIGHT,
                        -Environment.Auto.MovementCounts.PARK_CORNER_FAR_STRAFE);
    }
}