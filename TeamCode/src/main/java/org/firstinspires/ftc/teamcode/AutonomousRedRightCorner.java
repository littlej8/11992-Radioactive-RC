package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Environment;

/**
 *    Concept for autonomous structure
 */
@Autonomous
public class AutonomousRedRightCorner extends MainAutonomous {
    public void on_init() {
        super.init_vars(Environment.Auto.MovementCounts.MOVE_UP,
                        Environment.Auto.MovementCounts.MOVE_BACK,
                        Environment.Auto.MovementCounts.PARK_CORNER_CLOSE,
                        0.0,
                        0.0
                        );
    }
}