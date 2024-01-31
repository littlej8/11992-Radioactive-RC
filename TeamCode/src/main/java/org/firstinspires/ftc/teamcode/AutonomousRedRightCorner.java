package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MainAutonomous;
import org.firstinspires.ftc.teamcode.Environment;

/**
 *    Concept for autonomous structure
 */
@Autonomous()
public class AutonomousRedRightCorner extends MainAutonomous {
    public AutonomousRedRightCorner() {
        super(Environment.MOVE_UP, Environment.MOVE_BACK, Environment.PARK_CORNER_CLOSE);
    }
}