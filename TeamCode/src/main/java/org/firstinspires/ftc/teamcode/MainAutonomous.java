package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 *    Concept for autonomous structure
 */
public abstract class MainAutonomous extends OpMode {
    private final double MOVE_UP;
    private final double MOVE_BACK;
    private final double PARK_MOVE;

    public MainAutonomous(double move_up, double move_back, double park_move) {
        MOVE_UP = move_up;
        MOVE_BACK = move_back;
        PARK_MOVE = park_move;
    }

    public void init() {
        
    }

    public void start() {

    }

    public void loop() {
        
    }
}