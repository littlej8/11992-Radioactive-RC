package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 *    Concept for autonomous structure
 */
public abstract class MainAutonomous extends OpMode {
    private double MOVE_UP;
    private double MOVE_BACK;
    private double PARK_MOVE;

    public void set_vars(double move_up, double move_back, double park_move) {
        MOVE_UP = move_up;
        MOVE_BACK = move_back;
        PARK_MOVE = park_move;
    }

    abstract public void on_init();

    public void init() {
        
    }

    public void start() {

    }

    public void loop() {
        
    }
}