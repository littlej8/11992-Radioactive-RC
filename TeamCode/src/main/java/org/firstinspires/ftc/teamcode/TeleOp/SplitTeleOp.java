package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems;

@TeleOp
@Disabled
public class MainTeleOp extends LinearOpMode {
    private DriveTrain drive;
    private Lift lift;
    private DroneLauncher drone;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            drive.DriveTeleOp(gamepad1);
            lift.UpdateTeleOp(gamepad2);
            drone.UpdateTeleOp(gamepad1);
        }
    }

    public void initialize() {
        drive = new DriveTrain(hardwareMap);
        lift = new Lift(hardwareMap);
        drone = new DroneLauncher(hardwareMap);
    }
}