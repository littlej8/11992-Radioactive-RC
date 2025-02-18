package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.StateSubsystems.Claw;

import java.util.Locale;

@TeleOp(name="Junior's New Groove")
public class SplitTeleOp extends LinearOpMode {
    private DriveTrain drive;
    private Lift lift;
    private DroneLauncher drone;
    private Claw claw;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            // player 1
            drive.DriveTeleOp(gamepad1);
            drone.UpdateTeleOp(gamepad1);

            // player 2
            lift.UpdateTeleOp(gamepad2);
            claw.UpdateTeleOp(gamepad2);

            //telemetry
            UpdateTelemetry();
        }
    }

    public void UpdateTelemetry() {
        telemetry.addData("Driver Controller: ", String.format(Locale.getDefault(), "(%f,%f) (%f,%f)", -gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.right_stick_y));
        telemetry.addData("Operator Controller: ", String.format(Locale.getDefault(), "(%f,%f) (%f,%f)", -gamepad2.left_stick_x, -gamepad2.left_stick_y, -gamepad2.right_stick_x, -gamepad2.right_stick_y));
        telemetry.update();
    }

    public void initialize() {
        drive = new DriveTrain(this, hardwareMap, telemetry);
        drone = new DroneLauncher(hardwareMap);
        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);
    }
}