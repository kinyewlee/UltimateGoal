package org.firstinspires.ftc.team15091;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team15091.drive.SampleMecanumDrive;

@Autonomous(name = "Red: Auto (RR)", group = "Autonomous", preselectTeleOp = "Gamepad")
@Disabled
public class RoadRunner_Autonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-63, -34));
        drive.update();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.addData("Heading: ", "%.4f", drive.getHeading());
            telemetry.update();
            idle();
        }


        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 21), 0)
                .lineToLinearHeading(new Pose2d(76,21))
                .build();

        drive.followTrajectory(traj);

        telemetry.addData("Heading: ", "%.4f", drive.getHeading());
        telemetry.update();

        drive.gyroTurn(0.5f, 0f, 2);
    }
}
