package org.firstinspires.ftc.team15091;

import android.content.SharedPreferences;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.aztec.AztecMecanumDrive;
import org.firstinspires.aztec.DistanceDetector;
import org.firstinspires.aztec.RingDetector;
import org.firstinspires.aztec.RingLayout;

import static android.content.Context.MODE_PRIVATE;

@Autonomous(name = "Red: Auto", group = "Autonomous", preselectTeleOp = "Gamepad")
public class Red_Autonomous extends AztecOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        drivetrain = new AztecMecanumDrive(hardwareMap, true);
        robot = new Robot(hardwareMap);
        drivetrain.setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RingDetector ringDetector = new RingDetector(this, 0.65f);
        DistanceDetector wobbleDetector = new DistanceDetector(robot.wobbleRange, 5);

        SharedPreferences sharedPreferences = hardwareMap.appContext.getSharedPreferences("Aztec", MODE_PRIVATE);

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.addData("Heading: ", "%.4f", drivetrain.getHeading());
            telemetry.addData("Ring Detect", ringDetector.objectDetected().toString());
            telemetry.update();
            idle();
        }

        if (opModeIsActive()) {
            RingLayout ringNum = ringDetector.objectDetected();
            ringDetector.dispose();

            robot.trigger.setPosition(0.5d);

            //set arm ready to pickup wobble goal
            robot.wrist.setPosition(1d);
            robot.claw.setPosition(0.65d);
            sleep(1800l);

            //pickup wobble goal
            robot.claw.setPosition(0d);
            sleep(500l);

            //lift wobble goal
            robot.winch.setTargetPosition(400);
            robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.winch.setPower(1d);
            sleep(500l);

            //align first power shot
            gyroSlide(0.34f, 14.8d, 0, 4, null);

            //spin up shooter
            robot.shooter.setVelocity(1820d);

            gyroDrive(0.5f, 66d, 0, 6, null);
            //gyroTurn(0.5f, 0f, 2);

            //wait to shooter to spin up
            while (robot.shooter.getVelocity() < 1820d) {
                idle();
            }

            //stabilize
            sleep(750l);

            //shoot first ring
            robot.trigger.setPosition(0d);
            sleep(500l);
            robot.trigger.setPosition(0.5d);

            //align second power shot
            gyroSlide(0.4f, 8.91d, 0, 3, null);
            gyroTurn(0.5f, 0f, 1);

            //spin up shooter
            robot.shooter.setVelocity(1810d);

            //wait to shooter to spin up
            while (robot.shooter.getVelocity() < 1810d) {
                idle();
            }

            //stabilize
            sleep(750l);

            //shoot second ring
            robot.trigger.setPosition(0d);
            sleep(500l);
            robot.trigger.setPosition(0.5d);

            //align third power shot
            gyroSlide(0.4f, 8.6f, 0, 3, null);
            gyroTurn(0.5f, 0f, 1);

            //spin up shooter
            robot.shooter.setVelocity(1810d);

            //wait to shooter to spin up
            while (robot.shooter.getVelocity() < 1810d) {
                idle();
            }

            //stabilize
            sleep(750l);

            //shoot third ring
            robot.trigger.setPosition(0d);
            sleep(500l);
            robot.trigger.setPosition(0.5d);
            sleep(200);

            robot.shooter.setVelocity(0d);

            if (ringNum == RingLayout.Quad) {
                gyroDrive(0.9f, 40, 0, 3, null);
                gyroTurn(0.9f, -92, 2);

                //lower winch
                robot.winch.setTargetPosition(0);
                robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.winch.setPower(0.2d);

                gyroDrive(0.9f, 52.2d, -90, 4, null);

                //release wobble goal
                robot.claw.setPosition(0.65d);
                sleep(400);
                robot.wrist.setPosition(0.32d);
                robot.claw.setPosition(0d);

                //move towards more ring
                gyroDrive(0.9f, -21.8d, -90, 2, null);
                gyroTurn(0.9f, 0, 1);

                //shoot high goal
                robot.shooter.setVelocity(1810d);

                gyroDrive(0.9f, -60.2d, 0, 5, null);

                //pickup 3 rings
                robot.roller.setPower(1d);
                sleep(300);

                gyroDrive(0.1f, -15d, 0, 4, null);
                sleep(100);

                //drive behind white line
                gyroDrive(0.85f, 28.7d, -2d, 3, null);
                sleep(100);
                gyroTurn(0.6f, -1.9d, 2);

                //shoot 1st ring
                robot.trigger.setPosition(0d);
                sleep(300);
                robot.trigger.setPosition(0.5d);
                sleep(500);

                robot.roller.setPower(0d);

                //shoot 2nd ring
                robot.trigger.setPosition(0d);
                sleep(300);
                robot.trigger.setPosition(0.5d);
                sleep(500);

                //shoot 3rd ring
                robot.trigger.setPosition(0d);
                sleep(500);

                //park
                gyroDrive(1f, 10d, 0, 6, null);
            } else if (ringNum == RingLayout.Single) {
                gyroDrive(0.8f, 20, 0, 5, null);
                gyroTurn(0.8f, -90, 5);

                //lower winch
                robot.winch.setTargetPosition(0);
                robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.winch.setPower(0.5d);

                gyroDrive(0.7f, 30, -90, 5, null);

                //release wobble goal
                robot.claw.setPosition(0.65d);
                sleep(500);
                robot.wrist.setPosition(0.32d);
                robot.claw.setPosition(0d);

                gyroSlide(0.3f, -6, -90, 5, null);
                gyroTurn(0.8f, 0, 5);

                robot.roller.setPower(1d);
                gyroDrive(0.7f, -45d, 0, 6, null);
                gyroDrive(0.1f, -4d, 0, 1, null);

                robot.shooter.setVelocity(1810d);

                gyroDrive(0.8f, 27.5d, 0, 6, null);
                gyroTurn(0.7f, -3d, 1);

                //shoot 1 ring
                robot.trigger.setPosition(0d);
                sleep(500);

                //park
                gyroDrive(0.8f, 8d, 0, 6, null);
            } else {
                gyroTurn(0.8f, -90, 5);

                //lower winch
                robot.winch.setTargetPosition(0);
                robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.winch.setPower(0.5d);

                gyroDrive(0.7f, 52, -90, 5, null);

                //release wobble goal
                robot.claw.setPosition(0.65d);
                sleep(600);
                robot.wrist.setPosition(0.75d);

                //move to 2nd wobble goal
                gyroDrive(0.7f, -26.5, -90, 3, null);
                gyroTurn(0.8f, -180, 3);
                gyroDrive(0.7f, 53.5, -180, 5, null);

                //pick up 2nd wobble goal
                robot.wrist.setPosition(1d);
                robot.claw.setPosition(0.65d);
                sleep(500);
                gyroSlide(0.1d, 4d, -180d, 1, wobbleDetector);

                robot.claw.setPosition(0d);
                sleep(500);
                //lift wobble goal
                robot.winch.setTargetPosition(400);
                robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.winch.setPower(1d);

                //park
                gyroDrive(0.9f, -54, -180, 5, null);
                robot.winch.setTargetPosition(0);
                robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.winch.setPower(1d);
                gyroSlide(0.5d, 10d, -180d, 2, null);

                //release wobble goal
                robot.claw.setPosition(0.65d);
                sleep(600);
                robot.wrist.setPosition(0.32d);
                robot.claw.setPosition(0d);
                gyroSlide(0.6d, -10d, -180d, 2, null);
                gyroDrive(0.6f, -2, -180, 1, null);
                gyroTurn(0.9d, 0, 2);
            }

            SharedPreferences.Editor editor = sharedPreferences.edit();
            editor.putLong("Heading", (long) drivetrain.getHeading());
            editor.commit();
        }
    }
}
