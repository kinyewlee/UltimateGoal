package org.firstinspires.ftc.team15091;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.aztec.DistanceDetector;
import org.firstinspires.ftc.team15091.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
@Disabled
public class GamepadTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(hardwareMap);
        DistanceDetector wobbleDetector = new DistanceDetector(robot.wobbleRange, 25);

        boolean rightBumper = false, buttonA = false, buttonX = false, buttonY = false;
        boolean resetWinch = false;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            // POV Mode uses left stick y to go forward, and left stick x to turn.
            // right stick x to move side way
            // - This uses basic math to combine motions and is easier to drive straight.
            double stickY = -gamepad1.left_stick_y;
            if (!gamepad1.right_bumper) {
                stickY -= gamepad1.right_stick_y;
            }

            double forward = Range.scale(stickY, -1d, 1d, -1d, 1d);
            double turn = gamepad1.left_stick_x;
            double side = gamepad1.right_stick_x;

            if (robot.wrist.getPosition() == 1d && robot.claw.getPosition() != 0d) {
                if (wobbleDetector.getCurrentDistance() < 2.5d) {
                    if (side < 0d) {
                        side = 0d;
                    }
                } else if (wobbleDetector.getCurrentDistance() < 10d) {
                    if (side < -0.1d) {
                        side = -0.1d;
                    }
                }
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            forward,
                            -side,
                            -turn
                    )
            );

            drive.update();

            if (gamepad1.left_trigger > 0.7d) {
                robot.shooter.setVelocity(2080d);
                robot.roller.setPower(0d);
                if (robot.shooter.getVelocity() >= 2080d) {
                    robot.beep(1);
                }
            } else if (gamepad1.left_trigger > 0.2d) {
                robot.shooter.setVelocity(1800d);
                robot.roller.setPower(0d);
            } else {
                robot.shooter.setVelocity(0d);
            }

            if (gamepad1.right_bumper) {
                if (!rightBumper) {
                    if (robot.roller.getPower() == 0d) {
                        if (gamepad1.left_bumper) {
                            robot.roller.setPower(-1d);
                        } else {
                            robot.roller.setPower(1d);
                        }
                    } else {
                        robot.roller.setPower(0d);
                    }
                }
                rightBumper = true;
            } else {
                rightBumper = false;
            }

            if (gamepad1.a) {
                if (!buttonA) {
                    robot.trigger.setPosition(0d);
                }
                buttonA = true;
            } else {
                robot.trigger.setPosition(0.47d);
                buttonA = false;
            }

            if (gamepad1.x) {
                if (!buttonX) {
                    if (robot.wrist.getPosition() == 1d) {
                        robot.wrist.setPosition(0.32d);
                    } else {
                        robot.claw.setPosition(0.65d);
                        robot.wrist.setPosition(1d);
                    }
                }
                buttonX = true;
            } else {
                if (buttonX) {
                    buttonX = false;
                }
            }

            if (gamepad1.y) {
                if (!buttonY && robot.wrist.getPosition() == 1d) {
                    if (robot.claw.getPosition() == 0d) {
                        robot.claw.setPosition(0.65d);
                    } else {
                        robot.claw.setPosition(0d);
                    }
                }
                buttonY = true;
            } else {
                buttonY = false;
            }

            if (gamepad1.dpad_up) {
                if (gamepad1.left_bumper) {
                    robot.winch.setPower(0.4d);
                } else {
                    robot.winch.setTargetPosition(2600);
                    robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.winch.setPower(1d);
                }
            } else if (gamepad1.dpad_down) {
                if (gamepad1.left_bumper) {
                    robot.winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.winch.setPower(-0.4d);
                    resetWinch = true;
                } else {
                    robot.winch.setTargetPosition(0);
                    robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.winch.setPower(-1d);
                }
            } else {
                robot.winch.setPower(0d);
                if (resetWinch) {
                    robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetWinch = false;
                }
            }

            telemetry.addData("Winch", "%d",
                    robot.winch.getCurrentPosition());
            telemetry.addData("Shooter", "%.2f",
                    robot.shooter.getVelocity());
            telemetry.addData("Roller", "Power:%.2f, Velocity:%.2f",
                    robot.roller.getPower(), robot.roller.getVelocity());
            telemetry.addData("Wobble", "Arm:%.2f Dist:%.1f Claw: %.2f",
                    robot.wrist.getPosition(), wobbleDetector.getCurrentDistance(), robot.claw.getPosition());

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
