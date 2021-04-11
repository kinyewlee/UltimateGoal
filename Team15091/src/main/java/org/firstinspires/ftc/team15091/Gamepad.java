package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.aztec.DistanceDetector;
import org.firstinspires.aztec.UltimateGoalNavigator;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@TeleOp(name = "Gamepad", group = "Linear Opmode")
public class Gamepad extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        UltimateGoalNavigator navigator = new UltimateGoalNavigator(hardwareMap);
        Robot robot = new Robot(hardwareMap, false);
        DistanceDetector wobbleDetector = new DistanceDetector(robot.wobbleRange, 25);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        navigator.targetsUltimateGoal.activate();

        boolean rightBumper = false, buttonA = false, buttonX = false, buttonY = false;
        boolean resetWinch = false, notifyFire = false;
        int adjustMode = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double pLeftFront, pLeftRear, pRightFront, pRightRear;

            // POV Mode uses left stick y to go forward, and left stick x to turn.
            // right stick x to move side way
            // - This uses basic math to combine motions and is easier to drive straight.
            double stickY = -gamepad1.left_stick_y;
            if (!gamepad1.right_bumper) {
                stickY -= gamepad1.right_stick_y;
            }

            double drive = Range.scale(stickY, -1d, 1d, -1d, 1d);
            double turn = gamepad1.left_stick_x;
            double side = gamepad1.right_stick_x;

            navigator.checkRedTowerGoal();
            if (gamepad1.b) {
                if (navigator.targetVisible) {
                    robot.roller.setPower(0d);

                    // align robot with navigation target on red tower goal
                    VectorF translation = navigator.lastLocation.getTranslation();
                    Orientation rotation = Orientation.getOrientation(navigator.lastLocation, EXTRINSIC, XYZ, DEGREES);
                    // calculate the difference between destination and robot coordinates
                    double xGap = Range.clip((translation.get(0) / navigator.mmPerInch) - 10, -10, 10d);
                    double yGap = Range.clip((translation.get(1) / navigator.mmPerInch) + 40, -10d, 10d);
                    double hGap = Range.clip(rotation.thirdAngle - 90d, -5d, 5d);
                    //move along the axes according to the calculated difference
                    if ((adjustMode == 0 || adjustMode == 1) && Math.abs(xGap) > 1d) {
                        drive = Range.scale(-xGap, -10d, 10d, -0.5d, 0.5d);
                        adjustMode = 1;
                    } else
                        if ((adjustMode == 0 || adjustMode == 2) && Math.abs(hGap) > 0.7d) {
                        turn = Range.scale(hGap, -5d, 5d, -0.15d, 0.15d);
                        adjustMode = 2;
                    } else if ((adjustMode == 0 || adjustMode == 3) && Math.abs(yGap) > 1d) {
                        side = Range.scale(yGap, -10d, 10d, -0.4d, 0.4d);
                        adjustMode = 3;
                    } else {
                        if (adjustMode != 0) {
                            adjustMode = 0;
                            robot.beep();
                        }
                    }
                }
            } else {
                adjustMode = 0;
            }

            if (wobbleDetector.getCurrentDistance() < 10d && (side < -0.05d)) {
                side = -0.05d;
            } else if (wobbleDetector.getCurrentDistance() < 40d && (side < -0.2d)) {
                side = -0.2d;
            } else if (wobbleDetector.getCurrentDistance() < 70d && (side < -0.4d)) {
                side = -0.4d;
            }

            pLeftFront = Range.clip(drive + turn + side, -1.0, 1.0);
            pLeftRear = Range.clip(drive + turn - side, -1.0, 1.0);
            pRightFront = Range.clip(drive - turn - side, -1.0, 1.0);
            pRightRear = Range.clip(drive - turn + side, -1.0, 1.0);

            if (gamepad1.left_bumper) {
                pLeftFront *= 0.6d;
                pLeftRear *= 0.6d;
                pRightFront *= 0.6d;
                pRightRear *= 0.6d;
            }


            // Send calculated power to wheels
            robot.setDrivePower(pLeftFront, pRightFront, pLeftRear, pRightRear);

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

            telemetry.addData("Drive", "LF:%.2f, RF:%.2f, LR:%.2f, RR:%.2f",
                    pLeftFront, pRightFront, pLeftRear, pRightRear);
            telemetry.addData("Winch", "%d",
                    robot.winch.getCurrentPosition());
            telemetry.addData("Shooter", "%.2f",
                    robot.shooter.getVelocity());
            telemetry.addData("Roller", "Power:%.2f, Velocity:%.2f",
                    robot.roller.getPower(), robot.roller.getVelocity());
            telemetry.addData("Wobble", "Arm:%.2f Dist:%.1f Claw: %.2f",
                    robot.wrist.getPosition(), wobbleDetector.getCurrentDistance(), robot.claw.getPosition());

            // Provide feedback as to where the robot is located (if we know).
            if (navigator.targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = navigator.lastLocation.getTranslation();
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(navigator.lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Position", "{X, Y, H} = %.1f, %.1f, %.1f",
                        (translation.get(0) / navigator.mmPerInch),
                        (translation.get(1) / navigator.mmPerInch),
                        rotation.thirdAngle);
            }

            telemetry.update();
        }

        // Disable Tracking when we are done;
        navigator.targetsUltimateGoal.deactivate();
    }
}
