package org.firstinspires.ftc.teamAztec;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Gamepad", group = "Linear Opmode")
public class Gamepad extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, false);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

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

            telemetry.addData("Drive", "LF:%.2f, RF:%.2f, LR:%.2f, RR:%.2f",
                    pLeftFront, pRightFront, pLeftRear, pRightRear);
            telemetry.update();
        }
    }
}
