package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.aztec.IObjectDetector;

public abstract class AztecOpMode extends LinearOpMode {
    /* Declare OpMode members. */
    Robot robot;

    private static final double P_TURN_COEFF = 0.05d;     // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF = 0.09d;     // Larger is more responsive, but also less stable
    private static final double HEADING_THRESHOLD = 0.4d;      // As tight as we can make it with an integer gyro

    protected ElapsedTime runtime = new ElapsedTime();

    public final void gyroSlide(double speed,
                                double distance,
                                double angle,
                                double timeoutS,
                                IObjectDetector<Boolean> objectDetector) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Set Target and Turn On RUN_TO_POSITION
            robot.setDriveTarget(distance, true);
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            double counter = 0d;
            runtime.reset();
            double powerToSet = Math.max(speed, 0.5d);
            robot.setDrivePower(powerToSet, powerToSet, powerToSet, powerToSet);
            ElapsedTime driveTime = new ElapsedTime();

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    robot.isDriveBusy()) {
                idle();

                if (objectDetector != null && objectDetector.objectDetected()) {
                    robot.beep();
                    break;
                }
            }

            // Stop all motion;
            robot.setDrivePower(0d, 0d, 0d, 0d);

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public final boolean gyroDrive(double speed,
                                   double distance,
                                   double angle,
                                   double timeoutS,
                                   IObjectDetector<Boolean> objectDetector) {

        boolean successful = true;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Set Target and Turn On RUN_TO_POSITION
            robot.setDriveTarget(distance, false);
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.setDrivePower(speed, speed, speed, speed);
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    robot.isDriveBusy()) {

                idle();

                // adjust relative speed based on heading error.
                double error = getError(angle);
                double steer;
                if (Math.abs(speed) < 0.2d) {
                    steer = getSteer(error, P_DRIVE_COEFF, Math.abs(speed));
                }
                else {
                    steer = getSteer(error, P_DRIVE_COEFF);
                }

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0d)
                    steer *= -1d;

                double speedFL, speedFR, speedRL, speedRR;

                speedFL = speedRL = speed - steer;
                speedFR = speedRR = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                double max = Math.max(Math.abs(speedFL), Math.abs(speedFR));
                if (max > 1d) {
                    speedFL /= max;
                    speedRL /= max;
                    speedFR /= max;
                    speedRR /= max;
                }
                if (objectDetector != null && objectDetector.objectDetected()) {
                    robot.beep();
                    successful = false;
                    break;
                }
                robot.setDrivePower(speedFL,
                        speedFR,
                        speedRL,
                        speedRR);
            }

            // Stop all motion;
            robot.setDrivePower(0d, 0d, 0d, 0d);

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        return successful;
    }

    public final void gyroTurn(double speed, double angle, double timeoutS) {

        runtime.reset();
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() &&
                runtime.seconds() < timeoutS &&
                !onHeading(speed, angle)) {
            // Update telemetry & Allow time for other processes to run.
            idle();
        }

        // Stop all motion;
        robot.setDrivePower(0d, 0d, 0d, 0d);
    }

    private boolean onHeading(double speed, double angle) {
        double error;
        double steer;
        boolean onTarget = false;
        double speedFL, speedFR, speedRL, speedRR;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            speedFL = speedFR = speedRL = speedRR = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, P_TURN_COEFF);
            speedFR = speedRR = speed * steer;
            speedFL = speedRL = speed * -steer;
        }

        // Send desired speeds to motors.
        robot.setDrivePower(speedFL, speedFR,
                speedRL,
                speedRR);

        return onTarget;
    }

    private double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1d, 1d);
    }

    private double getSteer(double error, double PCoeff, double power) {
        return Range.scale(error * PCoeff, -1, 1d, -power, power);
    }
}
