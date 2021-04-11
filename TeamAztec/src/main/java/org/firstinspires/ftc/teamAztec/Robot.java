package org.firstinspires.ftc.teamAztec;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    /* Public OpMode members. */
    DcMotorEx leftFront = null;
    DcMotorEx rightFront = null;
    DcMotorEx leftRear = null;
    DcMotorEx rightRear = null;
    DcMotorEx roller = null;
    DcMotorEx shooter = null;
    BNO055IMU imu;
    Servo trigger;

    private boolean soundPlaying = false;
    private int beepSoundID, fireSoundID;
    private SoundPlayer.PlaySoundParams soundParams;
    private Context appContext;

    public Robot(HardwareMap hardwareMap, boolean initIMU) {
        init(hardwareMap);
        if (initIMU) {
            initIMU(hardwareMap);
        }
        initBeep(hardwareMap);
    }

    private void initBeep(HardwareMap hardwareMap) {
        beepSoundID = hardwareMap.appContext.getResources().getIdentifier("beep",
                "raw", hardwareMap.appContext.getPackageName());

        fireSoundID = hardwareMap.appContext.getResources().getIdentifier("fire",
                "raw", hardwareMap.appContext.getPackageName());
        soundParams = new SoundPlayer.PlaySoundParams();
        soundParams.loopControl = 0;
        soundParams.waitForNonLoopingSoundsToFinish = false;

        appContext = hardwareMap.appContext;
        beep();
    }

    public void beep() {
        beep(0);
    }

    public void beep(int type) {
        if (!soundPlaying) {
            soundPlaying = true;
            SoundPlayer.getInstance().startPlaying(appContext, type == 0 ? beepSoundID : fireSoundID, soundParams, null,
                    new Runnable() {
                        public void run() {
                            soundPlaying = false;
                        }
                    });
        }
    }

    /* Initialize standard Hardware interfaces */
    private void init(HardwareMap hardwareMap) {
        // Define and Initialize Motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        // Set all motors to run using encoders.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        roller = hardwareMap.get(DcMotorEx.class, "roller");
        roller.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        trigger = hardwareMap.servo.get("ringFeeder");

        //Initialize imu
        initIMU(hardwareMap);
    }

    private void initIMU(HardwareMap hardwareMap) {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            Thread.yield();
        }
    }

    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }
}
