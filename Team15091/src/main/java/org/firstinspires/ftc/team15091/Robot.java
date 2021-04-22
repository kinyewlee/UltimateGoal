package org.firstinspires.ftc.team15091;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot {
    /* Public OpMode members. */
    DcMotorEx winch = null;
    DcMotorEx shooter = null;
    DcMotorEx roller = null;
    Servo wrist;
    Servo claw;
    Servo trigger;
    DistanceSensor wobbleRange;

    private boolean soundPlaying = false;
    private int beepSoundID, fireSoundID, oneSoundID, threeSoundID;
    private SoundPlayer.PlaySoundParams soundParams;
    private Context appContext;

    public Robot(HardwareMap hardwareMap) {
        init(hardwareMap);
        initBeep(hardwareMap);
    }

    private void initBeep(HardwareMap hardwareMap) {
        beepSoundID = hardwareMap.appContext.getResources().getIdentifier("beep",
                "raw", hardwareMap.appContext.getPackageName());

        fireSoundID = hardwareMap.appContext.getResources().getIdentifier("fire",
                "raw", hardwareMap.appContext.getPackageName());

        oneSoundID = hardwareMap.appContext.getResources().getIdentifier("one",
                "raw", hardwareMap.appContext.getPackageName());

        threeSoundID = hardwareMap.appContext.getResources().getIdentifier("three",
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
            int soundID = beepSoundID;
            switch (type) {
                case 0:
                    soundID = beepSoundID;
                    break;
                case 1:
                    soundID = fireSoundID;
                    break;
                case 2:
                    soundID = oneSoundID;
                    break;
                case 3:
                    soundID = threeSoundID;
                    break;
            }

            SoundPlayer.getInstance().startPlaying(appContext, soundID, soundParams, null,
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
        winch = hardwareMap.get(DcMotorEx.class, "winch");
        winch.setDirection(DcMotorSimple.Direction.FORWARD);
        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        roller = hardwareMap.get(DcMotorEx.class, "roller");
        roller.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        wrist = hardwareMap.servo.get("wrist");
        claw = hardwareMap.servo.get("claw");
        trigger = hardwareMap.servo.get("ringFeeder");

        wobbleRange = hardwareMap.get(DistanceSensor.class, "wobbleRange");
    }


}
