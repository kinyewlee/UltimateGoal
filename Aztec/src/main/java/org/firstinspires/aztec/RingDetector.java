package org.firstinspires.aztec;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.aztec.RingLayout;
import org.firstinspires.aztec.IObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class RingDetector implements IObjectDetector<RingLayout> {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private final LinearOpMode opMode;
    private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod;

    private RingLayout ringDetected = RingLayout.None;
    float visibleMidpoint = -1f;
    double visibleAngle = 0d;
    List<Recognition> ringRecognitions;

    public RingDetector(LinearOpMode opMode, float confidence) {
        this.opMode = opMode;

        initVuforia();
        initTfod(confidence);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
    }

    private void initTfod(float confidence) {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = confidence;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        // The TensorFlow software will scale the input images from the camera to a lower resolution.
        // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
        // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        // should be set to the value of the images used to create the TensorFlow Object Detection model
        // (typically 16/9).
        tfod.setZoom(1.2, 16.0/9.0);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = Config.VUFORIA_KEY;
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.useExtendedTracking = false;

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    @Override
    public RingLayout objectDetected() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                ringRecognitions = new ArrayList<>();
                ringDetected = RingLayout.None;

                for (Recognition recognition : updatedRecognitions) {
                    float recognitionLeft = Math.max(recognition.getLeft(), 0L);
                    float recognitionWidth = recognition.getRight() - recognitionLeft;
                    float recognitionMid = recognitionLeft + (recognitionWidth / 2L);

                    if (recognition.getLabel() == LABEL_SECOND_ELEMENT || recognition.getLabel() == LABEL_FIRST_ELEMENT) {
                        visibleMidpoint = recognitionMid;
                        visibleAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {
                            ringDetected = RingLayout.Single;
                        } else {
                            ringDetected = RingLayout.Quad;
                        }

                        //ringDetected = recognition.getLabel() == LABEL_SECOND_ELEMENT ?
                        //RingLayout.Single : RingLayout.Quad;
                        ringRecognitions.add(recognition);
                    }
                }
            }
        }

        return ringDetected;
    }

    @Override
    public void reset() {
        ringDetected = RingLayout.None;
        visibleAngle = 0d;
        visibleMidpoint = -1f;
    }

    @Override
    public void dispose() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
