package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Vector;

public class Vision {


    private static final String VUFORIA_KEY =
            "AdIUULr/////AAABmQlfWCs9V0IckVYsRVAiKt+M9b2Vyzq80XxPRtYAxTNSZwN36h1rY2y5YQdJen9lLftKY2XUetNF7Cv/xeQNhwBk1L/grLHAAHF4q5Z2FiNFGJbohytHflJTIAjdncCwRKN1O+EZy4CA1qQzNe/+MolD45tZkU4uqiE4Z9VLJs2nU+0vB/MSZDxslo+Yv3t/Xbs4zS+4ghLhNPJaPMOPqDcqegyxsVaOTWIdk2cClYvHqP6+e2ZobePq6fG1+UN8h2s9ifIoq9tBA6rhmh/r8bbEVU9GeDL2u3AHCmnmieziAWqsHXV3TYtOd4rg8KZMUcRIhFjgk3YW0F1i/fuwPQz+pZrIWOR/UWCgzbiZvBCh";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private String label = "nothing";
    private double edgeleftx;
    private double edgerightx;
    public double locationx;
    private double edgetopy;
    private double edgebottomy;
    public double locationy;
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final double MIDLINE = 300;

    // Constructor

    //Init

    //Vuforia Init
    public void initVuforia(HardwareMap hwMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    // TensorFlow Init
    public void initTfod(HardwareMap hwMap) {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public String detect() {
        if (tfod != null) {
            tfod.activate();

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                // step through the list of recognitions and display boundary info.
                for (Recognition recognition : updatedRecognitions) {
                    //TODO: Did we discover more than one item?
                    label = recognition.getLabel();

                    edgeleftx = recognition.getLeft();
                    edgerightx = recognition.getRight();
                    locationx = (edgeleftx+edgerightx)/2;

                    edgetopy = recognition.getTop();
                    edgebottomy = recognition.getBottom();
                    locationy = (edgetopy+edgebottomy)/2;
                }
            }
        }

        return label;
    }

    public double getposistionx() {

        return locationx;

    }
    public double getposistiony() {

        return locationy;

    }

    public int readBarcode(){

        if(label != "nothing"){

            if ( locationx >= MIDLINE){
                // Object must be in position 3
                return 3;
            } else if ( locationx > 0 ){
                // Object must be in position 2
                return 2;
            }
        } else {
            // Object must be in Position 1 (based on our setup)
            return 1;
        }

        // catchall return
        return 1;

    }

    public void endTFOD() {
        if (tfod != null)
        tfod.deactivate();
    }
}