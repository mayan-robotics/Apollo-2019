package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


import org.firstinspires.ftc.robotcore.external.tfod.Recognition;



@Autonomous(name="Test: Drive to gold mineral", group="Test")
public class DriveToGoldMineral extends LinearOpMode {
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware

    public enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE
    }

    private static final String VUFORIA_KEY = "AQT2n9X/////AAABmTHqWr1WKUlSj7NxBU33qTA0HRGAoksXA9zdFamKhdujhotytdIIzsLYwMwbicUTOi41/mleQl+TD9QHEdKYY46CtD9/TnAlt0Tn4LJAujx3TqMapoGbM01NegyxhPQ3hlwehRB5tIPgJENUNbMfbKCyobKRUmxFC80UunnDcTLvyStsC8+FZUSGpQjqxpXXErSV933EAedlP0q7GXDgy9mUe3oCAMpWfU5zXSf+4sREDQzWBtuSSixhkov3eQ7+r/QH6Q5Di0SY+iQrZaZI27wem8B2i1SgAw+r3rX/LA2LjFYHQn6VePO2iVYkFt0twV/sv8KUkgByLezTnYJSqkKYEeQBODroVHcR98CB8xq9";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /* local OpMode members. */
    //HardwareMap hwMap  =  null;


    @Override
    public void runOpMode() throws InterruptedException {
        //robot.init(hardwareMap);
        //robot.initWebcamVuforia();
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {

            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
        }

        while (opModeIsActive())
        {
            if (opModeIsActive()) {
                moveToGoldMineralByVUforia();
            }
        }
        tfod.shutdown();

    }

    //Function returns the location of the gold mineral.
    public GoldPosition getVuforiaGoldMineralPosition(){
        if (opModeIsActive()) {

            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
        }
        if (tfod != null) {
            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() >= 1) {
                            int goldMineralX;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    telemetry.addData("GoldX", goldMineralX);
                                    telemetry.addData("# Object Detected2", updatedRecognitions.size());
                                    //telemetry.update();
                                    if (goldMineralX < robot.MineralMiddleLimitLeft) {
                                        telemetry.addData("Gold Position", "Left");
                                        //telemetry.update();
                                        return GoldPosition.LEFT;
                                    } else if (goldMineralX > robot.MineralMiddleLimitRight) {
                                        telemetry.addData("Gold Position", "Right");
                                        telemetry.update();
                                        return GoldPosition.RIGHT;
                                    } else if (goldMineralX > robot.MineralMiddleLimitLeft && goldMineralX < robot.MineralMiddleLimitRight) {
                                        telemetry.addData("Gold Position", "Middle");
                                        telemetry.update();
                                        return GoldPosition.MIDDLE;
                                    }
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
        return null;
    }

    //Function drives until the gold mineral is in the middle by our camera.
    public void moveToGoldMineralByVUforia(){
        if(getVuforiaGoldMineralPosition()!= null) {
            if (getVuforiaGoldMineralPosition() == GoldPosition.LEFT) {
                telemetry.addData("Drive", "left");
                //robot.setDriveMotorsPower(-0.3, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            } else if (getVuforiaGoldMineralPosition() == GoldPosition.RIGHT) {
                telemetry.addData("Drive", "right");
                //robot.setDriveMotorsPower(0.3, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            }
            else if (getVuforiaGoldMineralPosition() == GoldPosition.MIDDLE) {
                telemetry.addData("Drive", "Middle");
                //robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
            }else {
                telemetry.addData("camera", "NO GOLD MINERAl");
            }
        }

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}