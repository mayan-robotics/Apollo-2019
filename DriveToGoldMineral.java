package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


import org.firstinspires.ftc.robotcore.external.tfod.Recognition;



@Autonomous(name="Test: Drive to gold mineral", group="Test")
public class DriveToGoldMineral extends LinearOpMode {
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private ElapsedTime runtime = new ElapsedTime();

    public enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE,
        OUTOFRANGE
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

        telemetry.addData("Version", robot.Version);
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        waitForStart();

        // Get the main position of gold mineral at start.
        telemetry.addData("Gold Mineral Position is", getLocation());
        telemetry.update();
        waitSeconds(2);
        // Drive until the gold mineral is in front of the robot, by camera.
        while (opModeIsActive() && getLocation() != GoldPosition.MIDDLE) {
                moveToGoldMineralByVuforia();
        }

        telemetry.addData("Vision","cube moved");
        tfod.shutdown();

    }

    // Function to wait an amount of seconds.
    public void waitSeconds(double seconds)
    {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
        }
    }

    // Function drives until the gold mineral is in the middle by our camera.
    public void moveToGoldMineralByVuforia(){
        if(getVuforiaGoldMineralPosition()!= null) {
            telemetry.addData("Vision gold position", getVuforiaGoldMineralPosition());
            if (getLocation() == GoldPosition.LEFT) {
                telemetry.addData("Drive", "left");
            }
            else if (getLocation() == GoldPosition.RIGHT) {
                telemetry.addData("Drive", "right");
            }
            else if (getLocation() == GoldPosition.MIDDLE)
            {
                telemetry.addData("Drive", "Middle-stop");
            }
            else {
                telemetry.addData("camera", "Error");
            }
        }
    }

    // Function converts the location of the gold mineral on the camera
    // to the real location of the gold mineral compare to the silver minerals.
    public GoldPosition getLocation(){
        switch (getVuforiaGoldMineralPosition()){
            case LEFT:
                return GoldPosition.LEFT;
            case RIGHT:
                return GoldPosition.MIDDLE;
            case MIDDLE:
                return GoldPosition.LEFT;
            case OUTOFRANGE:
                return GoldPosition.RIGHT;
        }
        return null;
    }

    // Function returns the location of the gold mineral compare to the camera.
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
                                    telemetry.update();
                                    if (goldMineralX < robot.MineralMiddleLimitLeft) {
                                        return GoldPosition.LEFT;
                                    } else if (goldMineralX > robot.MineralMiddleLimitRight) {
                                        return GoldPosition.RIGHT;
                                    } else if (goldMineralX > robot.MineralMiddleLimitLeft && goldMineralX < robot.MineralMiddleLimitRight) {
                                        return GoldPosition.MIDDLE;
                                    }
                                }
                            }
                        }
                        else {
                            return GoldPosition.OUTOFRANGE;
                        }
                    }
                }
            }
        }
        return null;
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