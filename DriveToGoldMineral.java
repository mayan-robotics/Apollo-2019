package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


import org.firstinspires.ftc.robotcore.external.tfod.Recognition;



@Autonomous(name="Test: Drive to gold mineral", group="Test")
public class DriveToGoldMineral extends LinearOpMode {
    HardwareCam robot = new HardwareCam(); // use Apollo's hardware

    private MineralVision vision;
    private List<MatOfPoint> contoursGold = new ArrayList<>();

    public enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE
    }

    static final int MineralMiddleLimitLeft = 0 ;
    static final int MineralMiddleLimitRight = 0 ;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //vision = new MineralVision();
        //// can replace with ActivityViewDisplay.getInstance() for fullscreen
        //vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        //vision.setShowCountours(false);
        //// start the vision system
        //vision.enable();

        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        waitForStart();

        /** Activate Tensor Flow Object Detection. */
        if (robot.tfod != null) {
            robot.tfod.activate();
        }

        while (opModeIsActive())
        {
            //vision.setShowCountours(true);
            moveToGoldMineralByVUforia();
            telemetry.update();
        }
        //vision.disable();
    }

    //Function returns the location of the gold mineral.
    public GoldPosition GetGoldLocation(){
        try{
        contoursGold.clear();
        vision.getGoldContours(contoursGold);

        if ((vision.goldMineralFound()) && (contoursGold != null)) {
            if (!contoursGold.isEmpty())  {
                if (contoursGold.get(0) != null) {
                    Rect GoldBoundingRect = Imgproc.boundingRect(contoursGold.get(0));//Rect GoldBoundingRect1 = new Rect(0, 0, 10, 10);
                    int goldXPosition = GoldBoundingRect.x;

                    if (goldXPosition < 450) {
                        telemetry.addData("Gold Position", "Left");
                        return GoldPosition.LEFT;
                    } else if (goldXPosition > 650) {
                        telemetry.addData("Gold Position", "Right");
                        return GoldPosition.RIGHT;
                    } else if (goldXPosition > 450 && goldXPosition < 650) {
                        telemetry.addData("Gold Position", "Middle");
                        return GoldPosition.MIDDLE;
                    }
                }
            }
        }
        }catch (Exception e){
            telemetry.addData("Camera", "ERROR");
        }

        return null;}

/*
    public void moveToGoldMineralByCamera(){
        if(GetGoldLocation()!= null) {
            if (GetGoldLocation() == GoldPosition.LEFT) {
                telemetry.addData("Drive", "left");
                //robot.setDriveMotorsPower(-0.3, HardwareCam.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            } else if (GetGoldLocation() == GoldPosition.RIGHT) {
                telemetry.addData("Drive", "right");
                //robot.setDriveMotorsPower(0.3, HardwareCam.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            }
            else if (GetGoldLocation() == GoldPosition.MIDDLE) {
                telemetry.addData("Drive", "Middle");
                //robot.setDriveMotorsPower(0, HardwareCam.DRIVE_MOTOR_TYPES.ALL);
            }else {
                //telemetry.addData("camera", "NO GOLD MINERAl");
            }
        }

    }
*/
    public GoldPosition getVuforiaGoldMineralPosition(){
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            if (updatedRecognitions.size() == 3) {
                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(robot.LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getLeft();
                        if (goldMineralX < MineralMiddleLimitLeft) {
                            telemetry.addData("Gold Position", "Left");
                            return GoldPosition.LEFT;
                        } else if (goldMineralX > MineralMiddleLimitRight) {
                            telemetry.addData("Gold Position", "Right");
                            return GoldPosition.RIGHT;
                        } else if (goldMineralX > MineralMiddleLimitLeft && MineralMiddleLimitRight < 650) {
                            telemetry.addData("Gold Position", "Middle");
                            return GoldPosition.MIDDLE;
                        }
                    }
                }
            }
        }
        return null;
    }


    public void moveToGoldMineralByVUforia(){
        if(getVuforiaGoldMineralPosition()!= null) {
            if (getVuforiaGoldMineralPosition() == GoldPosition.LEFT) {
                telemetry.addData("Drive", "left");
                //robot.setDriveMotorsPower(-0.3, HardwareCam.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            } else if (getVuforiaGoldMineralPosition() == GoldPosition.RIGHT) {
                telemetry.addData("Drive", "right");
                //robot.setDriveMotorsPower(0.3, HardwareCam.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            }
            else if (getVuforiaGoldMineralPosition() == GoldPosition.MIDDLE) {
                telemetry.addData("Drive", "Middle");
                //robot.setDriveMotorsPower(0, HardwareCam.DRIVE_MOTOR_TYPES.ALL);
            }else {
                telemetry.addData("camera", "NO GOLD MINERAl");
            }
        }

    }

}