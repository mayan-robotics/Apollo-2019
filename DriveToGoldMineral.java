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
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware


    public enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE
    }


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {

            /** Activate Tensor Flow Object Detection. */
            if (robot.tfod != null) {
                robot.tfod.activate();
            }
        }

        while (opModeIsActive())
        {
            if (opModeIsActive()) {
                moveToGoldMineralByVUforia();
            }
        }
        robot.tfod.shutdown();

    }

    //Function returns the location of the gold mineral.
    public GoldPosition getVuforiaGoldMineralPosition(){
        if (opModeIsActive()) {

            /** Activate Tensor Flow Object Detection. */
            if (robot.tfod != null) {
                robot.tfod.activate();
            }
        }
        if (robot.tfod != null) {
            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() >= 1) {
                            int goldMineralX;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(robot.LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    telemetry.addData("GoldX", goldMineralX);
                                    telemetry.addData("# Object Detected2", updatedRecognitions.size());
                                    telemetry.update();
                                    if (goldMineralX < robot.MineralMiddleLimitLeft) {
                                        telemetry.addData("Gold Position", "Left");
                                        telemetry.update();
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

}