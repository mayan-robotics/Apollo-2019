package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name="Drive to gold mineral", group="Apollo")
public class DriveToGoldMineral extends LinearOpMode {

    public enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE
    }
    int i = 0;

    private MineralVision vision;
    private List<MatOfPoint> contoursGold = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system
        vision.enable();

        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        waitForStart();

        //vision.setShowCountours(true);
        //List<MatOfPoint> goldContours = vision.getGoldContours();
        //GetGoldLocation();
        //moveToGoldMineralByCamera();
        while (opModeIsActive()) {
            vision.setShowCountours(true);
            moveToGoldMineralByCamera();

            telemetry.update();
        }

        vision.disable();
    }


    //Function returns the location of the gold mineral.
    public GoldPosition GetGoldLocation(){

        try{
        contoursGold.clear();
        vision.getGoldContours(contoursGold);

        if ((vision.goldMineralFound()) && (contoursGold != null)) {
            if (!contoursGold.isEmpty())  {
                //Rect GoldBoundingRect = new Rect(0, 0, 10, 10);

                if (contoursGold.get(0) != null) {

                    Rect GoldBoundingRect = Imgproc.boundingRect(contoursGold.get(0));
                    //Rect GoldBoundingRect1 = new Rect(0, 0, 10, 10);

                    int goldXPosition = GoldBoundingRect.x;


                    if (goldXPosition < 450) {
                        //telemetry.addData("Gold Position", "Left");
                        return GoldPosition.LEFT;
                    } else if (goldXPosition > 650) {
                        //telemetry.addData("Gold Position", "Right");
                        return GoldPosition.RIGHT;
                    } else if (goldXPosition > 450 && goldXPosition < 650) {
                        //telemetry.addData("Gold Position", "Middle");
                        return GoldPosition.MIDDLE;
                    }
                }
            }
        }
        }catch (Exception e){
            telemetry.addData("Camera", "ERROR");
        }
        //else {
            //telemetry.addData("Apollo", "did not find a gold mineral");
        //}
        //if (vision.silverMineralFound() == true) {
        //telemetry.addData("Apollo", "found a silver mineral");
        // } else {
        //telemetry.addData("Apollo", "did not find a silver mineral");
        //}
        //telemetry.update();

        return null;}


    public void moveToGoldMineralByCamera(){
        //List<MatOfPoint> goldContours = vision.getGoldContours();
        //if (GetGoldLocation()!= null) {
        //while (GetGoldLocation() != GoldPosition.MIDDLE && opModeIsActive()) {
        //if(vision.goldMineralFound()== true && goldContours.size()  >=1 ) {
        if(GetGoldLocation()!= null) {
            if (GetGoldLocation() == GoldPosition.LEFT) {
                telemetry.addData("Drive", "left");
                //robot.setDriveMotorsPower(SIDE_WAYS_DRIVE_SPEED, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAY_LEFT_DRIVE);
            } else if (GetGoldLocation() == GoldPosition.RIGHT) {
                telemetry.addData("Drive", "right");
                //robot.setDriveMotorsPower(SIDE_WAYS_DRIVE_SPEED, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAY_RIGHT_DRIVE);
            }
            else if (GetGoldLocation() == GoldPosition.MIDDLE) {
                telemetry.addData("Drive", "Middle");
                //robot.setDriveMotorsPower(SIDE_WAYS_DRIVE_SPEED, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAY_RIGHT_DRIVE);
            }else {
                telemetry.addData("camera", "error");
            }
        }

    }
}