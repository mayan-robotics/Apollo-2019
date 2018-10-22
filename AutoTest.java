package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Apollo Autonomus.
 */
@Autonomous(name="Vision robot test", group="Apollo")
public class AutoTest extends LinearOpMode {
    public enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE
    }

    private MineralVision vision;
    private List<MatOfPoint> contoursGold = new ArrayList<>();

    HardwareTeleopTest robot = new HardwareTeleopTest(); // use Apollo's hardware

    private ElapsedTime     runtime = new ElapsedTime();

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED  = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF      = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF     = 0.15;     // Larger is more responsive, but also less stable
    static final double     middel = 0.5;
    static final double     left = 0.4;
    static final double     right = 0.6;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system
        vision.enable();

        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            vision.setShowCountours(true);
            moveToGoldMineralByCamera();

        }
        vision.disable();

    }

    //Function returns the location of the gold mineral.
    public GoldPosition GetGoldLocation(){

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
        } //else {
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
                robot.setDriveMotorsPower(0.5, HardwareTeleopTest.DRIVE_MOTOR_TYPES.RIGHT);
            } else if (GetGoldLocation() == GoldPosition.RIGHT) {
                telemetry.addData("Drive", "right");
                robot.setDriveMotorsPower(0.5, HardwareTeleopTest.DRIVE_MOTOR_TYPES.LEFT);            }
            else if (GetGoldLocation() == GoldPosition.MIDDLE) {
                telemetry.addData("Drive", "Middle");
                robot.setDriveMotorsPower(0, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);
            }else {
                telemetry.addData("camera", "error");
            }
        }

    }
}


