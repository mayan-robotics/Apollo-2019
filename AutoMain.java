package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;


/**
 * Apollo Auto Main.
 */

public abstract class AutoMain extends RobotFunctions
{
    //HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    //RobotFunctions functions = new RobotFunctions(); // use Apollo's hardware

    private ElapsedTime runtime = new ElapsedTime();

    private MineralVision vision;       // Use our vision class.
    private List<MatOfPoint> contoursGold = new ArrayList<>();


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 1;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.6;     // Nominal half speed for better accuracy.
    static final double SIDE_WAYS_DRIVE_SPEED = 1;     // Nominal speed for better accuracy.


    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.08;     // Larger is more responsive, but also less stable

    static final double goldMineralServoCloseLeft = 0.5;
    static final double goldMineralServoCloseRight = 0.6;

    int TURNRIGHTORLEFT = 1;    /* This number controls the directions of the robot,
                                   if its 1 -> the robot will turn right, to our crater,
                                   if its -1 -> the robot will turn left to the other crater. */

    boolean didInit = false;    // Boolean we use to know if we finished our init.
    int gyroDegrees = 0;    // Counter of gyro angle




    // Declaration of gold positions.
    private enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE,
        OUTOFRANGE
    }

    GoldPosition getStartGoldMineralPosition   = null ;     // The Gold Mineral position on the filed.

    Thread  duringClimb = new duringClimb();
    Thread  duringDrive = new duringDrive();


    //Init function, hardwareMap
    public void apolloInit() {
        //Hardware init
        robot.init(hardwareMap);
        robot.InitServoes();
        robot.setAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.push.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.imuRestart();
        gyroDegrees=0;  // Reset gyro angle.

        // Init camera and turn it on.
        try {
            InitMyVision();
        }
        catch (InterruptedException e) {

        }

        // Send telemetry message to signify robot is ready;
        TelementryRobotStartStatus();
    }

    public void testInterruptedException(){

        try {
            encoderPush(1,300);
        }catch (InterruptedException e){
            InterruptedException ex = e;
            robot.push.setPower(0);
            telemetry.addData("e:",e);
            telemetry.update();
        }

    }

    public void liftDown(){
        try
        {
            liftUntilStuck(1);
            encoderLift(1, (robot.lift.getCurrentPosition()-100));

        }catch (InterruptedException e){

        }
    }

    public void passMinrals(){
        try {
            mineralUp();
        }catch (InterruptedException e){

        }
    }

    public void grabMinerals(){
        try
        {
            liftUntilStuck(-1);
            robot.mineralGrab.setPosition(BACKWARDS);
            waitSeconds(3);
            robot.mineralGrab.setPosition(STOP);

        }catch (InterruptedException e){

        }

    }

    //The main function of the autonomous
    void apolloRun(boolean isCrater)
    {


        try {
            encoderLift(1,180);
            waitSeconds(1);
            encoderLift(1,0);
            waitSeconds(999);

            try {
                encoderPush(1,300);
            }catch (InterruptedException e){
                robot.push.setPower(0);
                telemetry.addData("e:",e);

                telemetry.update();
            }

            //encoderMineralSend(1, 700);
            waitSeconds(999);
            //encoderClimbVision(1, robot.climbOpenPosition);     // Get down from lender, and at the same time open systems and process image.

            turnAwayFromLender();     // Turn towards the minerals.
            encoderSideWaysDrive(1, -15);

            gyroDrive(DRIVE_SPEED, 50,angelForGyro(0));

            // According to the image processing we did.
            switch (getStartGoldMineralPosition) {
                case LEFT:      // If gold mineral is on the left.
                    encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, -90);
                    break;
                case RIGHT:     // If gold mineral is on the right.
                    encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, 90);
                    break;
                case MIDDLE:    // If gold mineral is on the middle, Close both servos.

                    break;
            }

            waitSeconds(999);

        }catch (InterruptedException e) { }


        /** If Crater **/
        if(isCrater)
        {
            try {
            if(getStartGoldMineralPosition==GoldPosition.MIDDLE)
            {   // If gold mineral is in te middle.
                encoderLift(1, 675);
                robot.mineralGrab.setPosition(FORWARD);
                gyroDrive(DRIVE_SPEED,95,angelForGyro(0));
                robot.mineralGrab.setPosition(STOP);
                encoderLift(1, 550);
                gyroDrive(DRIVE_SPEED,5,angelForGyro(0));
                encoderLift(1, 670);

            }
            gyroDrive(DRIVE_SPEED,115,angelForGyro(0));

            robot.mineralGrab.setPosition(FORWARD);        // Try to grab minerals.
            encoderLift(1, 685);
            encoderPush(1,3500);

            while (opModeIsActive()){            }  // Keep trying to grab minerals until times up.
            }catch (InterruptedException e) { }

        }

        /** If Depot **/
        else if (!isCrater)
        {
            try{
            if(getStartGoldMineralPosition==GoldPosition.MIDDLE)
            {   // If gold mineral is in te middle.
                encoderLift(1, 670);
                gyroDrive(DRIVE_SPEED, 175, angelForGyro(0));

                //encoderLift(1, 675);

                robot.mineralGrab.setPosition(BACKWARDS);
                //waitSeconds(0.5);

                encoderLift(1, 580);
                //gyroDrive(DRIVE_SPEED, -40, angelForGyro(0));
                //robot.setMineralGrabServos(0.8);
                //gyroDrive(DRIVE_SPEED, -90, angelForGyro(0));
                gyroDrive(DRIVE_SPEED, -132, angelForGyro(0));
                robot.mineralGrab.setPosition(STOP);
                //encoderLift(1, 665);
                //gyroDrive(DRIVE_SPEED, 32, angelForGyro(0));
                //encoderLift(1, 550);

            }else
            {
                gyroDrive(DRIVE_SPEED, 140, angelForGyro(0));
                encoderLift(1, 505);
                robot.mineralGrab.setPosition(BACKWARDS);
                waitSeconds(0.5);
                robot.mineralGrab.setPosition(STOP);

                gyroDrive(DRIVE_SPEED, -115, angelForGyro(0));
            }

            turnByGyro(0.6, angelForGyro(-95 * TURNRIGHTORLEFT));
            gyroDrive(DRIVE_SPEED, 250, angelForGyro(-3  * TURNRIGHTORLEFT));
            encoderLift(1, 675);


            robot.mineralGrab.setPosition(FORWARD);    // Try to grab minerals.
            while (opModeIsActive()){            }  // Keep trying to grab minerals until times up.
            }catch (InterruptedException e) { }

        }

    }

    // This function  sets all the positions for the robot to be ready for running.
    public void startRobotInit(){
        try{


        encoderPush(1,1400);
        encoderLift(1,260);
        }catch (InterruptedException e) { }

    }



    public void robotGrabMineral(){
        try {
            robot.mineralBoxServo.setPosition(1);
            encoderPush(1, 2000);
            encoderLift(1, 680);

            robot.mineralGrab.setPosition(FORWARD);

            encoderPush(1, 4000);
            encoderPush(1, 2000);
            robot.mineralGrab.setPosition(STOP);
            encoderLift(1, 650);
            encoderPush(1, 100);

            robot.mineralBoxServo.setPosition(1);
            //waitSeconds(1);
            robot.blockMineralServo.setPosition(robot.dontBlock);
            encoderLift(1, 190);

            waitSeconds(2.5);
            robot.mineralBoxServo.setPosition(1);
            encoderMineralSend(1, 7800);
            robot.mineralSend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        // Yotam helped
            robot.mineralBoxServo.setPosition(0.4);
            waitSeconds(2);
            encoderMineralSend(1, 20);
        }catch (InterruptedException e) { }
    }

    // This function turns away from lender
    public void turnAwayFromLender(){
        try {
        encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, -40);
        turnByGyro(TURN_SPEED, angelForGyro(90));
        }catch (InterruptedException e) { }
    }


    // Function return gold mineral Location after image processing.
    public GoldPosition visionProcessing(){
        GoldPosition Vision = null;
        try {
            GetGoldLocation();
            Vision=getRealLocation();
            telemetry.addData("Gold Mineral Position", getRealLocation());
            telemetry.update();
        }catch (Exception e)
        {   // if failed send error.
            telemetry.addData("ERROR","Vision");
            telemetry.update();
        }
        return Vision;
    }

    // Function will be used to keep track of the gyro positions.
    public int angelForGyro(int degreesWanted){
        gyroDegrees=(int)(gyroDegrees+degreesWanted);
        return gyroDegrees;
    }


    // Function to wait an amount of seconds.
    public void waitSeconds(double seconds)
    {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) { }
    }


    // Function converts the location of the gold mineral on the camera
    // to the real location of the gold mineral compare to the silver minerals.
    public GoldPosition getRealLocation(){
        switch ((GetGoldLocation())){
            case LEFT:
                return GoldPosition.LEFT;
            case RIGHT:
                return GoldPosition.MIDDLE;
            case MIDDLE:
                return GoldPosition.MIDDLE;
            case OUTOFRANGE:
                return GoldPosition.RIGHT;
        }
        return null;
    }

    // Image processing. Function returns the location of the gold mineral on the camera.
    public GoldPosition GetGoldLocation(){
        try {
            List<MatOfPoint> contoursGold = new ArrayList<>();
            vision.setShowCountours(true);
            contoursGold.clear();
            vision.getGoldContours(contoursGold);

            if ((vision.goldMineralFound() == true) && (contoursGold != null)) {
                if (!contoursGold.isEmpty()) {
                    if (contoursGold.size() >= 1) {
                        if (Imgproc.boundingRect(contoursGold.get(0)) != null) {
                            Rect GoldBoundingRect = Imgproc.boundingRect(contoursGold.get(0));
                            int goldYPosition = GoldBoundingRect.y;    // Get gold mineral Position on camera.

                            if (goldYPosition < 400) {  // If gold mineral position is on the left part of the camera.
                                return GoldPosition.LEFT;
                            } else if (goldYPosition > 400) {   // If gold mineral position is on the right part of the camera.
                                return GoldPosition.RIGHT;
                            }
                        }
                    }
                }
            } else {
                return GoldPosition.OUTOFRANGE;
            }
        }catch (Exception e){
            telemetry.addData("Vision", "Error");
            telemetry.update();
        }
        return null;}


//made by nir
    private class duringClimb extends Thread
    {
        public duringClimb()
        {
            this.setName("duringClimb");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            if (opModeIsActive()) {
                while (!isInterrupted()) {

                }
            }

        }
    }


    private class duringDrive extends Thread
    {
        public duringDrive()
        {
            this.setName("duringDrive");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            if (opModeIsActive()) {
                while (!isInterrupted()) {

                }
            }

        }
    }

}


