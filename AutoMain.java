package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.INIT_ERRORCODE;

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

    static final int gyroAngleForGoldMineralLeft = 30;
    static final int gyroAngleForGoldMineralMiddle = 0;
    static final int gyroAngleForGoldMineralRight = -30;

    //static final int climbEncoderOpen = 5000;

    Thread  during = new during();

    ThreadActions currentActionThread;


    // Declaration of gold positions.
    public enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE,
        OUTOFRANGE
    }

    public enum ThreadActions{
        VISIONCLIMB,
        LIFTSTART,
        TOGOLDMINERAL,
        EXTRUSIONS,
        LIFTUP,
        DRIVEFORWARD,
        LIFTDOWN,
        GRABMINERAL,
        PUSH,
        PUTMINERALDRIVE,
        DRIVETOCRATER

    }

    public enum GamePositions{
        CRATER,
        DEPOT,
        OURCRATER,
        OTHERCRATER,
        PARKFORWADRS,
        PARKBACKWARDS
    }

    GamePositions startRobotPosition;
    GoldPosition startGoldMineralPosition   = null ;     // The Gold Mineral position on the filed.
    GamePositions parking   = null ;     // The Gold Mineral position on the filed.
    GamePositions sideParking   = null ;     // The Gold Mineral position on the filed.



    int turnToCrater ;

    //Thread  duringOne = new duringOne();
    Thread  duringTwo = new duringTwo();
    ThreadActions threadOneCurrentAction;
    ThreadActions threadTwoCurrentAction;


    //Init function, hardwareMap
    public void apolloInit() {
        //Hardware init
        robot.init(hardwareMap);
        robot.InitServoes();
        robot.setAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.push.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.imuRestart();
        gyroDegrees=0;  // Reset gyro angle.
        robot.climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Init camera and turn it on.
        try {
            InitMyVision();
        }
        catch (InterruptedException e) {        }

        // Send telemetry message to signify robot is ready;
        TelemetryRobotStartStatus();
    }

    public void testInterruptedException(){
        try {
            encoderPush(1,300);
        }catch (InterruptedException e){
            InterruptedException ex = e;
            if(ex.getMessage()== "push"){

            }

            robot.push.setPower(0);
            telemetry.addData("e:",e);
            telemetry.update();
        }

    }

    public int parking(GamePositions crater){
        switch (crater){
            case OURCRATER:
                return -1;
            case OTHERCRATER:
                return 1;
        }
        return -1;
    }


    public void setGameParameters(GamePositions startPosition, GamePositions whichCraterToPark, GamePositions side){
        startRobotPosition=startPosition;
        parking = whichCraterToPark;
        turnToCrater = parking(whichCraterToPark);
        sideParking = side;


    }

    public void climbDown()throws InterruptedException{
        try{
            //RunThread(duringTwo,ThreadActions.LIFTSTART);
            RunThread(during, ThreadActions.VISIONCLIMB);
            encoderClimb(1,climbEncoderOpen);
            during.interrupt();
        }catch (InterruptedException e){
            throw new InterruptedException();
        }
    }

    public void turnToGoldMineral(){

        turnAwayFromLender(startGoldMineralPosition);

    }

    public void mainMoveGoldMineral() {
        switch (startRobotPosition){
            case CRATER:
                moveGoldMineral();
                break;
            case DEPOT:
                if(startGoldMineralPosition==GoldPosition.MIDDLE){
                    RunThread(during, ThreadActions.DRIVEFORWARD);
                    moveGoldMineral();
                    robot.mineralGrab.setPosition(BACKWARDS);
                }
                else {
                    moveGoldMineral();
                }
        }
    }

        public void moveGoldMineral(){
        try {
            telemetry.addData("here","yaroa");
            telemetry.update();
            RunThread(during, ThreadActions.LIFTDOWN);

            turnAwayFromLender(startGoldMineralPosition);
            RunThread(duringTwo,ThreadActions.EXTRUSIONS);
            robot.push.setPower(-1);
            waitSeconds(2);
            pushClose(0.6);
            /*
            robot.push.setPower(-1);
            waitSeconds(0.2);
            robot.push.setPower(0);
            */

            during.interrupt();

        }catch (InterruptedException e){

        }
    }

    public void mainPutMarker() throws InterruptedException{
        try{
            switch (startRobotPosition) {
                case CRATER:
                    fromCraterToPutMarker();
                    break;
                case DEPOT:
                    fromDepotToPutMarker();
                    break;
            }
        }catch (InterruptedException e){
            throw new InterruptedException();
        }
    }

    public void fromCraterToPutMarker() throws InterruptedException{
        try {
            RunThread(during, ThreadActions.LIFTUP);
            robot.mineralGrab.setPosition(STOP);
            turnByGyro(TURN_SPEED, angelForGyro(-getTheGyroAngleToTurnToTheGoldMineral(startGoldMineralPosition)));
            gyroDrive(1, 30, angelForGyro(0));

            //telemetry.addData("here2","hello");
            //telemetry.update();
            //waitSeconds(2);
            RunThread(duringTwo, ThreadActions.LIFTDOWN);
            goFromCraterToDepot(1);

            RunThread(during,ThreadActions.DRIVEFORWARD);



            robot.push.setPower(-1);
            RunThread(duringTwo,ThreadActions.EXTRUSIONS);
            robot.push.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.mineralGrab.setPosition(BACKWARDS);
            waitSeconds(2);

            robot.push.setPower(1);
            waitSeconds(0.3);
            robot.push.setPower(0);
            robot.mineralGrab.setPosition(STOP);

        }catch (InterruptedException e){
            throw new InterruptedException();
        }
    }

    public void fromDepotToPutMarker() throws InterruptedException{
        try {

            //RunThread(during,ThreadActions.DRIVEFORWARD);
            RunThread(duringTwo, ThreadActions.LIFTDOWN);

            robot.push.setPower(-1);
            waitSeconds(2);
            robot.mineralGrab.setPosition(BACKWARDS);

            robot.push.setPower(0);

        }catch (InterruptedException e){
            throw new InterruptedException();
        }
    }


    // This function turns away from lander
    public void turnAwayFromLender(GoldPosition goldPosition){
        try {
            gyroDrive(DRIVE_SPEED, 20, angelForGyro(0));
            encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, -30,false);
            gyroDrive(DRIVE_SPEED, -30, angelForGyro(0));
            turnByGyro(TURN_SPEED, angelForGyro(90+
                    getTheGyroAngleToTurnToTheGoldMineral(goldPosition)));
        }catch (InterruptedException e) { }
    }


    public void pushGoldMineral(){
        try
        {

            turnByGyro(TURN_SPEED,angelForGyro(-getTheGyroAngleToTurnToTheGoldMineral(startGoldMineralPosition)));
        }catch (InterruptedException e){

        }
    }

    public void goPutMarker(){

    }



    public void RunThread( Thread thread, ThreadActions action){
        thread.interrupt();
        if(thread == during) {
            threadOneCurrentAction = action;
        }else if(thread == duringTwo){
            threadTwoCurrentAction = action;
        }
        thread.start();
    }



    public void goFromCraterToDepot(int turnTo) throws InterruptedException{
        try {
            turnByGyro(TURN_SPEED, angelForGyro(90 * turnTo));
            gyroDrive(DRIVE_SPEED, 120,angelForGyro(0));
            turnByGyro(TURN_SPEED, angelForGyro(45 * turnTo));
            encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED,85 *turnTo,false);

        }catch (InterruptedException e){
            throw new InterruptedException();
        }
    }

    public void backToCraterFromDepot(){
        try {
            switch (parking) {
                case OURCRATER:
                    if(sideParking == GamePositions.PARKFORWADRS) {
                        RunThread(during, ThreadActions.LIFTUP);
                        gyroDrive(DRIVE_SPEED, -100, angelForGyro(0));
                        gyroTurn(TURN_SPEED, angelForGyro(angelForGyro(-175)));
                        RunThread(during, ThreadActions.LIFTDOWN);
                        gyroTurn(TURN_SPEED, angelForGyro(angelForGyro(-65)));
                        gyroDrive(DRIVE_SPEED, 60, angelForGyro(0));
                        robot.mineralGrab.setPosition(FORWARD);
                        waitSeconds(10);

                    /*
                    gyroDrive(DRIVE_SPEED, -30, angelForGyro(0));
                    gyroTurn(TURN_SPEED, 20);
                    gyroDrive(DRIVE_SPEED, 80, angelForGyro(0));
                    */
                    }
                    else{
                        gyroDrive(DRIVE_SPEED,-240,angelForGyro(-3));

                    }
                    break;
                case OTHERCRATER:
            }
        }catch (InterruptedException e){

        }

    }

    public void liftDown() throws InterruptedException{
        try
        {
            liftUntilStuck(1);
            encoderLift(1, (robot.lift.getCurrentPosition() - 115));
            robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }catch (InterruptedException e){
            throw new InterruptedException();
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

    public void grabMineralsAndPutInLander(){
        try {
            RunThread(during,ThreadActions.DRIVETOCRATER);
            RunThread(duringTwo,ThreadActions.EXTRUSIONS);
            waitSeconds(0.2);
            during.interrupt();
            //duringTwo.interrupt();
            waitSeconds(2);
            //WaitUntilThreadIsActive(during);
            //WaitUntilThreadIsActive(duringTwo);

            robot.mineralGrab.setPosition(FORWARD);
            RunThread(duringTwo,ThreadActions.PUSH);
            RunThread(during, ThreadActions.GRABMINERAL);
            waitSeconds(1.5);
            //during.interrupt();

            telemetry.addData("Yarboa", "here");
            telemetry.update();
            robot.push.setPower(-1);
            waitSeconds(2);
            //robot.push.setPower(0);

            telemetry.addData("HERE", "HERE");
            telemetry.update();
            pushClose(0.6);

            //waitSeconds(1);



            encoderMineralSend(1,0);
            liftUntilStuck(-1);
            robot.blockMineralServo.setPosition(robot.dontBlock);   // Set Mode of servo to not block minerals.
            robot.mineralGrab.setPosition(FORWARD);
            waitSeconds(0.4);

            robot.mineralGrab.setPosition(STOP);
            RunThread(during,ThreadActions.PUTMINERALDRIVE);
            senderGood(1,1500);
            //encoderMineralSend(1, 1500);
            robot.mineralSend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //robot.mineralGrab.setPosition(STOP);
            robot.mineralBoxServo.setPosition(robot.mineralBoxServoClose);
            telemetry.addData("Yrboa", "HERE");
            telemetry.update();

            waitSeconds(5);
        }catch (InterruptedException e){ }
    }

    public void getReadyToGrabMinerals(){
        try {
            RunThread(during, ThreadActions.LIFTUP);
            turnByGyro(TURN_SPEED, angelForGyro(-getTheGyroAngleToTurnToTheGoldMineral(startGoldMineralPosition)));
            //RunThread(duringTwo,ThreadActions.DRIVETOCRATER);
            //gyroDrive(DRIVE_SPEED, 105, angelForGyro(0));
        }catch (InterruptedException e){

        }

    }




    public int getTheGyroAngleToTurnToTheGoldMineral(GoldPosition goldPosition){
        switch (goldPosition){
            case LEFT:
                return  gyroAngleForGoldMineralLeft;
            case MIDDLE:
                return gyroAngleForGoldMineralMiddle;
            case RIGHT:
                return gyroAngleForGoldMineralRight;
        }
        return 0;
    }


    // Function return gold mineral Location after image processing.
    public GoldPosition ImageProcessing(){
        GoldPosition position = null;
        try {
            //GetGoldLocation();
            position=getRealLocation();
            telemetry.addData("Gold Mineral Position", position);
            telemetry.update();
        }catch (Exception e)
        {   // if failed send error.
            telemetry.addData("ERROR","Vision");
            telemetry.update();
        }
        return position;
    }

    // Function will be used to keep track of the gyro positions.
    public int angelForGyro(int degreesWanted){
        gyroDegrees=(int)(gyroDegrees+degreesWanted);
        return gyroDegrees;
    }


    // Function to wait an amount of seconds.
    public void waitSeconds(double seconds) throws InterruptedException
    {
        try {
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < seconds)) {Thread.sleep(threadSleepTimeMS); }
        }catch (InterruptedException e){
        }
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



        /** ---------------------------------------------------------- **/

        private class during extends Thread
        {
            public during()
            {
                this.setName("during");

            }

            // called when tread.start is called. thread stays in loop to do what it does until exit is
            // signaled by main code calling thread.interrupt.
            @Override
            public void run()
            {
                if (opModeIsActive()) {
                    //while (!isInterrupted()) {
                    try {
                        if(threadOneCurrentAction == ThreadActions.VISIONCLIMB){
                            while (opModeIsActive()){
                                startGoldMineralPosition=ImageProcessing();
                                Thread.sleep(100);
                            }
                        }
                        if (threadOneCurrentAction == ThreadActions.LIFTDOWN) {
                            waitSeconds(1.8);
                            liftDown();
                        }
                        if (threadOneCurrentAction == ThreadActions.LIFTUP) {
                            liftUntilStuck(-1);
                        }
                        if (threadOneCurrentAction == ThreadActions.DRIVEFORWARD) {
                            gyroDrive(DRIVE_SPEED,80,angelForGyro(0));
                        }
                        if(threadOneCurrentAction == ThreadActions.GRABMINERAL){
                            liftUntilStuck(1);
                            encoderLift(1, (robot.lift.getCurrentPosition() - 100));

                            robot.blockMineralServo.setPosition(robot.block);   // Set Mode of servo to not block minerals.
                            robot.mineralGrab.setPosition(FORWARD);
                            telemetry.addData("Finished1", "here");
                            telemetry.update();
                        }
                        if(threadOneCurrentAction == ThreadActions.PUTMINERALDRIVE){
                            waitSeconds(0.2);
                            gyroDrive(0.6,-20,angelForGyro(0));
                        }if(threadOneCurrentAction==ThreadActions.DRIVETOCRATER){
                            gyroDrive(DRIVE_SPEED, 90, angelForGyro(0));
                        }
                    }catch (InterruptedException e){
                        telemetry.addData("Interrupt",e);
                        telemetry.update();
                    }

                    //}
                }

            }
        }



    private class duringTwo extends Thread
    {
        public duringTwo()
        {
            this.setName("duringTwo");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            if (opModeIsActive()) {
                try {
                    if(threadTwoCurrentAction==ThreadActions.LIFTSTART){
                        encoderLift(1,40);
                        liftUntilStuck(-1);
                    }
                    if (threadTwoCurrentAction == ThreadActions.EXTRUSIONS) {
                        encoderMineralSend(1, 150);
                    }
                    if (threadTwoCurrentAction == ThreadActions.LIFTDOWN) {
                        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        liftDown();
                    }
                    if(threadTwoCurrentAction==ThreadActions.PUSH){
                        //waitSeconds(0.2);
                        //robot.push.setPower(1);
                        //waitSeconds(0.4);
                        robot.push.setPower(-1);
                        waitSeconds(0.1);
                        robot.push.setPower(0.6);
                        //robot.mineralGrab.setPosition(FORWARD);
                        waitSeconds(1);
                    }
                }catch (InterruptedException e){

                }
            }

        }
    }

    public void WaitUntilThreadIsActive (Thread currentThread)
    {

        while (currentThread.isAlive())
        {
            sleep(100);
        }
    }

/*
    private class duringTwo extends Thread
    {
        public duringTwo()
        {
            this.setName("duringTwo");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            if (opModeIsActive()) {
                switch (threadTwoCurrentAction){

                }
            }

        }
    }
*/



















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

            //turnAwayFromLender();     // Turn towards the minerals.
            encoderSideWaysDrive(1, -15,true);

            gyroDrive(DRIVE_SPEED, 50,angelForGyro(0));

            // According to the image processing we did.
            switch (startGoldMineralPosition) {
                case LEFT:      // If gold mineral is on the left.
                    encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, -90,true);
                    break;
                case RIGHT:     // If gold mineral is on the right.
                    encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, 90,true);
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
                if(startGoldMineralPosition==GoldPosition.MIDDLE)
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
                if(startGoldMineralPosition==GoldPosition.MIDDLE)
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

}


