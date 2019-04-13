package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
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

    private ElapsedTime runtime = new ElapsedTime();
    private MineralVision vision;       // Use our vision class.
    private List<MatOfPoint> contoursGold = new ArrayList<>();


    int gyroDegrees = 0;    // Counter of gyro angle

    static final int gyroAngleForGoldMineralLeft = 33;
    static final int gyroAngleForGoldMineralMiddle = 0;
    static final int gyroAngleForGoldMineralRight = -33;


    static final int craterGyroAngleForGoldMineralLeft = 30;
    static final int craterGyroAngleForGoldMineralMiddle = 0;
    static final int craterGyroAngleForGoldMineralRight = -30;


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
        DRIVEFORWARDMARKER,
        EXTRUSIONS,
        LIFTUP,
        DRIVEFORWARD,
        LIFTDOWN,
        GRABMINERAL,
        PUSH,
        PUTMINERALDRIVE,
        DRIVETOCRATER,
        DRIVEBACKWARD,
        DRIVEBETWEENMINERALGRAB,
        LIFTDOWNGRAB,
        MYDEPOTDRIVEFORWARD,
        MYDEPOTDRIVEBACKWARD

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


    Thread  during = new during();          // Thread number one
    Thread  duringTwo = new duringTwo();    // Thread number Two
    ThreadActions threadOneCurrentAction;
    ThreadActions threadTwoCurrentAction;


    //Init function, hardwareMap
    public void apolloInit() {
        //Hardware init
        robot.init(hardwareMap);
        robot.InitServos();
        robot.setAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.push.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.imuRestart();
        gyroDegrees=0;  // Reset gyro angle.
        robot.climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Init camera and turn it on.
        InitMyVision();


        // Send telemetry message to signify robot is ready;
        TelemetryRobotStartStatus();
    }

    // Function returns the number to control the directions of the robot for going to the crater,
    // if its 1 -> the robot will turn right, to our crater,
    // if its -1 -> the robot will turn left to the other crater.
    public int parking(GamePositions crater){
        switch (crater){
            case OURCRATER:
                return 1;
            case OTHERCRATER:
                return -1;
        }
        return 1;
    }

    // Function returns tthe number which will effect the direction of the robot when turing to move the gold mineral.
    public int depotGoldMineralTurn(GoldPosition goldMineral){
        switch (goldMineral){
            case RIGHT:
                return  1;
            case LEFT:
                return -1;
        }
        return 1;
    }


    // Function sets game parameters.
    public void setGameParameters(GamePositions startPosition, GamePositions whichCraterToPark, GamePositions side){
        startRobotPosition=startPosition;
        parking = whichCraterToPark;
        turnToCrater = parking(whichCraterToPark);
        sideParking = side;
    }


    // Function makes the robot clim down and do Image processing.
    public void climbDown()throws InterruptedException{
        try{
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


    public void moveMineralByServo(){
        try {
            switch (startGoldMineralPosition) {
                case LEFT:
                    break;
                case RIGHT:
                    break;

            }

            gyroDrive(DRIVE_SPEED, 100, angelForGyro(0));
        }catch (InterruptedException e){}
    }

    public void mainMoveGoldMineral() throws InterruptedException {
        try {
            switch (startRobotPosition) {
                case CRATER:
                    moveGoldMineralCrater();
                    break;
                case DEPOT:
                    moveGoldMineralDepot();
            }
        }catch (InterruptedException e){
            throw new InterruptedException();
        }
    }



    public void moveGoldMineralCrater() throws InterruptedException {
        try {
            RunThread(during, ThreadActions.LIFTDOWN);
            turnAwayFromLender(startGoldMineralPosition);
            waitUntilThreadIsNotActive(during);

            RunThread(duringTwo,ThreadActions.EXTRUSIONS);
            RunThread(during, ThreadActions.DRIVEFORWARD);

            robot.push.setPower(-1);
            waitSeconds(0.5);
            pushClose(0.6);

            RunThread(during, ThreadActions.DRIVEBACKWARD);

        }catch (InterruptedException e){
            throw new InterruptedException();
        }
    }


    public void moveGoldMineralDepot() {
        try {
            if(startGoldMineralPosition==GoldPosition.MIDDLE) {
                moveGoldMineralCrater();
                robot.mineralGrab.setPosition(BACKWARDS);
                gyroDrive(DRIVE_SPEED, 100, angelForGyro(0));
            }else{
                moveGoldMineralFromSideDepot();
            }
        }catch (InterruptedException e){

        }
    }

    public void moveGoldMineralCraterWithout(){
        try {

            RunThread(during, ThreadActions.LIFTDOWN);
            waitUntilThreadIsNotActive(duringTwo);
            turnAwayFromLenderWithoutMove(startGoldMineralPosition);

            robot.push.setPower(-1);
            waitSeconds(1.6);
            pushClose(0.6);

        }catch (InterruptedException e){

        }
    }

    public void moveGoldMineralFromSideDepot(){
        try {
            RunThread(during, ThreadActions.LIFTDOWN);
            turnAwayFromLender(startGoldMineralPosition);
            gyroDrive(DRIVE_SPEED, 160, angelForGyro(0));
            gyroTurn(TURN_SPEED, angelForGyro(70 * depotGoldMineralTurn(startGoldMineralPosition)));

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
                    if(startGoldMineralPosition!=GoldPosition.MIDDLE) {
                        putMarkerDepot();
                    }else {
                        waitUntilThreadIsNotActive(duringTwo);
                        waitUntilThreadIsNotActive(during);
                    }
                    break;
            }
        }catch (InterruptedException e){
            throw new InterruptedException();

        }
    }


    public void putMarkerDepot(){
        try {
            robot.mineralGrab.setPosition(BACKWARDS);
            waitUntilThreadIsNotActive(during);
            RunThread(during, ThreadActions.DRIVEFORWARDMARKER);
            waitSeconds(1.5);
            robot.mineralGrab.setPosition(STOP);
        }catch (InterruptedException e){}

    }

    public void fromCraterToPutMarker() throws InterruptedException{
        try {
            RunThread(duringTwo, ThreadActions.LIFTUP);
            robot.mineralGrab.setPosition(STOP);
            waitUntilThreadIsNotActive(during);
            turnByGyro(TURN_SPEED, angelForGyro(-getTheGyroAngleToTurnToTheGoldMineral(startGoldMineralPosition)));
            gyroDrive(1, 45, angelForGyro(0));
            goFromCraterToDepot(1);

            RunThread(during,ThreadActions.DRIVEFORWARD);
            robot.push.setPower(-1);
            RunThread(duringTwo,ThreadActions.EXTRUSIONS);
            robot.mineralGrab.setPosition(BACKWARDS);
            waitSeconds(2);
            pushClose(0.6);
            robot.mineralGrab.setPosition(STOP);

        }catch (InterruptedException e){
            throw new InterruptedException();
        }
    }


    public void fromDepotToPutMarker() throws InterruptedException{
        try {
            RunThread(duringTwo, ThreadActions.LIFTUP);
            turnByGyro(TURN_SPEED, angelForGyro(-getTheGyroAngleToTurnToTheGoldMineral(startGoldMineralPosition)));
            waitUntilThreadIsNotActive(during);
            RunThread(during,ThreadActions.DRIVEFORWARD);
            waitUntilThreadIsNotActive(during);
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
            encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, -30);
            gyroDrive(DRIVE_SPEED, -30, angelForGyro(0));
            turnByGyro(TURN_SPEED, angelForGyro(90+
                    getTheGyroAngleToTurnToTheGoldMineral(goldPosition)));
        }catch (InterruptedException e) { }
    }


    // This function turns away from lander to gold mineral to move without driving.
    public void turnAwayFromLenderWithoutMove(GoldPosition goldPosition){
        try {
            gyroDrive(DRIVE_SPEED, 20, angelForGyro(0));
            encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, -30);
            gyroDrive(DRIVE_SPEED, -30, angelForGyro(0));
            turnByGyro(TURN_SPEED, angelForGyro(90+
                    getTheGyroAngleToTurnToTheGoldMineralWithoutMove(goldPosition)));
        }catch (InterruptedException e) { }
    }


    public void pushGoldMineral(){
        try
        {
            turnByGyro(TURN_SPEED,angelForGyro(-getTheGyroAngleToTurnToTheGoldMineral(startGoldMineralPosition)));
        }catch (InterruptedException e){

        }
    }


    public void RunThread( Thread thread, ThreadActions action){
        thread.interrupt();
        waitUntilThreadIsNotActive(thread);
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
            RunThread(duringTwo, ThreadActions.LIFTDOWN);
            turnByGyro(TURN_SPEED, angelForGyro(45 * turnTo));
            encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED,100 *turnTo);

        }catch (InterruptedException e){
            throw new InterruptedException();
        }
    }

    public void backToCraterFromDepot(){
        try {
            switch (startRobotPosition) {
                case CRATER:
                    if (sideParking == GamePositions.PARKFORWADRS) {
                        RunThread(during, ThreadActions.LIFTUP);
                        gyroDrive(DRIVE_SPEED, -140, angelForGyro(0));
                        gyroTurn(TURN_SPEED, angelForGyro(angelForGyro(-175)));
                        gyroTurn(TURN_SPEED, angelForGyro(angelForGyro(-65)));
                        RunThread(during, ThreadActions.LIFTDOWN);
                        gyroDrive(DRIVE_SPEED, 80, angelForGyro(0));
                        robot.mineralGrab.setPosition(FORWARD);
                    } else {
                        gyroDrive(DRIVE_SPEED, -160, angelForGyro(-3));
                        gyroDrive(0.3, -20, angelForGyro(0));
                    }
                    break;

                case DEPOT:
                    liftUntilStuck(-1);
                    parkFromDepot();
                    break;
            }

        }catch (InterruptedException e){

        }

    }

    public void parkFromDepot(){
        try {
            if (parking == GamePositions.OTHERCRATER) {
                switch (startGoldMineralPosition) {
                    case RIGHT:
                        goFromDepotToCraterFarMineralCase();
                        liftDown();

                        break;
                    case LEFT:
                        if(sideParking == GamePositions.PARKBACKWARDS) {
                            gyroTurn(TURN_SPEED, angelForGyro(40 * depotGoldMineralTurn(startGoldMineralPosition)));
                            gyroDrive(DRIVE_SPEED, -250, angelForGyro(0));
                        }else {
                            goFromDepotToCraterForwardCloseMineralCase();
                            grab();
                            liftDown();
                        }
                        break;
                    case MIDDLE:
                        goFromDepotToCraterMiddleCase();
                        break;

                }
            } else {
                switch (startGoldMineralPosition) {
                    case RIGHT:
                        if(sideParking == GamePositions.PARKBACKWARDS) {
                            gyroTurn(TURN_SPEED, angelForGyro(40 * depotGoldMineralTurn(startGoldMineralPosition)));
                            gyroDrive(DRIVE_SPEED, -250, angelForGyro(0));

                        }else {
                            goFromDepotToCraterForwardCloseMineralCase();
                            grab();
                            liftDown();

                        }
                        break;
                    case LEFT:
                        goFromDepotToCraterFarMineralCase();
                        liftDown();
                        break;

                    case MIDDLE:
                        goFromDepotToCraterMiddleCase();
                        break;
                }

            }
        }catch (InterruptedException e){        }
    }


    public void killThreads(){
        during.interrupt();
        duringTwo.interrupt();
        waitUntilThreadIsNotActive(during);
        waitUntilThreadIsNotActive(duringTwo);
    }

    public void goFromDepotToCraterFarMineralCase(){
        try {
            gyroTurn(TURN_SPEED, angelForGyro(-5 * turnToCrater));
            gyroDrive(DRIVE_SPEED, -135, angelForGyro(0));
            gyroTurn(TURN_SPEED, angelForGyro(-78 * turnToCrater));
            gyroDrive(DRIVE_SPEED, 130, angelForGyro(0));
            gyroTurn(TURN_SPEED, angelForGyro(36 * turnToCrater));
            gyroDrive(DRIVE_SPEED, 250, angelForGyro(0 ));
            gyroTurn(TURN_SPEED, angelForGyro(-52 * turnToCrater));
            encoderSideWaysDrive(DRIVE_SPEED,-40 * turnToCrater);
            gyroDrive(DRIVE_SPEED,35, angelForGyro(0));

        }catch (InterruptedException e){}

    }

    public void goFromDepotToCraterForwardCloseMineralCase(){
        try {
            gyroTurn(TURN_SPEED, angelForGyro(8 * turnToCrater ));
            gyroDrive(DRIVE_SPEED, -195, angelForGyro(0));
            gyroTurn(TURN_SPEED, angelForGyro(-178 * turnToCrater ));
            encoderSideWaysDrive(DRIVE_SPEED,-60 * turnToCrater);
            gyroDrive(DRIVE_SPEED, 90, angelForGyro(0));

        }catch (InterruptedException e){}
    }


    public void goFromDepotToCraterMiddleCase(){
        try {
            gyroDrive(DRIVE_SPEED, -125, angelForGyro(0));
            gyroTurn(TURN_SPEED, angelForGyro(-90 * turnToCrater));
            gyroDrive(DRIVE_SPEED, 180, angelForGyro(3 * turnToCrater));
            gyroTurn(TURN_SPEED, angelForGyro(-32 * turnToCrater));
            encoderSideWaysDrive(DRIVE_SPEED,-94 * turnToCrater);
            gyroDrive(DRIVE_SPEED, 80, angelForGyro(0));

            killThreads();
            grab();
            liftDown();
        }catch (InterruptedException e){}
    }

    public void grabWitoutLift(){
        try {
            robot.mineralGrab.setPosition(FORWARD);

            robot.mineralSend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.mineralBoxServo.setPosition(robot.mineralBoxServoOpen);
            waitUntilThreadIsNotActive(during);

            robot.mineralGrab.setPosition(FORWARD);
            RunThread(during, ThreadActions.GRABMINERAL);

            RunThread(duringTwo, ThreadActions.PUSH);
            RunThread(during, ThreadActions.LIFTDOWNGRAB);
            waitUntilThreadIsNotActive(during);
            robot.lift.setPower(0.1);
            robot.mineralGrab.setPosition(FORWARD);

            robot.push.setPower(-1);
            waitSeconds(1.2);
            RunThread(during, ThreadActions.LIFTUP);
            pushClose(0.6);
        }catch (InterruptedException e){
        }
    }

    public void grab() {
        try {
            robot.mineralGrab.setPosition(FORWARD);
            RunThread(duringTwo, ThreadActions.EXTRUSIONS);

            robot.mineralSend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.mineralBoxServo.setPosition(robot.mineralBoxServoOpen);
            waitUntilThreadIsNotActive(during);


            robot.mineralGrab.setPosition(FORWARD);
            RunThread(during, ThreadActions.GRABMINERAL);
            RunThread(duringTwo, ThreadActions.PUSH);
            RunThread(during, ThreadActions.LIFTDOWNGRAB);

            waitUntilThreadIsNotActive(during);
            robot.lift.setPower(0.1);
            robot.mineralGrab.setPosition(FORWARD);

            robot.push.setPower(-1);
            waitSeconds(1.2);
            RunThread(during, ThreadActions.LIFTUP);
            pushClose(0.6);

            encoderMineralSend(1, 0);
            robot.blockMineralServo.setPosition(robot.dontBlock);   // Set Mode of servo to not block minerals.
            robot.mineralGrab.setPosition(FORWARD);

            waitSeconds(2);
        }catch (InterruptedException e){

        }
    }

    public void putMineralsInLander(){
        try {
            RunThread(during, ThreadActions.PUTMINERALDRIVE);
            robot.mineralGrab.setPosition(STOP);
            senderGood(1, 1500);
            robot.mineralSend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            robot.mineralBoxServo.setPosition(robot.mineralBoxServoClose);
            waitSeconds(3);
            during.interrupt();
        }catch (InterruptedException e){}
    }


    public void park(){
        try {
            if (startGoldMineralPosition==GoldPosition.MIDDLE){RunThread(during, ThreadActions.LIFTUP);}
            gyroDrive(DRIVE_SPEED,-95,angelForGyro(0));
            turnByGyro(0.6, angelForGyro(angelForGyro(-95 * parking(parking))));
            gyroDrive(DRIVE_SPEED, 180, angelForGyro(0));
            if(startGoldMineralPosition==GoldPosition.RIGHT) {
                RunThread(during, ThreadActions.LIFTDOWN);
                encoderSideWaysDrive(DRIVE_SPEED, 120);
            }
            else {
                gyroTurn(TURN_SPEED,angelForGyro(25* parking(parking)));
                RunThread(during, ThreadActions.LIFTDOWN);
                gyroDrive(DRIVE_SPEED,50,angelForGyro(0));
            }

            RunThread(during, ThreadActions.LIFTDOWN);
        }catch (InterruptedException e){

        }
    }

    public void parkForBlockingRobot(){
        try {
            encoderSideWaysDrive(DRIVE_SPEED,250);
            gyroTurn(TURN_SPEED,angelForGyro(-35));
            gyroDrive(DRIVE_SPEED, 50 ,angelForGyro(0));
            encoderSideWaysDrive(DRIVE_SPEED,-170);
        }catch (InterruptedException e){

        }
    }


    public void blockBetween(){
        try {
            waitUntilThreadIsNotActive(during);

            RunThread(during, ThreadActions.LIFTUP);
            turnByGyro(TURN_SPEED, angelForGyro(-getTheGyroAngleToTurnToTheGoldMineral(startGoldMineralPosition)));
            gyroDrive(0.5, 55, angelForGyro(0));

        }catch (InterruptedException e){

        }
    }


    public void parkForBlockingRobotIN(){
        try {
            RunThread(during,ThreadActions.LIFTUP);
            gyroDrive(DRIVE_SPEED, -20 ,angelForGyro(0));
            encoderSideWaysDrive(DRIVE_SPEED,250);
            gyroTurn(TURN_SPEED,angelForGyro(25));
            gyroDrive(DRIVE_SPEED, 60 ,angelForGyro(0));
            liftDown();

        }catch (InterruptedException e){

        }
    }

    public void liftDown() throws InterruptedException{
        try
        {
            liftUntilStuckBIT(1);
            encoderLift(1, (robot.lift.getCurrentPosition() - 125));
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
            liftUntilStuckBIT(-1);
            robot.mineralGrab.setPosition(BACKWARDS);
            waitSeconds(3);
            robot.mineralGrab.setPosition(STOP);

        }catch (InterruptedException e){

        }

    }

    public void test(){
        try {
            robot.mineralGrab.setPosition(FORWARD);
            RunThread(duringTwo, ThreadActions.PUSH);
            RunThread(during, ThreadActions.GRABMINERAL);
            waitSeconds(2.5);

            robot.push.setPower(-1);
            waitSeconds(1);

            pushClose(0.6);
        }catch (InterruptedException e){

        }
    }

    public void betweenMineralGrab(){
        try {
            RunThread(during, ThreadActions.DRIVEBETWEENMINERALGRAB);
            encoderMineralSend(1, 0);
        }catch (InterruptedException e){

        }
    }


    public void getReadyToGrabMinerals(){
        try {
            RunThread(during, ThreadActions.LIFTUP);
            turnByGyro(TURN_SPEED, angelForGyro(-getTheGyroAngleToTurnToTheGoldMineral(startGoldMineralPosition)));
            if(startGoldMineralPosition==GoldPosition.RIGHT){
                encoderSideWaysDrive(DRIVE_SPEED,10);
            }
            gyroDrive(DRIVE_SPEED,90,angelForGyro(0));
            gyroDrive(0.6,20,angelForGyro(0));
            waitUntilThreadIsNotActive(during);
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

    public int getTheGyroAngleToTurnToTheGoldMineralWithoutMove(GoldPosition goldPosition){
        switch (goldPosition){
            case LEFT:
                return  craterGyroAngleForGoldMineralLeft;
            case MIDDLE:
                return craterGyroAngleForGoldMineralMiddle;
            case RIGHT:
                return craterGyroAngleForGoldMineralRight;
        }
        return 0;
    }


    // Function return gold mineral Location after image processing.
    public GoldPosition ImageProcessing(){
        GoldPosition position = null;
        try {
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

                            if (goldYPosition > 310) {  // If gold mineral position is on the left part of the camera.
                                return GoldPosition.RIGHT;
                            } else if (goldYPosition < 310) {   // If gold mineral position is on the right part of the camera.
                                return GoldPosition.LEFT;
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


    // Function turns on the camera and enables processing.
    public void InitMyVision(){
        vision = new MineralVision();
        // Start to display image of camera.
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0);
        vision.setShowCountours(false);
        // start the vision system.
        vision.enable();
    }

    public void endAuto(){
        killThreads();
        telemetry.addData("finished", "done");
        telemetry.update();
    }



    /** Threads **/

    private class during extends Thread {
        public during() {
            this.setName("during");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {
            if (opModeIsActive()) {
                switch (threadOneCurrentAction) {

                    case VISIONCLIMB:
                        while (opModeIsActive()) {
                            startGoldMineralPosition = ImageProcessing();
                        }
                        break;

                    case LIFTDOWN:
                        try {
                            waitSeconds(1.8);
                            liftDown();
                            break;
                        } catch (InterruptedException e) {
                            robot.lift.setPower(0);
                        }
                        break;

                    case LIFTUP:
                        try {
                            liftUntilStuck(-1);
                            break;
                        } catch (InterruptedException e) {
                            robot.lift.setPower(0);
                        }
                        break;

                    case DRIVEFORWARD:
                        try {
                            gyroDrive(0.8, 90, angelForGyro(0));
                            break;
                        } catch (InterruptedException e) {
                            robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                        }

                    case DRIVEFORWARDMARKER:
                        try {
                            gyroDrive(0.8, 80, angelForGyro(0));
                            break;
                        } catch (InterruptedException e) {
                            robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                        }


                    case GRABMINERAL:
                        try {
                            liftUntilStuck(1);
                            robot.blockMineralServo.setPosition(robot.block);   // Set Mode of servo to not block minerals.
                            robot.mineralGrab.setPosition(FORWARD);
                            break;
                        } catch (InterruptedException e) {
                            robot.lift.setPower(0);
                            robot.mineralGrab.setPosition(STOP);
                        }
                        break;

                    case PUTMINERALDRIVE:
                        try {
                            waitSeconds(0.2);
                            gyroDrive(0.6, -18, angelForGyro(0));
                            break;

                        } catch (InterruptedException e) {
                            robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                        }
                        break;

                    case DRIVETOCRATER:
                        try {
                            gyroDrive(DRIVE_SPEED, 80, angelForGyro(0));
                            break;

                        } catch (InterruptedException e) {
                            robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                        }
                        break;

                    case DRIVEBACKWARD:
                        try {
                            gyroDrive(DRIVE_SPEED, -80, angelForGyro(0));
                            break;

                        } catch (InterruptedException e) {
                            robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                        }

                    case DRIVEBETWEENMINERALGRAB:
                        try {
                            gyroDrive(0.6, 15, angelForGyro(0));
                            break;

                        } catch (InterruptedException e) {
                            robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                        }
                        break;

                    case MYDEPOTDRIVEFORWARD:
                        try {
                            gyroDrive(DRIVE_SPEED, 80, angelForGyro(0));
                            break;

                        } catch (InterruptedException e) {
                            robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                        }

                    case MYDEPOTDRIVEBACKWARD:
                        try {
                            gyroDrive(DRIVE_SPEED, 80, angelForGyro(0));
                            break;
                        } catch (InterruptedException e) {
                            robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                        }
                        break;

                    case LIFTDOWNGRAB:
                        try {
                            liftUntilStuck(1);
                            break;
                        } catch (InterruptedException e) {
                            robot.lift.setPower(0);
                        }
                        break;
                }

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
                switch (threadTwoCurrentAction) {
                    case LIFTSTART:
                        try {
                            encoderLift(1, 40);
                            liftUntilStuckBIT(-1);
                            break;

                        } catch (InterruptedException e) {
                            robot.lift.setPower(0);
                        }
                        break;

                    case LIFTUP:

                        try {
                            liftUntilStuckBIT(-1);
                            break;
                        } catch (InterruptedException e) {
                            robot.lift.setPower(0);
                        }
                        break;

                    case EXTRUSIONS:
                        try {
                            encoderMineralSend(1, 130);
                            break;

                        } catch (InterruptedException e) {
                            robot.mineralSend.setPower(0);
                        }
                        break;

                    case LIFTDOWN:
                        try {
                            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            liftDown();
                            break;

                        } catch (InterruptedException e) {
                            robot.lift.setPower(0);
                        }

                    case PUSH:
                        try {
                            robot.push.setPower(-1);
                            waitSeconds(0.2);
                            robot.push.setPower(0.6);
                            waitSeconds(0.1);
                            robot.push.setPower(0);
                            break;

                        } catch (InterruptedException e) {
                            robot.push.setPower(0);
                        }
                        break;

                    case LIFTDOWNGRAB:
                        try {
                            liftUntilStuck(1);
                            break;
                        } catch (InterruptedException e) {
                            robot.lift.setPower(0);
                        }
                        break;
                }
            }
        }
    }

    public void waitUntilThreadIsNotActive (Thread currentThread)
    {
        while (currentThread.isAlive())
        {
            sleep(50);
        }
    }

}


