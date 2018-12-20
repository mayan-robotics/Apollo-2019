package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.MatOfPoint;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


/**
 * Apollo Autonomus.
 */

public abstract class AutoMain extends LinearOpMode
{
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private ElapsedTime runtime = new ElapsedTime();

    //Vuforia parameters
    private static final String VUFORIA_KEY = "ARCYecv/////AAABmcqxLpTXUUOPuxsa+4HIQ/GIzDMvaqWwbHZGO/Ai1kF7+COWChW41B25PqOkg6T0pwD5mJxJjStWJnIzFCHi0JyRYYqH+tscLebqWRxN7Me7udkEyQIwGw5VKxc4+gvttO/m04DvUGXEC7NjJNFtGZbbAGFBfD1UQY2vdDX1d14bIlRsHFiL9cD56NT4D0D+MACRGNnYUGs2DszENbJhIXy8uhUWeAHr3qERtEnGB0E/QoNVOxsa0G4LXl21NQhtmgYBvya9+2aC6BOjcwkEwu3XKTdYdfklbB8KNLB2+Wk6KYhTyET1YQg1+3E9asYLpnkkrZ836Y6WK7akFYds37io1yMZPRWG036tVQHfzxtD";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.
    static final double SIDE_WAYS_DRIVE_SPEED = 0.3;     // Nominal speed for better accuracy.


    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    static final int grabMineralsAmountSecondes= 5;

    static final double mineralLiftositionIn = 0 ;
    static final double mineralLiftPositionOut = 0;
    static final int mineralSendPositionIn = 0 ;
    static final int mineralSendPositionOut = -5000;

    static final double senderOpenLimitPoint = -4500 ; // Limit so the sender motors wont open to much, by encoder ticks.

    int mineralSenderWantedPosition  = 0 ; // Encoder positions


    int distenceForGoldMineral  = 80 ; // Encoder positions


    //int MineralLimitY = 550;


    static final double mineralGraberPower = 0.7;
    private MineralVision phoneVision;
    private List<MatOfPoint> contoursGold = new ArrayList<>();

    // Declaration of gold positions.
    private enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE,
        OUTOFRANGE
    }

    private enum SuccessORFail {
        SUCCESS,
        FAIL
    }

    GoldPosition getStartGoldPositins= null;

    //Init function, hardwareMap
    public void apolloInit() {
        //Hardware init
        robot.init(hardwareMap);
        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setMineralSendMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Init Vuforia
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("ERROR!", "This device is not compatible with TFOD");
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Apollo", "Ready");
        telemetry.addData("Version", robot.Version);
        telemetry.update();
    }

    //The main function of the autonomous
    void apolloRun(boolean isCrater)
    {
        // Get the main position of gold mineral at start.
        //waitSeconds(2);
        telemetry.addData("here", "2");
        getStartGoldPositins=getLocation();
        telemetry.addData("here", "1");
        waitSeconds(2);
        getStartGoldPositins=getLocation();
        telemetry.addData("Gold Mineral Position is", getStartGoldPositins);
        telemetry.update();
        waitSeconds(2);

        driveByGyro(0.3,40,0);
        waitSeconds(1);


       switch (getStartGoldPositins){
            case LEFT:
             encoderSideWaysDrive(0.8,distenceForGoldMineral);

                break;
            case RIGHT:
               encoderSideWaysDrive(0.8,-distenceForGoldMineral);

                break;
            case MIDDLE:

                break;
        }

        gyroTurn(0.6,-43);

        driveByGyro(1,-90,47);
        //encoderMineralSend(0.2, mineralSendPositionOut);

        tfod.shutdown();

        //encoderMineralSend(0.2, mineralSendPositionOut);


        //switch (getStartGoldPositins){
        //    case LEFT:
        //
        //        break;
        //    case RIGHT:
        //
        //        break;
        //}

        //vision.setShowCountours(true);
        if(isCrater)
        {
            //phoneVisionOn();
        }
        else if (!isCrater)
        {
            if(getStartGoldPositins==GoldPosition.MIDDLE)
            {
            }

        }

    }

    // Function activates the mineral grab motor to grab minerals.
    public void grabMinerals(){
        robot.mineralGrab.setPower(mineralGraberPower);
        waitSeconds(grabMineralsAmountSecondes);
        robot.mineralGrab.setPower(0);
    }


    // Function to wait an amount of seconds.
    public void waitSeconds(double seconds)
    {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
        }
    }

    // Function drives until the gold mineral is in the middle by our camera.
    public SuccessORFail moveToGoldMineralByVuforia( GoldPosition useGoldPositions){
        if(getVuforiaGoldMineralPosition()!= null) {
            telemetry.addData("Vision gold position", getVuforiaGoldMineralPosition());
            if (useGoldPositions == GoldPosition.LEFT) {
                telemetry.addData("Drive", "left");
                robot.setDriveMotorsPower(-SIDE_WAYS_DRIVE_SPEED, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            }
            else if (useGoldPositions == GoldPosition.RIGHT) {
                telemetry.addData("Drive", "right");
                robot.setDriveMotorsPower(SIDE_WAYS_DRIVE_SPEED, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            }
            else if (useGoldPositions == GoldPosition.MIDDLE)
            {
                telemetry.addData("Drive", "Middle");
                robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                return SuccessORFail.SUCCESS;
            }
            else {
                robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                telemetry.addData("camera", "Error");
                return SuccessORFail.FAIL;
            }
        }
        return null;
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
                            int goldMineralY;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    goldMineralY = (int) recognition.getBottom();
                                    telemetry.addData("GoldX", goldMineralX);
                                    telemetry.addData("GoldY", goldMineralY);
                                    telemetry.addData("# Object Detected2", updatedRecognitions.size());
                                    telemetry.update();
                                    //if(goldMineralY<robot.MineralLimitY) {
                                    if (goldMineralX < robot.MineralMiddleLimitLeft) {
                                        return GoldPosition.LEFT;
                                    } else if (goldMineralX > robot.MineralMiddleLimitRight) {
                                        return GoldPosition.RIGHT;
                                    } else if (goldMineralX > robot.MineralMiddleLimitLeft && goldMineralX < robot.MineralMiddleLimitRight) {
                                        return GoldPosition.MIDDLE;
                                    } else {
                                        return GoldPosition.OUTOFRANGE;
                                    }
                                    //}
                                } else {
                                    return GoldPosition.OUTOFRANGE;
                                }
                            }
                        } else {
                            return GoldPosition.OUTOFRANGE;
                        }
                    }else {
                            return GoldPosition.OUTOFRANGE;
                        }
                    }
                }
            }

        return null;
    }

    // Function divides the minerals to gold and silver with servo by camera.
    public void MineralDivideByVision(){
        if(phoneVision.goldMineralFound()== true){
            robot.mineralsDivider.setPosition(robot.dividerRight);
        }else{
            robot.mineralsDivider.setPosition(robot.dividerLeft);
        }
    }


    /** Drive By Gyro Functions **/
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void gyroDrive ( double speed,
                            double Distance,
                            double angle)
    {

        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (Distance);

            // Set Target and Turn On RUN_TO_POSITION
            robot.setDriveMotorsPosition(moveCounts, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
            robot.setDriveMotorsPosition(moveCounts, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);

            robot.setDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            robot.setDriveMotorsPower(speed, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.driveLeftFront.isBusy()
                            && robot.driveLeftBack.isBusy()
                            && robot.driveRightFront.isBusy()
                            && robot.driveRightBack.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (Distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.setDriveMotorsPower(leftSpeed, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                robot.setDriveMotorsPower(rightSpeed, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);

            }

            // Stop all motion;
            robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);

            // Turn off RUN_TO_POSITION
            robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle)
    {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold ( double speed, double angle, double holdTime)
    {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);

    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading ( double speed, double angle, double PCoeff)
    {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setDriveMotorsPower(leftSpeed, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
        robot.setDriveMotorsPower(rightSpeed, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError ( double targetAngle)
    {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer ( double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }


    public void turnByGyro(double speed, double angle)
    {
        for (int i=0; i<3;i++) {
            gyroTurn(speed,angle);
        }
    }

    public void driveByGyro(double speed, double Distance, double angle)
    {
        gyroDrive(speed, Distance, angle);
        turnByGyro(TURN_SPEED,angle);
    }

    /** Activate Motor By Encoder Functions **/

    // Drive side ways by encoder function.
    public void encoderSideWaysDrive(double speed,
                                     double Distance)
    {
        // Ensure that the opmode is still active
        if (opModeIsActive())
        {
            robot.setDriveMotorsPosition(Distance, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            // Turn On RUN_TO_POSITION
            robot.setDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Set Speed
            robot.setDriveMotorsPower(Math.abs(speed), HardwareApollo.DRIVE_MOTOR_TYPES.ALL);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.driveLeftFront.isBusy()
                            && robot.driveLeftBack.isBusy()
                            && robot.driveRightFront.isBusy()
                            && robot.driveRightBack.isBusy())) {
            }
            // Stop all motion;
            robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
            // Turn off RUN_TO_POSITION
            robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderLift(double speed, double Distance) {

        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.setLiftMotorsPosition(Distance);
            // Turn On RUN_TO_POSITION
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.lift.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.lift.isBusy())){
            }

            // Stop all motion;
            robot.lift.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void encoderMineralSend(double speed, int Position) {
        //robot.mineralSendRight.setTargetPosition(mineralSenderWantedPosition);
        //robot.mineralSendLeft.setTargetPosition(mineralSenderWantedPosition);
        //robot.setMineralSendMode(DcMotor.RunMode.RUN_TO_POSITION);
        int mineralSenderPosition;


        mineralSenderPosition = robot.mineralSendRight.getCurrentPosition();

        //robot.setMineralSendMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //if(-gamepad2.right_stick_y > 0 && mineralSenderPosition >= senderOpenLimitPoint) {
        //    robot.setMineralSendPower(-gamepad2.right_stick_y);
        //    mineralSenderWantedPosition = robot.mineralSendRight.getCurrentPosition();
        //    telemetry.addData("tese", "here");
        //}
        // Ensure that the opmode is still active
        for (int i=0; i<2; i++) {
            if (opModeIsActive()) {
                mineralSenderPosition = robot.mineralSendRight.getCurrentPosition();


                //robot.mineralSendRight.setTargetPosition(Position);
                //robot.mineralSendLeft.setTargetPosition(Position);


                // Turn On RUN_TO_POSITION
                //robot.setMineralSendMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                //robot.setMineralSendPower(speed);
                //robot.setMineralSendMode(DcMotor.RunMode.RUN_TO_POSITION);
                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.

                if (mineralSenderPosition >= Position) {
                    while (opModeIsActive() && mineralSenderPosition >= Position) {
                        robot.setMineralSendPower(speed);
                        mineralSenderPosition = robot.mineralSendRight.getCurrentPosition();

                        telemetry.addData("sender left", robot.mineralSendLeft.getCurrentPosition());
                        telemetry.addData("sender right", robot.mineralSendRight.getCurrentPosition());
                        telemetry.update();
                    }
                } else if (mineralSenderPosition <= Position) {
                    while (opModeIsActive() && mineralSenderPosition <= Position) {
                        robot.setMineralSendPower(-speed);
                        mineralSenderPosition = robot.mineralSendRight.getCurrentPosition();

                        telemetry.addData("sender left", robot.mineralSendLeft.getCurrentPosition());
                        telemetry.addData("sender right", robot.mineralSendRight.getCurrentPosition());
                        telemetry.update();
                    }
                }

                // Stop all motion;
                robot.setMineralSendPower(0);

                // Turn off RUN_TO_POSITION
                robot.setMineralSendMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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

    /** PhoneVision **/
    public void phoneVisionOn(){
        phoneVision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        phoneVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        phoneVision.enable();
        phoneVision.setShowCountours(true);
    }
}


