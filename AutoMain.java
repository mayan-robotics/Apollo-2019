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
    static final double SIDE_WAYS_DRIVE_SPEED = 0.8;     // Nominal speed for better accuracy.


    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.05;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.05;     // Larger is more responsive, but also less stable

    static final int grabMineralsAmountSecondes= 3;


    static final int liftOpen = 1900;

    static final int liftClose = -2200;


    static final double senderOpenLimitPoint = -4500 ; // Limit so the sender motors wont open to much, by encoder ticks.

    int mineralSenderWantedPosition  = 0 ; // Encoder positions


    int distenceForGoldMineral  = 80 ; // Encoder positions
    int gyroDegrees = 0;


    //int MineralLimitY = 550;


    static final double mineralGraberPower = 1;
    private MineralVision vision;
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
        robot.mineralSend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        climb();

        waitSeconds(999);
        // Get the main position of gold mineral at start.
        getStartGoldPositins=getLocation(GetGoldLocation());
        waitSeconds(2);
        //while (opModeIsActive()) {
            //getVuforiaGoldMineralPosition();

            getStartGoldPositins = getLocation(GetGoldLocation());
            telemetry.addData("Gold Mineral Position is", getStartGoldPositins);
            telemetry.update();

        waitSeconds(1);

        encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, -30);
        //waitSeconds(5);

        gyroTurn(TURN_SPEED,angelForGyro(90));
        waitSeconds(0.2);
        gyroTurn(TURN_SPEED,angelForGyro(0));

        //robot.imuRestart();
        //waitSeconds(1);
        //robot.init(hardwareMap);
        //robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.mineralSend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //waitSeconds(1);

        //gyroDegrees = 0;

        driveByGyro(0.3,-5 ,angelForGyro(0));

       switch (getStartGoldPositins){
            case LEFT:
                driveByGyro(0.3,40,angelForGyro(0));
                encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, -88);
                //driveByGyro(0.3,-20,angelForGyro(0));
                break;
            case RIGHT:
                driveByGyro(0.3,40,angelForGyro(0));
                encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, distenceForGoldMineral);
                //driveByGyro(0.3,-20,angelForGyro(0));
               break;
        }


        waitSeconds(1);



        /** If Crater **/
        if(isCrater)
        {

            switch (getStartGoldPositins){
                case LEFT:

                    driveByGyro(0.3,50, angelForGyro(0));

                    waitSeconds(1);
                    robot.init(hardwareMap);
                    robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.mineralSend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    waitSeconds(1);
                    gyroDegrees = 0;

                    encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, -35);

                    driveByGyro(DRIVE_SPEED,40,angelForGyro(0));

                    break;
                case RIGHT:

                    driveByGyro(0.3,30, angelForGyro(0));

                    waitSeconds(1);
                    robot.init(hardwareMap);
                    robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.mineralSend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    waitSeconds(1);
                    gyroDegrees = 0;

                    encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, 30);
                    driveByGyro(DRIVE_SPEED,30,angelForGyro(0));

                    //driveByGyro(DRIVE_SPEED,215,angelForGyro(0));

                    break;
                case MIDDLE:
                    encoderLift(0.5,liftOpen);
                    //waitSeconds(2);
                    driveByGyro(0.5,20, angelForGyro(0));
                    grabMinerals(-1);
                    waitSeconds(2);
                    encoderLift(0.5 , liftClose);

                    robot.lift.setPower(0.01);


                    waitSeconds(1);
                    driveByGyro(1,80, angelForGyro(0));

                    //driveByGyro(-DRIVE_SPEED,-30, angelForGyro(0));
                    //gyroTurn(TURN_SPEED,angelForGyro(90));
                    //driveByGyro(DRIVE_SPEED,135,angelForGyro(0));

                    break;
            }
            encoderLift(0.5,liftOpen);
            grabMinerals(-1);
            waitSeconds(2);
            //gyroTurn(TURN_SPEED,angelForGyro(-133));
            //driveByGyro(1,-80,angelForGyro(0));

            //encoderMineralSend(-1, mineralSendPositionOut);
            //waitSeconds(3);
            //encoderMineralSend(1, mineralSendPositionIn);


        }
        /** If Depot **/
        else if (!isCrater)
        {
            //driveByGyro(1,55, angelForGyro(0));
            //waitSeconds(5);

            switch (getStartGoldPositins){
                case LEFT:
                    encoderLift(0.5,liftOpen);
                    driveByGyro(0.5,20, angelForGyro(0));

                    robot.setMineralGrabServos(-1);


                    driveByGyro(0.5,40, angelForGyro(0));
                    robot.setMineralGrabServos(0);


                    driveByGyro(1,55, angelForGyro(0));

                    gyroTurn(TURN_SPEED,angelForGyro(-20));

                    robot.setMineralGrabServos(1);
                    waitSeconds(2);
                    robot.setMineralGrabServos(0);


                    break;
                case RIGHT:
                    //encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED,distenceForGoldMineral);
                    driveByGyro(0.5,-45, angelForGyro(0));

                    encoderLift(0.5,liftOpen);

                    robot.setMineralGrabServos(-1);


                    driveByGyro(0.5,40, angelForGyro(0));
                    robot.setMineralGrabServos(0);


                    driveByGyro(1,60, angelForGyro(0));

                    gyroTurn(TURN_SPEED,angelForGyro(20));

                    robot.setMineralGrabServos(-1);
                    waitSeconds(2);
                    robot.setMineralGrabServos(0);

                    break;
                case MIDDLE:
                    encoderLift(0.5,liftOpen);
                    //waitSeconds(2);
                    driveByGyro(0.5,20, angelForGyro(0));
                    //grabMinerals(-1);
                    //waitSeconds(2);
                    //encoderLift(0.5 , liftClose);

                    robot.setMineralGrabServos(1);


                    driveByGyro(0.5,40, angelForGyro(0));
                    robot.setMineralGrabServos(0);
                    encoderLift(0.5 , liftClose);
                    //robot.lift.setPower(0.01);

                    driveByGyro(1,70, angelForGyro(0));
                    robot.setMineralGrabServos(0);

                    break;

            }


        }
        tfod.shutdown();

    }

    // Function will be used to keep track of the gyro positions.
    public int angelForGyro(int degreesWanted){
        gyroDegrees=gyroDegrees+degreesWanted;
        return gyroDegrees;
    }

    // Function activates the mineral grab motor to grab minerals.
    public void grabMinerals(double speed){
        robot.setMineralGrabServos(speed);
        waitSeconds(grabMineralsAmountSecondes);
        robot.setMineralGrabServos(0);
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
        if(GetGoldLocation()!= null) {
            telemetry.addData("Vision gold position", GetGoldLocation());
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
    public GoldPosition getLocation(GoldPosition vision){
        switch ((vision)){
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

    //Function returns the location of the gold mineral.
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
                            //int goldXPosition = GoldBoundingRect.x;
                            int goldYPosition = GoldBoundingRect.y;

                            //telemetry.addData("Gold Position X", goldXPosition);
                            telemetry.addData("Gold Position Y", goldYPosition);

                            if (goldYPosition < 300) {
                                telemetry.addData("Gold Position", "Left");
                                return GoldPosition.LEFT;
                            } else if (goldYPosition > 300) {
                                telemetry.addData("Gold Position", "Right");
                                return GoldPosition.RIGHT;
                            } //else if (goldYPosition > 450 && goldYPosition < 650) {
                            //  telemetry.addData("Gold Position", "Middle");
                            //  return GoldPosition.MIDDLE;
                            //}
                        }
                    }
                }
            } else {
                telemetry.addData("Apollo", "did not find a gold mineral");
                return GoldPosition.OUTOFRANGE;
            }
        }catch (Exception e){
            telemetry.addData("Vision", "Error");
            telemetry.update();
            return GoldPosition.OUTOFRANGE;
        }

        return null;}


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
        robot.setDriveMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void climb(){
        while (robot.climbMotor.getCurrentPosition() < 53700) {
            robot.climbMotor.setPower(1);

            getStartGoldPositins = getLocation(GetGoldLocation());
            telemetry.addData("Gold Mineral Position is", getStartGoldPositins);
            telemetry.update();
        }
        robot.climbMotor.setPower(0);
    }

    public void encoderClimb(double speed, double Distance) {

        robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.setLiftMotorsPosition(Distance);
            // Turn On RUN_TO_POSITION
            robot.climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.climbMotor.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.climbMotor.isBusy())){
            }

            // Stop all motion;
            robot.climbMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void encoderMineralSend(double speed, int Position) {

        int mineralSenderPosition;

        mineralSenderPosition = robot.mineralSend.getCurrentPosition();

        // Ensure that the opmode is still active
        for (int i=0; i<2; i++) {
            if (opModeIsActive()) {
                mineralSenderPosition = robot.mineralSend.getCurrentPosition();

                // reset the timeout time and start motion.
                runtime.reset();

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.

                if (mineralSenderPosition >= Position) {
                    while (opModeIsActive() && mineralSenderPosition >= Position) {
                        robot.mineralSend.setPower(speed);
                        mineralSenderPosition = robot.mineralSend.getCurrentPosition();

                        telemetry.addData("sender left", robot.mineralSend.getCurrentPosition());
                        //telemetry.addData("sender right", robot.mineralSendRight.getCurrentPosition());
                        telemetry.update();
                    }
                } else if (mineralSenderPosition <= Position) {
                    while (opModeIsActive() && mineralSenderPosition <= Position) {
                        robot.mineralSend.setPower(-speed);
                        mineralSenderPosition = robot.mineralSend.getCurrentPosition();

                        telemetry.addData("sender left", robot.mineralSend.getCurrentPosition());
                        //telemetry.addData("sender right", robot.mineralSendRight.getCurrentPosition());
                        telemetry.update();
                    }
                }

                // Stop all motion;
                robot.mineralSend.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.mineralSend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void initMyVision(){
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system
        vision.enable();
        runtime.reset();
    }
}


