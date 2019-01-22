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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.
    static final double SIDE_WAYS_DRIVE_SPEED = 1;     // Nominal speed for better accuracy.


    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    static final double grabMineralsAmountSecondes= 2;


    static final int liftOpen = 300;

    static final int liftClose = 60;


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
    GoldPosition Vision= null;

    //Init function, hardwareMap
    public void apolloInit() {
        //Hardware init
        robot.init(hardwareMap);
        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mineralSend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.imuRestart();
        gyroDegrees=0;
        InitMyVision();



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Apollo", "Ready");
        telemetry.addData("Version", robot.Version);
        telemetry.update();
    }

    //The main function of the autonomous
    void apolloRun(boolean isCrater)
    {
        encoderClimbVision(1,13000);
        encoderClimb(1,27300);


        getStartGoldPositins=GoldPosition.MIDDLE;

        encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, -30);
        turnByGyro(0.6, angelForGyro(90));

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
                    gyroDrive(1,100, angelForGyro(0));

                    encoderLift(1,liftOpen);
                    robot.setDriveMotorsPower(0.1, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                    //waitSeconds(1);
                    grabMinerals(0.8);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);

                    robot.blockMineralServo.setPosition(robot.dontBlock);
                    robot.mineralBoxServo.setPosition(0.3);
                    encoderLift(1 , liftClose);
                    //waitSeconds(1);

                    gyroDrive(1,-20,angelForGyro(0));
                    gyroTurn(TURN_SPEED, angelForGyro(-45));
                    robot.blockMineralServo.setPosition(robot.block);
                    encoderMineralSend(-1, 7850);
                    robot.mineralSend.setPower(-0.01);
                    gyroTurn(0.2, angelForGyro(33));
                    //waitSeconds(1);

                    robot.mineralBoxServo.setPosition(1);
                    robot.mineralSend.setPower(0);
                    gyroDrive(1,20,angelForGyro(0));
                    encoderLift(1,liftOpen);


                    break;
            }



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

                    robot.setMineralGrabServos(5);


                    driveByGyro(0.5,40, angelForGyro(0));
                    robot.setMineralGrabServos(0);
                    encoderLift(0.5 , liftClose);
                    //robot.lift.setPower(0.01);

                    driveByGyro(1,70, angelForGyro(0));
                    robot.setMineralGrabServos(0);

                    break;

            }


        }
        //tfod.shutdown();

    }

    public GoldPosition visionActivate(){
        try {
            GetGoldLocation();
            Vision=getLocation();
            telemetry.addData("Gold Mineral Position", getLocation());
            telemetry.update();
        }catch (Exception e){
            telemetry.addData("ERROR","HELL");
            telemetry.update();
        }
        return Vision;
    }

    // Function will be used to keep track of the gyro positions.
    public int angelForGyro(int degreesWanted){
        gyroDegrees=(int)(GetGyroAngle())+degreesWanted;
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
    public GoldPosition getLocation(/* GoldPosition vision */){
        switch ((GetGoldLocation())){
            case LEFT:
                return GoldPosition.MIDDLE;
            case RIGHT:
                return GoldPosition.RIGHT;
            case MIDDLE:
                return GoldPosition.MIDDLE;
            case OUTOFRANGE:
                return GoldPosition.LEFT;
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

                            if (goldYPosition < 220) {
                                telemetry.addData("Gold Position", "Left");
                                return GoldPosition.LEFT;
                            } else if (goldYPosition > 220) {
                                telemetry.addData("Gold Position", "Right");
                                return GoldPosition.RIGHT;
                            }
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
                            double angle){

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
                telemetry.addData("left speed",leftSpeed);
                telemetry.addData("right speed",rightSpeed);
                telemetry.update();

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
    public void gyroTurn (  double speed, double angle) {

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

    public void encoderLift(double speed, int ticks) {

        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.lift.setTargetPosition(ticks);
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
        while (robot.climbMotor.getCurrentPosition() < 23100) {
            robot.climbMotor.setPower(1);

            visionActivate();
            telemetry.addData("Gold Mineral Position is", visionActivate());

            //getStartGoldPositins = getLocation(/*GetGoldLocation()*/);
            //telemetry.addData("Gold Mineral Position is", getStartGoldPositins);
            telemetry.update();
        }
        robot.climbMotor.setPower(0);
    }



    public void encoderClimbVision(double speed, int ticks) {

        robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.climbMotor.setTargetPosition(ticks);
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
                visionActivate();
                getStartGoldPositins=visionActivate();
                //telemetry.addData("Gold Mineral Position is", visionActivate());
            }

            // Stop all motion;
            robot.climbMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderClimb(double speed, int ticks) {

        robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.climbMotor.setTargetPosition(ticks);
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
                //telemetry.addData("Gold Mineral Position is", visionActivate());
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

                        telemetry.addData("sender ", robot.mineralSend.getCurrentPosition());
                        //telemetry.addData("sender right", robot.mineralSendRight.getCurrentPosition());
                        telemetry.update();
                    }
                } else if (mineralSenderPosition <= Position) {
                    while (opModeIsActive() && mineralSenderPosition <= Position) {
                        robot.mineralSend.setPower(-speed);
                        mineralSenderPosition = robot.mineralSend.getCurrentPosition();

                        telemetry.addData("sender ", robot.mineralSend.getCurrentPosition());
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


    public void InitMyVision(){
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system
        vision.enable();
    }
    public float GetGyroAngle(){
        Orientation angles =robot.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        return(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
    }


}


