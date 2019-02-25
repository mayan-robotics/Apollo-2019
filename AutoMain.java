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
 * Apollo Autonomus.
 */

public abstract class AutoMain extends LinearOpMode
{
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
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

    //Init function, hardwareMap
    public void apolloInit() {
        //Hardware init
        robot.init(hardwareMap);
        robot.InitServoes();
        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.imuRestart();
        gyroDegrees=0;  // Reset gyro angle.

        // Init camera and turn it on.
        InitMyVision();

        // Send telemetry message to signify robot is ready;
        telemetry.addData("Version", robot.Version);      // Program date.
        telemetry.addData("Apollo", "Init success");    // Telemetry message to signify the robot is ready.
        telemetry.update();
    }

    //The main function of the autonomous
    void apolloRun(boolean isCrater)
    {
        encoderClimbVision(1, robot.climbOpenPosition);     // Get down from lender, and at the same time open systems and process image.
        robot.goldMineralRightServo.setPosition(0);
        turnAwayFromLender();     // Turn towards the minerals.

        // According to the image processing we did.
        switch (getStartGoldMineralPosition){
            case LEFT:      // If gold mineral is on the left.
                robot.goldMineralRightServo.setPosition(0.3);   // Close right servo.
                break;
            case RIGHT:     // If gold mineral is on the right.
                robot.goldMineralLeftServo.setPosition(goldMineralServoCloseLeft);  // Close left servo.
                break;
            case MIDDLE:    // If gold mineral is on the middle, Close both servos.
                robot.goldMineralLeftServo.setPosition(goldMineralServoCloseLeft);
                robot.goldMineralRightServo.setPosition(0.3);
                break;
        }


        /** If Crater **/
        if(isCrater)
        {
            if(getStartGoldMineralPosition==GoldPosition.MIDDLE)
            {   // If gold mineral is in te middle.
                encoderLift(1, 675);
                robot.setMineralGrabServos(0.8);
                gyroDrive(DRIVE_SPEED,100,angelForGyro(0));
                robot.setMineralGrabServos(0);
                encoderLift(1, 550);
                gyroDrive(DRIVE_SPEED,15,angelForGyro(0));
                encoderLift(1, 675);

            }
            gyroDrive(DRIVE_SPEED,115,angelForGyro(0));
            robot.goldMineralLeftServo.setPosition(0.3);
            robot.goldMineralRightServo.setPosition(0.6);
            encoderLift(1, 675);

            robot.setMineralGrabServos(0.8);        // Try to grab minerals.
            while (opModeIsActive()){            }  // Keep trying to grab minerals until times up.



        }
        /** If Depot **/
        else if (!isCrater)
        {
            if(getStartGoldMineralPosition==GoldPosition.MIDDLE)
            {   // If gold mineral is in te middle.
                gyroDrive(DRIVE_SPEED, 140, angelForGyro(0));
                robot.setMineralGrabServos(0.2);
                encoderLift(1, 675);
                gyroDrive(DRIVE_SPEED, -45, angelForGyro(0));
                robot.setMineralGrabServos(0.8);
                gyroDrive(DRIVE_SPEED, -90, angelForGyro(0));
                robot.setMineralGrabServos(0);
                encoderLift(1, 665);
                gyroDrive(DRIVE_SPEED, 30, angelForGyro(0));
                encoderLift(1, 550);
                robot.goldMineralLeftServo.setPosition(robot.goldMineralServoCloseLeft);
                robot.goldMineralRightServo.setPosition(robot.goldMineralServoCloseRight);
            }else
            {
                gyroDrive(DRIVE_SPEED, 140, angelForGyro(0));
                encoderLift(1, 505);
                robot.setMineralGrabServos(0.2);
                robot.goldMineralRightServo.setPosition(0.4);
                robot.goldMineralLeftServo.setPosition(0.6);
                waitSeconds(0.5);
                robot.setMineralGrabServos(0);
                robot.goldMineralLeftServo.setPosition(robot.goldMineralServoCloseLeft);
                robot.goldMineralRightServo.setPosition(robot.goldMineralServoCloseRight);
                gyroDrive(DRIVE_SPEED, -105, angelForGyro(0));
            }

            turnByGyro(0.6, angelForGyro(-95 * TURNRIGHTORLEFT));
            gyroDrive(DRIVE_SPEED, 250, angelForGyro(-2  * TURNRIGHTORLEFT));
            encoderLift(1, 650);
            robot.goldMineralLeftServo.setPosition(robot.goldMineralServoCloseLeft);
            robot.goldMineralRightServo.setPosition(robot.goldMineralServoCloseRight);

            robot.setMineralGrabServos(0.8);    // Try to grab minerals.
            while (opModeIsActive()){            }  // Keep trying to grab minerals until times up.

        }

    }

    // This function  sets all the positions for the robot to be ready for running.
    public void startRobotInit(){
        robot.goldMineralLeftServo.setPosition(0.3);
        waitSeconds(0.5);
        robot.goldMineralLeftServo.setPosition(0.6);
        robot.goldMineralRightServo.setPosition(0.15);
        waitSeconds(0.5);
        robot.goldMineralRightServo.setPosition(0.4);
        waitSeconds(2);
        robot.goldMineralLeftServo.setPosition(0.92);

        encoderPush(1,1400);
        encoderLift(1,260);

    }

    // Function to open the gold mineral left servo.
    public void openLeftMineralServo(){
        robot.goldMineralLeftServo.setPosition(0.55);
        waitSeconds(0.5);
        robot.goldMineralLeftServo.setPosition(0.95);
    }

    // Function to open the right gold mineral servo.
    public void openRightMineralServo(){
        robot.goldMineralRightServo.setPosition(0.6);
        waitSeconds(0.5);
        robot.goldMineralRightServo.setPosition(0);
    }


    public void robotGrabMineral(){
        robot.mineralBoxServo.setPosition(1);
        encoderPush(1,2000);
        encoderLift(1,680);

        robot.setMineralGrabServos(0.8);

        encoderPush(1,4000);
        encoderPush(1,2000);
        robot.setMineralGrabServos(0);
        encoderLift(1,650);
        encoderPush(1,100);

        robot.mineralBoxServo.setPosition(1);
        //waitSeconds(1);
        robot.blockMineralServo.setPosition(robot.dontBlock);
        encoderLift(1,190);
        robot.mineralPush.setPosition(0.8);

        waitSeconds(2.5);
        robot.mineralPush.setPosition(0);
        robot.mineralBoxServo.setPosition(1);
        encoderMineralSend(1,7800);
        robot.mineralSend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        // Yotam helped
        robot.mineralBoxServo.setPosition(0.4);
        waitSeconds(2);
        encoderMineralSend(1,20);
    }

    // This function turns away from lender
    public void turnAwayFromLender(){
        encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, -40);
        turnByGyro(TURN_SPEED, angelForGyro(90));
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


    /** Gyro Functions **/
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

    //
    public void driveByGyro(double speed, double Distance, double angle)
    {
        gyroDrive(speed, Distance, angle);
        turnByGyro(TURN_SPEED,angle);
    }

    /** Activate Motors By Encoder Functions **/

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

    // Push motor by encoder function.
    public void encoderPush(double speed, int ticks) {
        robot.push.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.push.setTargetPosition(ticks);
            // Turn On RUN_TO_POSITION
            robot.push.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.push.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.push.isBusy())){
            }

            // Stop all motion;
            robot.push.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.push.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Lift motor by encoder function.
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

    // Send motor by encoder function.
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


    // Function gets the climbing motor to the wanted position.
    // Opens systems, and activates vision to check where the gold mineral, in the same time.
    public void encoderClimbVision(double speed, int ticks) {
        robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.climbMotor.setTargetPosition(ticks);  // Set the wanted target.
            // Turn On RUN_TO_POSITION
            robot.climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.climbMotor.setPower(Math.abs(speed));     // Set speed
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (robot.climbMotor.isBusy())){

                if (didInit==false){
                    startRobotInit();
                    didInit= true;
                }
                // Activate Vision and check where is the gold mineral. Save the position.
                getStartGoldMineralPosition = visionProcessing();
            }

            // Stop all motion;
            robot.climbMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Function turns on the camera and enables processing.
    public void InitMyVision(){
        vision = new MineralVision();
        // Start to display image of camera.
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system.
        vision.enable();
    }

    // Gyro angle
    public float GetGyroAngle(){
        Orientation angles =robot.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        return(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
    }

}


