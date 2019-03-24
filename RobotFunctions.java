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

import static java.lang.Math.abs;

/**
 * Apollo's Robot Functions.
 */

public abstract class RobotFunctions extends LinearOpMode
{
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timeRespons = new ElapsedTime();
    private ElapsedTime timeResponsExtrusions = new ElapsedTime();

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

    static final long threadSleepTimeMS = 500;

    int TURNRIGHTORLEFT = 1;    /* This number controls the directions of the robot,
                                   if its 1 -> the robot will turn right, to our crater,
                                   if its -1 -> the robot will turn left to the other crater. */

    boolean didInit = false;    // Boolean we use to know if we finished our init.
    int gyroDegrees = 0;    // Counter of gyro angle

    final static double FORWARD = 0.8 ;
    final static double BACKWARDS = 0.2 ;
    final static double STOP = 0 ;

    static final double encoderTicksRange = 5;
    static final double encoderRespondingTimeSeconds = 0.1;


    // This function turns away from lender
    public void turnAwayFromLender() throws InterruptedException{
        encoderSideWaysDrive(SIDE_WAYS_DRIVE_SPEED, -40);
        turnByGyro(TURN_SPEED, angelForGyro(90));
    }
    // Function will be used to keep track of the gyro positions.
    public int angelForGyro(int degreesWanted) throws InterruptedException{
        gyroDegrees=(int)(gyroDegrees+degreesWanted);
        return gyroDegrees;
    }


    // Function to wait an amount of seconds.
    public void waitSeconds(double seconds)throws InterruptedException
    {
        try {
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < seconds)) {
                Thread.sleep(0);
            }
        }catch (InterruptedException e) {
                throw new InterruptedException();
            }
    }

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
                            double angle) throws InterruptedException{
        try{
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
                speed = Range.clip(abs(speed), 0.0, 1.0);

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
                    max = Math.max(abs(leftSpeed), abs(rightSpeed));
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
        }catch (InterruptedException e) {
            throw new InterruptedException();
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
    public void gyroTurn (  double speed, double angle) throws InterruptedException {
        try{
            // keep looping while we are still active, and not on heading.
            while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
                // Update telemetry & Allow time for other processes to run.
                telemetry.update();

                    telemetry.update();

            }
        }catch (InterruptedException e) {
            throw new InterruptedException();
        }
    }

    public void FancyTurn (  double speed, double angle) throws InterruptedException {
        try{
            // keep looping while we are still active, and not on heading.
            angle+=GetGyroAngle();
            while (opModeIsActive() && !TheFancyFance(speed, angle, P_TURN_COEFF)) {
                // Update telemetry & Allow time for other processes to run.
                telemetry.update();

            }
        }catch (InterruptedException e) {
            throw new InterruptedException();
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
    public void gyroHold ( double speed, double angle, double holdTime) throws InterruptedException
    {
        try{

            ElapsedTime holdTimer = new ElapsedTime();

            // keep looping while we have time remaining.
            holdTimer.reset();
            while (opModeIsActive() && (holdTimer.time() < holdTime)) {
                // Update telemetry & Allow time for other processes to run.
                onHeading(speed, angle, P_TURN_COEFF);
                telemetry.update();
                Thread.sleep(threadSleepTimeMS);
            }

            // Stop all motion;
            robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
        }catch (InterruptedException e) {
            throw new InterruptedException();
        }

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
    boolean onHeading ( double speed, double angle, double PCoeff) throws InterruptedException
    {
        try {
                {
            }
            double error;
            double steer;
            boolean onTarget = false;
            double leftSpeed;
            double rightSpeed;

            // determine turn power based on +/- error
            error = getError(angle);

            if (abs(error) <= HEADING_THRESHOLD) {
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
        }catch (InterruptedException e) {
            throw new InterruptedException();
        }
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError ( double targetAngle) throws InterruptedException
    {
        try {
            double robotError;

            // calculate error in -179 to +180 range  (
            robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while (robotError > 180) {
                robotError -= 360;
                Thread.sleep(0);
            }
            while (robotError <= -180) {
                robotError += 360;
                Thread.sleep(0);
            }
            return robotError;
        }catch (InterruptedException e) {
            throw new InterruptedException();
        }
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer ( double error, double PCoeff) throws InterruptedException
    {
        return Range.clip(error * PCoeff, -1, 1);
    }


    public void turnByGyro(double speed, double angle) throws InterruptedException
    {
        for (int i=0; i<3;i++) {
            gyroTurn(speed,angle);
        }
    }

    //
    public void driveByGyro(double speed, double Distance, double angle) throws InterruptedException
    {
        gyroDrive(speed, Distance, angle);
        turnByGyro(TURN_SPEED,angle);
    }

    /** Activate Motors By Encoder Functions **/

    // Drive side ways by encoder function.
    public void encoderSideWaysDrive(double speed,
                                     double Distance) throws InterruptedException
    {

        try{
            robot.setDriveMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Ensure that the opmode is still active
            if (opModeIsActive())
            {
                robot.setDriveMotorsPosition(Distance, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                // Turn On RUN_TO_POSITION
                robot.setDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Set Speed
                robot.setDriveMotorsPower(abs(speed), HardwareApollo.DRIVE_MOTOR_TYPES.ALL);

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
                    Thread.sleep(threadSleepTimeMS);
                }
                // Stop all motion;
                robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                // Turn off RUN_TO_POSITION
                robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }catch (InterruptedException e) {
            throw new InterruptedException("drive");
        }
    }

    // Push motor by encoder function.
    public void encoderPush(double speed, int ticks) throws InterruptedException {
        try {

            timeRespons.reset();
            double pushLastPosition = robot.push.getCurrentPosition();
            double pushCurrentPosition;

            robot.push.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                robot.push.setTargetPosition(ticks);
                // Turn On RUN_TO_POSITION
                robot.push.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.push.setPower(abs(speed));
                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (robot.push.isBusy())){


                    pushCurrentPosition = robot.push.getCurrentPosition();
                    if(timeRespons.seconds()>0.5){
                        if((pushLastPosition-encoderTicksRange) < pushCurrentPosition &&
                                pushCurrentPosition < (pushLastPosition+encoderTicksRange)){
                            throw new InterruptedException("push");
                        }
                        pushLastPosition = robot.push.getCurrentPosition();
                        timeRespons.reset();
                    }


                    Thread.sleep(threadSleepTimeMS);
                }

                // Stop all motion;
                robot.push.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.push.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }catch (InterruptedException e) {
            throw new InterruptedException("push");
        }
    }

    // Lift motor by encoder function.
    public void encoderLift(double speed, int ticks) throws InterruptedException {
        try{
            timeRespons.reset();
            double liftLastPosition = robot.lift.getCurrentPosition();
            double liftCurrentPosition;

            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                robot.lift.setTargetPosition(ticks);
                // Turn On RUN_TO_POSITION
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.lift.setPower(abs(speed));
                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (robot.lift.isBusy())){

                    liftCurrentPosition = robot.lift.getCurrentPosition();
                    if(timeRespons.seconds()>0.5){
                        if((liftLastPosition-encoderTicksRange) < liftCurrentPosition &&
                                liftCurrentPosition < (liftLastPosition+encoderTicksRange)){
                            // Stop all motion;
                            robot.lift.setPower(0);
                            throw new InterruptedException("lift");
                        }
                        liftLastPosition = robot.push.getCurrentPosition();
                        timeRespons.reset();
                    }

                    Thread.sleep(threadSleepTimeMS);
                }

                // Stop all motion;
                robot.lift.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }catch (InterruptedException e) {
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            throw new InterruptedException("lift");
        }
    }

    public void liftUntilStuck (double speed) throws InterruptedException{
        try {
            timeRespons.reset();
            double liftLastPosition = robot.lift.getCurrentPosition();
            double liftCurrentPosition = 0;
            robot.lift.setPower(speed);
            boolean Exit = false;
            while (!Exit) {

                if (timeRespons.seconds() > encoderRespondingTimeSeconds) {
                    telemetry.addData("liftCurrentPosition time",liftCurrentPosition);
                    liftCurrentPosition = robot.lift.getCurrentPosition();
                    if (((liftLastPosition - encoderTicksRange) < liftCurrentPosition) &&
                         (liftCurrentPosition < (liftLastPosition + encoderTicksRange))) {
                        // Stop all motion;
                        robot.lift.setPower(0);
                        Exit = true;
                        telemetry.addData("liftCurrentPosition stop",liftCurrentPosition);
                        break;
                    }
                    liftLastPosition = robot.lift.getCurrentPosition();
                    timeRespons.reset();
                }
                Thread.sleep(100/*threadSleepTimeMS*/);
                telemetry.addData("liftCurrentPosition",liftCurrentPosition);
                telemetry.addData("liftLastPosition",liftLastPosition);
                telemetry.update();

            }
        }catch (InterruptedException e){
            robot.lift.setPower(0);
            throw new InterruptedException("lift");
        }
    }

    // Send motor by encoder function.
    public void encoderMineralSend(double speed, int ticks) throws InterruptedException {
        try{
            timeRespons.reset();
            double mineralSendLastPosition = robot.mineralSend.getCurrentPosition();
            double mineralSendCurrentPosition;

            robot.mineralSend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                robot.mineralSend.setTargetPosition(ticks);
                // Turn On RUN_TO_POSITION
                robot.mineralSend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.mineralSend.setPower(abs(speed));
                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (robot.mineralSend.isBusy())){

                    mineralSendCurrentPosition = robot.mineralSend.getCurrentPosition();
                    if(timeRespons.seconds()>0.5){
                        if((mineralSendLastPosition-encoderTicksRange) < mineralSendCurrentPosition &&
                                mineralSendCurrentPosition < (mineralSendLastPosition+encoderTicksRange)){
                            // Stop all motion;
                            robot.mineralSend.setPower(0);
                            throw new InterruptedException("mineralSend");
                        }
                        mineralSendLastPosition = robot.push.getCurrentPosition();
                        timeRespons.reset();
                    }

                    Thread.sleep(threadSleepTimeMS);
                }
//
                // Stop all motion;
                robot.mineralSend.setPower(0);
//
                // Turn off RUN_TO_POSITION
                robot.mineralSend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }catch (InterruptedException e) {
            throw new InterruptedException("mineralSend");
        }//im blackie and im know it
    }

    public boolean extrusionsStuck(){
        timeResponsExtrusions.reset();
        double mineralSendLastPosition = robot.mineralSend.getCurrentPosition();
        double mineralSendCurrentPosition;

        mineralSendCurrentPosition = robot.mineralSend.getCurrentPosition();
        if(timeResponsExtrusions.seconds()>0.5){
            mineralSendLastPosition = robot.push.getCurrentPosition();
            timeResponsExtrusions.reset();
            if((mineralSendLastPosition-encoderTicksRange) < mineralSendCurrentPosition &&
                    mineralSendCurrentPosition < (mineralSendLastPosition+encoderTicksRange)){
                // Stop all motion;
                robot.mineralSend.setPower(0);
                return true;
            }
            else {return false;}

        }
        return false;
    }


    // Function gets the climbing motor to the wanted position.
    // Opens systems, and activates vision to check where the gold mineral, in the same time.
    public void encoderClimb(double speed, int ticks) throws InterruptedException {
        try {
            timeRespons.reset();
            double climbMotorLastPosition = robot.climbMotor.getCurrentPosition();
            double climbMotorCurrentPosition;

            robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                robot.climbMotor.setTargetPosition(ticks);  // Set the wanted target.
                // Turn On RUN_TO_POSITION
                robot.climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.climbMotor.setPower(abs(speed));     // Set speed
                //keep looping while we are still active, and there is time left, and both motors are running.
                while (opModeIsActive() &&
                        (robot.climbMotor.isBusy())) {

                    climbMotorCurrentPosition = robot.climbMotor.getCurrentPosition();
                    if(timeRespons.seconds()>0.5){
                        if((climbMotorLastPosition-encoderTicksRange) < climbMotorCurrentPosition &&
                                climbMotorCurrentPosition < (climbMotorLastPosition+encoderTicksRange)){
                            // Stop all motion;
                            robot.climbMotor.setPower(0);
                            throw new InterruptedException("climbMotor");
                        }
                        climbMotorLastPosition = robot.push.getCurrentPosition();
                        timeRespons.reset();
                    }

                    Thread.sleep(threadSleepTimeMS);
                }

                 //Stop all motion;
                robot.climbMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        } catch (InterruptedException e) {
            throw new InterruptedException("climbMotor");
        }

    }


    // Function gets the climbing motor to the wanted position.
    // Opens systems, and activates vision to check where the gold mineral, in the same time.
    public void encoderClimbVision(double speed, int ticks) throws InterruptedException  {
        try{
            robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                robot.climbMotor.setTargetPosition(ticks);  // Set the wanted target.
                // Turn On RUN_TO_POSITION
                robot.climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.climbMotor.setPower(abs(speed));     // Set speed
                // keep looping while we are still active, and there is time left, and both motors are running.
                while (opModeIsActive() &&
                        (robot.climbMotor.isBusy())){
                    Thread.sleep(threadSleepTimeMS);
                    if (didInit==false){
                        //startRobotInit();
                        didInit= true;
                    }
                    // Activate Vision and check where is the gold mineral. Save the position.
                    //getStartGoldMineralPosition = visionProcessing();
                }

                // Stop all motion;
                robot.climbMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }catch (InterruptedException e) {
            throw new InterruptedException("climbMotor");
        }
    }

    // Function turns on the camera and enables processing.
    public void InitMyVision() throws InterruptedException{
        vision = new MineralVision();
        // Start to display image of camera.
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0);
        vision.setShowCountours(false);
        // start the vision system.
        vision.enable();
    }

    // Gyro angle
    public float GetGyroAngle() throws InterruptedException{

        Orientation angles =robot.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
        return(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
    }

    public void RestartAllEncoders() throws InterruptedException{
        robot.setAllMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setAllMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void pushClose() throws InterruptedException{
        try{
            while (robot.touchPusher.getState()) {
                robot.push.setPower(1);
                Thread.sleep(threadSleepTimeMS);
            }
        }catch (InterruptedException e) {
            throw new InterruptedException();
        }
    }
    public boolean JoysStickInDeadZone() throws InterruptedException{
        try{
            if(abs(gamepad1.left_stick_y)<0.3 && abs(gamepad1.left_stick_x)<0.3 ){
                Thread.sleep(threadSleepTimeMS);
                return true;
            }
            else{
                return false;
            }
        }catch (InterruptedException e) {
            throw new InterruptedException();
        }
    }

    public boolean TheFancyFance( double speed, double angle, double PCoeff) throws InterruptedException{
        try {
            {
            }
            double error;
            double steer;
            boolean onTarget = false;
            double leftSpeed;
            double rightSpeed;

            // determine turn power based on +/- error
            error = getError(angle);

            if (abs(error) <= HEADING_THRESHOLD) {
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
            robot.setDriveMotorsPower(rightSpeed-0.3, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
            return onTarget;

        }catch (InterruptedException e) {
            throw new InterruptedException();
        }

    }

    public void mineralUp() throws InterruptedException{
        try {
            pushClose();
            liftUntilStuck(1);
            telemetry.update();
            robot.blockMineralServo.setPosition(robot.dontBlock);   // Set Mode of servo to not block minerals.
            robot.mineralGrab.setPosition(FORWARD);
        }catch (InterruptedException e){
            throw new InterruptedException();
        }
    }


    public void TelementryRobotStartStatus(){
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Version", robot.Version);
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
    }

}


