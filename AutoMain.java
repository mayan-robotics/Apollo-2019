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

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double DRIVE_SIDEWAYS_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.
    static final double SIDE_WAYS_DRIVE_SPEED = 0.4;     // Nominal speed for better accuracy.


    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    static final double mineralGraberPower = 0.7;
    private MineralVision vision;
    private List<MatOfPoint> contoursGold = new ArrayList<>();

    private enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE
    }


    //Init function, hardwareMap
    public void apolloInit() {
        //Hardware init
        robot.init(hardwareMap);
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        vision.enable();
        vision.setShowCountours(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
    }

    //The main function of the autonomous
    void apolloRun(boolean isCrater)
    {
        vision.setShowCountours(true);
        while (GetGoldLocation() != GoldPosition.MIDDLE){
            moveToGoldMineralByCamera();
        }
        waitSeconds(1);
        while (GetGoldLocation() != GoldPosition.MIDDLE){
            moveToGoldMineralByCamera();
        }
        telemetry.addData("Vision","cube moved");
        telemetry.update();

        if(isCrater)
        {

        }
        else if (!isCrater)
        {

        }

    }

    //Function returns which mineral the camera detected.
    public HardwareApollo.TYPE_MINERALS mineralTypeDetect()
    {
        vision.setShowCountours(true);
        if (vision.goldMineralFound() == true) {
            return HardwareApollo.TYPE_MINERALS.GOLD;
        }
        telemetry.update();
        return null;
    }

    //Function returns the location of the gold mineral.
    public GoldPosition GetGoldLocation()
    {

        contoursGold.clear();
        vision.getGoldContours(contoursGold);

        if ((vision.goldMineralFound()) && (contoursGold != null)) {
            if (!contoursGold.isEmpty())  {

                if (contoursGold.get(0) != null) {

                    Rect GoldBoundingRect = Imgproc.boundingRect(contoursGold.get(0));
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
        return null;
    }


    public void moveToGoldMineralByCamera(){

        if(GetGoldLocation()!= null) {
            if (GetGoldLocation() == GoldPosition.LEFT) {
                telemetry.addData("Drive", "left");
                robot.setDriveMotorsPower(-DRIVE_SIDEWAYS_SPEED, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            } else if (GetGoldLocation() == GoldPosition.RIGHT) {
                telemetry.addData("Drive", "right");
                robot.setDriveMotorsPower(DRIVE_SIDEWAYS_SPEED, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);            }
            else if (GetGoldLocation() == GoldPosition.MIDDLE) {
                telemetry.addData("Drive", "Middle");
                robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
            }else {
                telemetry.addData("camera", "error");
            }
        }
    }


    public void waitSeconds(double seconds)
    {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
        }
    }


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
    public void gyroTurn ( double speed, double angle)
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

    public void encoderSideWaysDrive(double speed,
                                     double Distance,
                                     double timeoutS)
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
                    (runtime.seconds() < timeoutS) &&
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
}


