package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import java.util.List;

/**
 * Apollo Autonomus.
 */

public abstract class AutoMain extends LinearOpMode {
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED  = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF      = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF     = 0.15;     // Larger is more responsive, but also less stable

    static final double     mineralGraberPower = 0.7 ;
    private MineralVision vision;

    private enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE
    }

    public class MineralOnBot{
        HardwareApollo.TYPE_MINERALS[] mineralsType = new HardwareApollo.TYPE_MINERALS[2];
        int numberOfMinerals ;
    }

    //Init function, hardwareMap
    public void apolloInit() {
        robot.init(hardwareMap);
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system
        vision.enable();
    }

    //The main function of the autonomous
    void apolloRun(boolean isCrater){

    }

    //Function to get two minerals.
    public MineralOnBot getTwoMineral(){
        //Activate all motors to grab minerals, and make robot slowly move forward to get more minerals
        robot.setModeAllMineralsMotor(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.runAllMineralsMotor(mineralGraberPower);
        robot.setDriveMotorsPower(0.05, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);

        // Reset mineral on bot parameters
        MineralOnBot mineral = null;
        mineral.numberOfMinerals = 0;

        telemetry.addData("Minerals", mineral.numberOfMinerals);
        telemetry.update();

        while (mineral.numberOfMinerals<2 && opModeIsActive()) {
            if (mineralTypeDetect() == HardwareApollo.TYPE_MINERALS.GOLD) {
                mineral.mineralsType[mineral.numberOfMinerals] = HardwareApollo.TYPE_MINERALS.GOLD;
                mineral.numberOfMinerals++;
                telemetry.addData("Minerals", mineral.numberOfMinerals);
                telemetry.addData("Mineral", "number: Type: = %d, GOLD",
                        mineral.numberOfMinerals);
                telemetry.update();
                waitSeconds(2);
            } else if (mineralTypeDetect() == HardwareApollo.TYPE_MINERALS.SILVER) {
                mineral.mineralsType[mineral.numberOfMinerals] = HardwareApollo.TYPE_MINERALS.SILVER;
                mineral.numberOfMinerals++;
                telemetry.addData("Minerals", mineral.numberOfMinerals);
                telemetry.addData("Mineral", "number: Type: = %d, SILVER",
                        mineral.numberOfMinerals);
                telemetry.update();
                waitSeconds(2);
            }
        }
        //stop motors (stop grabbing minerals)
        robot.runAllMineralsMotor(0);
        robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
        //return what minerals the robot got (in order).
        return (mineral);

    }

    //Function returns which mineral the camera detected.
    public HardwareApollo.TYPE_MINERALS mineralTypeDetect(){
        vision.setShowCountours(true);
        if(vision.goldMineralFound()== true) {
            return HardwareApollo.TYPE_MINERALS.GOLD;
        }
        if(vision.silverMineralFound()== true) {
            return HardwareApollo.TYPE_MINERALS.SILVER;
        }
        telemetry.update();
        return null;
    }

    //Function returns the location of the gold mineral.
    public GoldPosition GetGoldMineralPosition(){
        List<MatOfPoint> goldContours = vision.getGoldContours();
        List<MatOfPoint> silverContours = vision.getSilverContours();

        if(vision.goldMineralFound()== true && goldContours.size()  >=1 ) {
            Rect GoldBoundingRect = Imgproc.boundingRect(goldContours.get(0));

            int goldXPosition = GoldBoundingRect.x;
            if (goldXPosition < 150) {
                return GoldPosition.LEFT;
            } else if (goldXPosition > 1000) {
                return GoldPosition.RIGHT;
            } else if (goldXPosition > 450 && goldXPosition < 650) {
                return GoldPosition.MIDDLE;
            }
        }
        return null;
    }

    public void waitSeconds(double seconds){
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {}
    }

    //TO DO:
    public void moveToGoldMineralByCamera(){

    }

    public void gyroDrive ( double speed,
                            double tickDistance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(tickDistance);
            newLeftTarget = robot.driveLeftFront.getCurrentPosition() + moveCounts;
            newRightTarget = robot.driveRightFront.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.setDriveMotorsPosition(newLeftTarget, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
            robot.setDriveMotorsPosition(newRightTarget, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);


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
                if (tickDistance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
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
    public void gyroHold( double speed, double angle, double holdTime) {

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
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setDriveMotorsPower(leftSpeed, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
        robot.setDriveMotorsPower(rightSpeed, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}


