package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Apollo Autonomus.
 */
@Autonomous(name="Apollo: Auto test", group="Apollo")

public class AutoTestOne extends LinearOpMode {
    HardwareTeleopTest robot = new HardwareTeleopTest(); // use Apollo's hardware
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


    public class MineralOnBot{
        HardwareApollo.TYPE_MINERALS[] mineralsType = new HardwareApollo.TYPE_MINERALS[2];
        int numberOfMinerals ;
    }

    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        waitForStart();

        //robot.setDriveMotorsPower(0.5, HardwareTeleopTest.DRIVE_MOTOR_TYPES.RIGHT);

        gyroDrive(DRIVE_SPEED,50,0);
        encoderSideWaysDrive(0.8,500,0);

        //waitSeconds(2);

        //gyroTurn(TURN_SPEED,90);
        //gyroTurn(TURN_SPEED,-90);
        //gyroTurn(TURN_SPEED,180);
        //gyroTurn(TURN_SPEED,-180);
        //gyroDrive(DRIVE_SPEED,50,0);
        //gyroDrive(DRIVE_SPEED,-50,0);

        //gyroDrive(DRIVE_SPEED,50,0);
        //gyroTurn(TURN_SPEED,-90);
        //gyroDrive(DRIVE_SPEED,80,0);
        //gyroTurn(TURN_SPEED,-45);
        //while (opModeIsActive()){
        //    encoderSideWaysDrive(0,300,0);
        //    waitSeconds(2);
        //    //encoderSideWaysDrive(1,200,0);
//
        //}

    }

    //Function to get two minerals.
    public MineralOnBot getTwoMineral(){
        //Activate all motors to grab minerals, and make robot slowly move forward to get more minerals
        robot.setModeAllMineralsMotor(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.runAllMineralsMotor(mineralGraberPower);
        robot.setDriveMotorsPower(0.05, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);

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
        robot.setDriveMotorsPower(0, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);
        //return what minerals the robot got (in order).
        return (mineral);

    }

    //Function returns which mineral the camera detected.
    public HardwareApollo.TYPE_MINERALS mineralTypeDetect(){
        vision.setShowCountours(true);
        if(vision.goldMineralFound()== true) {
            return HardwareApollo.TYPE_MINERALS.GOLD;
        }
        //if(vision.silverMineralFound()== true) {
            //return HardwareApollo.TYPE_MINERALS.SILVER;
        //}
        telemetry.update();
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
                            double Distance,
                            double angle){

        int newLeftTarget;
        int newRightTarget;
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
            //newLeftTarget = robot.driveLeftFront.getCurrentPosition() + moveCounts;
            //newRightTarget = robot.driveRightFront.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            //robot.setDriveMotorsPosition(newLeftTarget, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
            //robot.setDriveMotorsPosition(newRightTarget, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
            robot.setDriveMotorsPosition(moveCounts, HardwareTeleopTest.DRIVE_MOTOR_TYPES.LEFT);
            robot.setDriveMotorsPosition(moveCounts, HardwareTeleopTest.DRIVE_MOTOR_TYPES.RIGHT);


            robot.setDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            robot.setDriveMotorsPower(speed, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);

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

                robot.setDriveMotorsPower(leftSpeed, HardwareTeleopTest.DRIVE_MOTOR_TYPES.LEFT);
                robot.setDriveMotorsPower(rightSpeed, HardwareTeleopTest.DRIVE_MOTOR_TYPES.RIGHT);

            }

            // Stop all motion;
            robot.setDriveMotorsPower(0, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);

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
    public void gyroTurn ( double speed, double angle){

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
    public void gyroHold ( double speed, double angle, double holdTime){

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.setDriveMotorsPower(0, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);

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
    boolean onHeading ( double speed, double angle, double PCoeff){
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
        robot.setDriveMotorsPower(leftSpeed, HardwareTeleopTest.DRIVE_MOTOR_TYPES.LEFT);
        robot.setDriveMotorsPower(rightSpeed, HardwareTeleopTest.DRIVE_MOTOR_TYPES.RIGHT);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError ( double targetAngle){

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
    public double getSteer ( double error, double PCoeff){
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void encoderSideWaysDrive(double speed,
                                     double Distance,
                                     double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            //// Determine new target position, and pass to motor controller
            //newLeftFrontTarget = robot.driveLeftFront.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
            //newLeftBackTarget = robot.driveLeftBack.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            //newRightFrontTarget = robot.driveRightFront.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            //newRightBackTarget = robot.driveRightBack.getCurrentPosition() - (int)(distance * COUNTS_PER_INCH);
//
            ////newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            //robot.driveLeftFront.setTargetPosition(newLeftFrontTarget);
            //robot.driveLeftBack.setTargetPosition(newLeftBacktTarget);
            //robot.driveRightFront.setTargetPosition(newRightFrontTarget);
            //robot.driveRightBack.setTargetPosition(newRightBackTarget);

            //robot.setDriveMotorsPosition(newLeftTarget, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);

            robot.setDriveMotorsPosition(Distance, HardwareTeleopTest.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            // Turn On RUN_TO_POSITION
            robot.setDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            //robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.setDriveMotorsPower(Math.abs(speed), HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);

            //robot.leftDrive.setPower(Math.abs(speed));
            //robot.rightDrive.setPower(Math.abs(speed));

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
            robot.setDriveMotorsPower(0, HardwareTeleopTest.DRIVE_MOTOR_TYPES.ALL);

            // Turn off RUN_TO_POSITION
            robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}

