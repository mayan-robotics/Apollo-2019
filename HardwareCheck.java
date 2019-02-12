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
 * Check robot hardware.
 */

@Autonomous(name="Test: Hardware check", group="Test")
public class HardwareCheck extends AutoMain{
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Version", robot.Version);
        telemetry.addData("ROBOT","SHOULD NOT BE ON THE GROUND");
        telemetry.update();
        waitSeconds(3);
        telemetry.clear();

        try {
            robot.init(hardwareMap);
            telemetry.addData("Apollo","init succeeded");
            //telemetry.update();
        }catch (Exception e){
            telemetry.addData("ERROR01","INIT FAILED!   check your config");
            telemetry.update();
        }

        try {
            robot.driveLeftFront.setTargetPosition(1);
            robot.driveRightFront.setTargetPosition(1);
            robot.driveLeftBack.setTargetPosition(1);
            robot.driveRightBack.setTargetPosition(1);

            double dlf = robot.driveLeftFront.getCurrentPosition();
            double drf = robot.driveRightFront.getCurrentPosition();
            double dlb = robot.driveLeftBack.getCurrentPosition();
            double drb = robot.driveRightBack.getCurrentPosition();
            telemetry.addData("motor",dlf);
            telemetry.addData("motor",drf);
            telemetry.addData("motor",dlb);
            telemetry.addData("motor",drb);
            telemetry.update();

            if(dlf!=1 || drf!=1 || dlb!=1 || drb!=1){
                telemetry.addData("ERROR02","DRIVE MOTORS ARE NOT DETECTED!   check motors connections");
                telemetry.update();
            }else{
                telemetry.addData("DRIVE","succeed");
            }
            //telemetry.clear();

            telemetry.addData("DRIVE","succeed");
            telemetry.update();
        }catch (Exception e) {
            telemetry.addData("ERROR02","DRIVE MOTORS ARE NOT DETECTED!   check motors connections");
            telemetry.update();
        }

        try {
            double lift = robot.lift.getCurrentPosition();
            telemetry.addData("motor",lift);
            telemetry.clear();
        }catch (Exception e){
            telemetry.addData("ERROR03","DC MOTORS ARE NOT DETECTED!   check lift motor connections");
        }

        try {
            //double md = robot.mineralsDivider.getPosition();
            double b = robot.blockMineralServo.getPosition();
            //telemetry.addData("servo", md);
            telemetry.addData("servo",b);
            telemetry.clear();
        }catch (Exception e){
            telemetry.addData("ERROR04","SERVOS ARE NOT DETECTED!   check motors connections");
        }
        telemetry.addData("Apollo","Init test done");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


        telemetry.addData("X", "Encoder drive speed test");
        telemetry.addData("Y", "Encoder drive one wheel rotation");
        telemetry.addData("B", "Gyro drive test");
        telemetry.addData("A", "Gyro drive test square");
        telemetry.addData("RB", "Drive wheels test");
        telemetry.addData("LB", "Drive Modes test");

        telemetry.update();

        while (opModeIsActive()) {
            if(gamepad1.left_trigger>0){
                telemetry.addData("Left Front",robot.driveLeftFront.getCurrentPosition());
                telemetry.addData("Left Back",robot.driveLeftBack.getCurrentPosition());
                telemetry.addData("Right Front",robot.driveRightFront.getCurrentPosition());
                telemetry.addData("Right Back",robot.driveRightBack.getCurrentPosition());
                telemetry.update();
            }


            if (gamepad1.x)
            {
                //Encoder drive speed test
                telemetry.clear();
                telemetry.addData("To STOP press","right trigger");
                telemetry.update();
                while (opModeIsActive() && gamepad1.right_trigger < 0.2)
                {
                    robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.setDriveMotorsPower(0.1, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                    telemetry.addData("Left Front",robot.driveLeftFront.getCurrentPosition());
                    telemetry.addData("Left Back",robot.driveLeftBack.getCurrentPosition());
                    telemetry.addData("Right Front",robot.driveRightFront.getCurrentPosition());
                    telemetry.addData("Right Back",robot.driveRightBack.getCurrentPosition());
                    telemetry.update();
                }
                robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                telemetry.addData("Test","Stopped");
                telemetry.update();
            } else if (gamepad1.y)
            {
                //Encoder drive one wheel rotation
                telemetry.clear();
                while (opModeIsActive() && gamepad1.right_trigger < 0.2) {
                    int newLeftFrontTarget = robot.driveLeftFront.getCurrentPosition() + (int) (robot.COUNTS_PER_INCH);
                    int newLeftBackTarget = robot.driveLeftBack.getCurrentPosition() + (int) (robot.COUNTS_PER_INCH);
                    int newRightFrontTarget = robot.driveRightFront.getCurrentPosition() + (int) (robot.COUNTS_PER_INCH);
                    int newRightBackTarget = robot.driveRightBack.getCurrentPosition() + (int) (robot.COUNTS_PER_INCH);

                    robot.driveLeftFront.setTargetPosition(newLeftFrontTarget);
                    robot.driveLeftBack.setTargetPosition(newLeftBackTarget);
                    robot.driveRightFront.setTargetPosition(newRightFrontTarget);
                    robot.driveRightBack.setTargetPosition(newRightBackTarget);

                    robot.setDriveMotorsPower(0.5, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                    robot.setDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
                    telemetry.addData("Apollo","finished test");
                    telemetry.update();
                    break;
                }
                telemetry.addData("Test","Stopped");
                telemetry.update();
            } else if (gamepad1.b)
            {
                //Gyro drive test
                telemetry.clear();
                //while (opModeIsActive() && gamepad1.right_trigger < 0.2)
                //{
                    gyroDrive(0.2, 130, 0);
                //}
                telemetry.addData("Test","Stopped");
                telemetry.update();
            } else if (gamepad1.a)
            {
                //Gyro drive square
                telemetry.clear();
                while (opModeIsActive() && gamepad1.right_trigger < 0.2)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        driveByGyro(0.7, 150, 90 * i);
                        waitSeconds(1);
                        turnByGyro(TURN_SPEED, 90 * (i + 1));
                        waitSeconds(2);
                    }
                    telemetry.addData("Apollo","finished test");
                    telemetry.update();
                    waitSeconds(1);
                    break;
                }
                telemetry.clear();
                telemetry.addData("Test","Stopped");
                telemetry.update();
            }
            else if (gamepad1.right_bumper)
            {
                //Drive wheels test
                telemetry.clear();
                while (opModeIsActive() && gamepad1.right_trigger < 0.2)
                {
                    robot.driveLeftFront.setPower(0.2);
                    waitSeconds(3);
                    robot.driveLeftFront.setPower(0);
                    if(gamepad1.right_trigger < 0.2){break;}

                    robot.driveLeftBack.setPower(0.2);
                    waitSeconds(3);
                    robot.driveLeftBack.setPower(0);
                    if(gamepad1.right_trigger < 0.2){break;}

                    robot.driveRightBack.setPower(0.2);
                    waitSeconds(3);
                    robot.driveRightBack.setPower(0);
                    if(gamepad1.right_trigger < 0.2){break;}

                    robot.driveRightFront.setPower(0.2);
                    waitSeconds(3);
                    robot.driveRightFront.setPower(0);
                    if(gamepad1.right_trigger < 0.2){break;}
                    telemetry.addData("Apollo","finished test");
                    telemetry.update();
                    break;
                }
                telemetry.clear();
                telemetry.addData("Test","Stopped");
                telemetry.update();
            }
            else if (gamepad1.left_bumper)
            {
                //Drive Modes test
                telemetry.clear();
                while (opModeIsActive() && gamepad1.right_trigger < 0.2)
                {
                    robot.setDriveMotorsPower(0.2, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                    waitSeconds(3);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                    waitSeconds(1);
                    robot.setDriveMotorsPower(-0.2, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                    waitSeconds(3);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
                    if(gamepad1.right_trigger < 0.2){break;}
                    waitSeconds(2);


                    robot.setDriveMotorsPower(0.2, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                    waitSeconds(3);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                    waitSeconds(1);
                    robot.setDriveMotorsPower(-0.2, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                    waitSeconds(3);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                    if(gamepad1.right_trigger < 0.2){break;}
                    waitSeconds(2);

                    robot.setDriveMotorsPower(0.2, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
                    waitSeconds(3);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
                    waitSeconds(1);
                    robot.setDriveMotorsPower(-0.2, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
                    waitSeconds(3);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
                    if(gamepad1.right_trigger < 0.2){break;}
                    waitSeconds(2);

                    robot.setDriveMotorsPower(0.2, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                    waitSeconds(5);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                    waitSeconds(2);
                    robot.setDriveMotorsPower(-0.2, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                    waitSeconds(5);
                    robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                    if(gamepad1.right_trigger < 0.2){break;}
                    telemetry.addData("Apollo","finished test");
                    telemetry.update();
                    break;
                }
                telemetry.clear();
                telemetry.addData("Test","Stopped");
                telemetry.update();
            }
            else if(gamepad1.dpad_up){
                gyroDriveSideWays(1,30,0);
            }
            else {
                telemetry.addData("X", "Encoder drive speed test");
                telemetry.addData("Y", "Encoder drive one wheel rotation");
                telemetry.addData("B", "Gyro drive test");
                telemetry.addData("A", "Gyro drive test square");
                telemetry.addData("RB", "Drive wheels test");
                telemetry.addData("LB", "Drive Modes test");
            }
            telemetry.update();
        }
    }


    public void waitSeconds(double seconds){
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {}
    }

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

    public void encoderLift(double Distance, double speed) {

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

    public void encoderSideWaysDrive(double speed,
                                     double Distance) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.setDriveMotorsPosition(Distance, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            // Turn On RUN_TO_POSITION
            robot.setDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
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

    public void turnByGyro(double speed, double angle){
        for (int i=0; i<3;i++) {
            gyroTurn(speed,angle);
        }
    }
    public void driveByGyro(double speed, double Distance, double angle){
        gyroDrive(speed, Distance, angle);
        turnByGyro(TURN_SPEED,angle);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void gyroDriveSideWays ( double speed,
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
            robot.setDriveMotorsPosition(moveCounts, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
            //robot.setDriveMotorsPosition(moveCounts, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);


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

                robot.driveLeftFront.setPower(leftSpeed);
                robot.driveRightBack.setPower(leftSpeed);
                robot.driveLeftBack.setPower(rightSpeed);
                robot.driveRightFront.setPower(rightSpeed);

                //robot.setDriveMotorsPower(leftSpeed, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                //robot.setDriveMotorsPower(rightSpeed, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
            }

            // Stop all motion;
            robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);

            // Turn off RUN_TO_POSITION
            robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}

