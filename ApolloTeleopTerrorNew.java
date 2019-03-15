package org.firstinspires.ftc.teamcode;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Teleop New Terror", group="terror")

public class ApolloTeleopTerrorNew extends RobotFunctions {

    //HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private MineralVision vision;

    //private ElapsedTime runtime = new ElapsedTime();
    //private ElapsedTime time = new ElapsedTime();

    static double speedFactor = 1;  // Speed factor
    static double normalOrReversDrive = 1;  //

    static final double joyStickLimitPoints = 0.3;  // For better control

    boolean climbMotorInUse = false;
    boolean climbThreadActive = false;

    boolean moveMineralsThreadActive = false;

    boolean extrusionsThreadActive = false;

    @Override
    public void runOpMode() {
        //Hardware init
        Thread  climb = new climb();
        Thread  moveMinerals = new moveMinerals();
        Thread  extrusions = new extrusions();
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Version", robot.Version);
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        waitForStart();

        climb.start();
        moveMinerals.start();
        robot.mineralBoxServo.setPosition(robot.mineralBoxServoClose);


        while (opModeIsActive()) {
            //Controllers drive sticks inputs
            double LeftStickX = gamepad1.left_stick_x * normalOrReversDrive;
            double LeftStickY = -gamepad1.left_stick_y * normalOrReversDrive;  // The joystick goes negative when pushed forwards, so negate it.
            double RightStickX = gamepad1.right_stick_x * normalOrReversDrive;
            double RightStickY = -gamepad1.right_stick_y * normalOrReversDrive;    // The joystick goes negative when pushed forwards, so negate it

            // Thread auto climb set
            if(!climbThreadActive && !gamepad2.dpad_up && !gamepad2.dpad_down){
                climb.start();
            }




            // Game pad 1, left bumper. Revers all driving control.
            if (gamepad1.left_bumper) {
                normalOrReversDrive = -1;
            } else {
                normalOrReversDrive = 1;
            }

            //Drive speed control. Game pad 1, stick buttons.
            if (gamepad1.right_stick_button || gamepad1.left_stick_button) {
                speedFactor = 0.5;  // Decrease drive speed for better accuracy.
            } else {
                speedFactor = 1;    // Normal drive speed.
            }

            /** Main Driving **/
            // Drive modes control. Game pad 1, sticks.
            if (LeftStickX < -joyStickLimitPoints && (Math.abs(LeftStickY) > joyStickLimitPoints) &&
                    (RightStickX < -joyStickLimitPoints && Math.abs(RightStickY) > joyStickLimitPoints)) {       // If both joysticks are pushed to a left conner. drive diagonal left.
                robot.setDriveMotorsPower(LeftStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_LEFT);
                telemetry.addData("Drive", "DIAGONAL_LEFT");
            } else if (LeftStickX > joyStickLimitPoints && (Math.abs(LeftStickY) > joyStickLimitPoints) &&
                    (RightStickX > joyStickLimitPoints && Math.abs(RightStickY) > joyStickLimitPoints)) {       // If both joysticks are pushed to a right conner. drive diagonal right.
                robot.setDriveMotorsPower(LeftStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_RIGHT);
                telemetry.addData("Drive", "DIAGONAL_RIGHT");
            } else if ((Math.abs(LeftStickX) > Math.abs(LeftStickY) && Math.abs(LeftStickX) > joyStickLimitPoints && Math.abs(LeftStickY) < 0.8) &&
                    (Math.abs(RightStickX) > Math.abs(RightStickY) && Math.abs(RightStickX) > joyStickLimitPoints && Math.abs(RightStickY) < 0.8 &&
                            (RightStickX < 0 && LeftStickX < 0 || RightStickX > 0 && LeftStickX > 0))) {       // If both joysticks are pushed to the side. drive sideways.
                robot.setDriveMotorsPower(LeftStickX * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                telemetry.addData("Drive", "Side ways");
            } else {   // Drive Normally, tank mode.
                robot.setDriveMotorsPower(LeftStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                robot.setDriveMotorsPower(RightStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
                telemetry.addData("Drive", "Normal");
            }


            // mineral push, game pad 1 dpad.
            if(gamepad1.b){
                try {
                RestartAllEncoders();
                }catch (InterruptedException e) { }
            }


            //Mineral graber control. Game pad 2, triggers.
            if (gamepad2.left_trigger < 0 && gamepad2.right_trigger < 0) {
                robot.mineralGrab.setPosition(FORWORD);
            }
            if (gamepad2.right_trigger > 0) {
                robot.mineralGrab.setPosition(FORWORD);
            } else if (gamepad2.left_trigger > 0) {
                robot.mineralGrab.setPosition(BACKWORDS);
            } else {
                robot.mineralGrab.setPosition(STOP);
            }


            // Game Pad 1, triggers. Extrusions control.
            robot.mineralSend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad1.right_trigger > 0.1 )
            {   // Right trigger pushed, open extrusions.
                robot.mineralSend.setPower(gamepad1.right_trigger);
                if(!gamepad1.y) {
                    robot.mineralBoxServo.setPosition(1);
                }

            } else if (gamepad1.left_trigger > 0.1)

            {   // Left trigger pushed, close extrusions.
                robot.mineralSend.setPower(-0.25);
                robot.mineralBoxServo.setPosition(robot.mineralBoxServoOpen);
            } else {
                robot.mineralSend.setPower(0);
                robot.mineralSend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        // Yotam helped
            }


            // Mineral blocker control. Game pad 2 bumper.
            if (gamepad2.left_bumper) {
                robot.blockMineralServo.setPosition(robot.dontBlock);   // Set Mode of servo to not block minerals.
            } else {
                robot.blockMineralServo.setPosition(robot.block);   //Set Mode of servo to block minerals.
            }


            // Game pad 1, buttons. Mineral box control.
            if (gamepad1.y) {
                robot.mineralBoxServo.setPosition(robot.mineralBoxServoClose);
            } else if (gamepad1.a) {
                robot.mineralBoxServo.setPosition(robot.mineralBoxServoOpen);
            }


            // Mineral push Control. Game pad 2, right stick.
            robot.mineralSend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (-gamepad2.right_stick_y < 0.2 && robot.touchPusher.getState()) {
                robot.push.setPower(gamepad2.right_stick_y);
                //robot.blockMineralServo.setPosition(robot.block);
            } else if (-gamepad2.right_stick_y > -0.2) {
                robot.push.setPower(gamepad2.right_stick_y);
            } else{
                robot.push.setPower(0);
                robot.push.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        // Yotam helped
            }



            // Mineral lift Control. Game pad 2, left stick.
            if (-gamepad2.left_stick_y < 0 ) {
                robot.lift.setPower(-gamepad2.left_stick_y );
            } else if (-gamepad2.left_stick_y > 0  ){
                robot.lift.setPower(-gamepad2.left_stick_y );
            } else {

                robot.lift.setPower(0);
            }


            // Climb Control. Game pad 2, up down buttons.
            if (gamepad2.dpad_up && gamepad2.dpad_down) {
                if(!climbMotorInUse){
                    robot.climbMotor.setPower(0);
                }
            } else if (gamepad2.dpad_up) {
                robot.climbMotor.setPower(1);
                climb.interrupt();
                climbThreadActive=false;

            } else if (gamepad2.dpad_down) {
                robot.climbMotor.setPower(-1);
                climb.interrupt();
                climbThreadActive=false;

            } else {
                if(!climbMotorInUse) {
                    robot.climbMotor.setPower(0);
                }
            }

            if (robot.touchPusher.getState()) {
                // If left Stick pushed backwards and touch sensor  is not pressed activate mineral push and lift to push in.
                telemetry.addData("touch", "not pressed");
            }else {
                telemetry.addData("touch", "pressed");
            }




            telemetry.addData("Encoder lift", robot.lift.getCurrentPosition());
            telemetry.addData("Encoder push", robot.push.getCurrentPosition());
            telemetry.addData("Encoder climb", robot.climbMotor.getCurrentPosition());
            telemetry.addData("Encoder Sender", robot.mineralSend.getCurrentPosition());
            telemetry.addData("gyro", GetGyroAngle());


            telemetry.update();
            //climb.interrupt();
            //driving.interrupt();
        }

    }
    private class climb extends Thread
    {
        public climb()
        {
            this.setName("climb");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            try {
                while (opModeIsActive()) {
                    while (!isInterrupted()) {
                        telemetry.addData("thread","here");
                        telemetry.update();
                        climbThreadActive=true;
                        if (gamepad2.y) {
                            climbMotorInUse=true;
                            robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            encoderClimb(1, robot.climbLanderPosition);
                            climbMotorInUse=false;
                        }
                    }
                }
            }catch (InterruptedException e) { }

        }
    }


    private class moveMinerals extends Thread
    {
        public moveMinerals()
        {
            this.setName("moveMinerals");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            try {
                while (opModeIsActive()) {
                    while (!isInterrupted()) {
                        if(gamepad2.left_bumper){
                            if(robot.mineralSend.getCurrentPosition()<10){
                                moveMineralsThreadActive = true;
                                encoderPush(1,0);
                                robot.blockMineralServo.setPosition(robot.dontBlock);   // Set Mode of servo to not block minerals.
                                encoderLift(1,10);
                            }
                        }
                    }

                }
            }catch (InterruptedException e) { }
        }
    }


    private class extrusions extends Thread
    {
        public extrusions()
        {
            this.setName("extrusions");
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            try{
                while (opModeIsActive()) {
                    while (!isInterrupted()) {
                        extrusionsThreadActive = true;
                        encoderMineralSend(-0.25,0);
                    }
                }

            }catch (InterruptedException e) { }

        }
    }


    public float GetGyroAngle(){
        Orientation angles =robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
    }


    // Function gets the climbing motor to the wanted position.
    // Opens systems, and activates vision to check where the gold mineral, in the same time.
    public void encoderClimbThread(double speed, int ticks) throws InterruptedException {
        try {
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
                    Thread.sleep(0);
                }

                // Stop all motion;
                robot.climbMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } catch (InterruptedException e) {
            throw new InterruptedException();
        }
    }

}

