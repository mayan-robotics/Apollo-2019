package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Teleop Apollo Terror", group="terror")

public class ApolloTeleopLinear extends LinearOpMode {

    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private MineralVision vision;

    private ElapsedTime runtime = new ElapsedTime();

    static double speedFactor = 1;  // Speed factor
    static double normalOrReversDrive = 1;  //

    static final double joyStickLimitPoints = 0.3;  // For better control
    static final double senderOpenLimitPoint = 7900; // Limit so the sender motors wont open to much, by encoder ticks.
    static final double senderCloseLimitPoint = 0; // Limit so the sender motors wont open to much, by encoder ticks.
    static final double convertJoystickToPosition = 10; //
    int mineralSenderPosition; // Encoder positions
    int mineralSenderWantedPosition = 0; // Encoder positions

    boolean blockMineral = false;

    int liftencoderPosition = 0;
    int climbPosition = 0;
    int liftPosition = 0;
    int pushPosition = 0;

    double positionServo = 0;


    @Override
    public void runOpMode() {
        //Hardware init
        robot.init(hardwareMap);
        //robot.setMineralSendMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Vision
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        vision.enable();
        vision.setShowCountours(true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Version", robot.Version);
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {


            //Controllers drive sticks inputs
            double LeftStickX = gamepad1.left_stick_x * normalOrReversDrive;
            double LeftStickY = -gamepad1.left_stick_y * normalOrReversDrive;  // The joystick goes negative when pushed forwards, so negate it.
            double RightStickX = gamepad1.right_stick_x * normalOrReversDrive;
            double RightStickY = -gamepad1.right_stick_y * normalOrReversDrive;    // The joystick goes negative when pushed forwards, so negate it

            if (gamepad1.left_bumper) {
                normalOrReversDrive = -1;
            } else {
                normalOrReversDrive = 1;
            }

            if (gamepad1.dpad_up) {
                if (speedFactor < 1) {
                    speedFactor++;
                }
            } else if (gamepad1.dpad_down) {
                if (speedFactor > 0) {
                    speedFactor--;
                }
            }
            //Drive speed control. Game pad 1, stick buttons.
            if (gamepad1.right_stick_button || gamepad1.left_stick_button) {
                speedFactor = 0.5;  // Decrease drive speed
            } else {
                speedFactor = 1;    // Normal drive speed
            }

            // Drive modes control. Game pad 1, sticks.
            if (LeftStickX < -joyStickLimitPoints && (Math.abs(LeftStickY) > joyStickLimitPoints) &&
                    (RightStickX < -joyStickLimitPoints && Math.abs(RightStickY) > joyStickLimitPoints)) {   // If both joysticks are pushed to a left conner. drive diagonal left.
                robot.setDriveMotorsPower(LeftStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_LEFT);
                telemetry.addData("Drive", "DIAGONAL_LEFT");
            } else if (LeftStickX > joyStickLimitPoints && (Math.abs(LeftStickY) > joyStickLimitPoints) &&
                    (RightStickX > joyStickLimitPoints && Math.abs(RightStickY) > joyStickLimitPoints)) {   // If both joysticks are pushed to a right conner. drive diagonal right.
                robot.setDriveMotorsPower(LeftStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_RIGHT);
                telemetry.addData("Drive", "DIAGONAL_RIGHT");
            } else if ((Math.abs(LeftStickX) > Math.abs(LeftStickY) && Math.abs(LeftStickX) > joyStickLimitPoints && Math.abs(LeftStickY) < 0.8) &&
                    (Math.abs(RightStickX) > Math.abs(RightStickY) && Math.abs(RightStickX) > joyStickLimitPoints && Math.abs(RightStickY) < 0.8 &&
                            (RightStickX < 0 && LeftStickX < 0 || RightStickX > 0 && LeftStickX > 0))) {   // If both joysticks are pushed to the side. drive sideways.
                robot.setDriveMotorsPower(LeftStickX * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                telemetry.addData("Drive", "Side ways");
            } else {   // Drive Normally, tank mode.
                robot.setDriveMotorsPower(LeftStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                robot.setDriveMotorsPower(RightStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
                telemetry.addData("Drive", "Normal");
            }
            telemetry.update();


            //Mineral graber control. Game pad 2, triggers.
            if (gamepad2.left_trigger < 0 && gamepad2.right_trigger < 0) {
                telemetry.addData("main graber", "stop");
                //robot.mineralGrab.setPower(0);
                robot.setMineralGrabServos(0);
            }
            if (gamepad2.left_trigger > 0) {
                telemetry.addData("main graber", "grab out");
                telemetry.addData("power graber", gamepad2.left_trigger);
                //robot.mineralGrab.setPower(gamepad2.left_trigger);
                robot.setMineralGrabServos(0.8);
            } else if (gamepad2.right_trigger > 0) {
                telemetry.addData("main graber", "grab in");
                telemetry.addData("power graber", gamepad2.right_trigger);
                //robot.mineralGrab.setPower(-gamepad2.right_trigger);
                robot.setMineralGrabServos(0.2);
            } else {
                //robot.mineralGrab.setPower(0);
                robot.setMineralGrabServos(0);
            }

            //Mineral sender control. Game pad 2, right stick.
            //if (Math.abs(gamepad1.right_trigger) > 0) {
            robot.mineralSend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad1.right_trigger > 0.1 && mineralSenderPosition <= senderOpenLimitPoint) {
                //telemetry.addData("1", "here");

                robot.mineralBoxServo.setPosition(0.1);
                robot.mineralSend.setPower(gamepad1.right_trigger);
                mineralSenderWantedPosition = robot.mineralSend.getCurrentPosition();
                //robot.mineralBoxServo.setPosition(robot.mineralBoxBlock);
            } else if (gamepad1.left_trigger > 0.1 && mineralSenderPosition >= senderCloseLimitPoint) {
                //telemetry.addData("2", "here");
                robot.mineralSend.setPower(-gamepad1.left_trigger);
                mineralSenderWantedPosition = robot.mineralSend.getCurrentPosition();
                //robot.mineralBoxServo.setPosition(robot.mineralBoxBlock);
            } else {
                robot.mineralSend.setPower(0);

                //}
            }
            //else{
            //robot.setMineralSendPower(0);
            //}
            mineralSenderPosition = robot.mineralSend.getCurrentPosition();
            //telemetry.addData("Sender Encoder ", robot.mineralSend.getCurrentPosition());
            //telemetry.addData("Sender Want Encoder ", mineralSenderPosition);
            //telemetry.update();

            // Mineral blocker control. Game pad 2 bumper.
            if (gamepad2.left_bumper) {
                if (blockMineral) {
                    blockMineral = false; // Set Mode of servo to not block minerals.
                } else {
                    blockMineral = true;  //Set Mode of servo to block minerals.

                }
            }
/*
        // Keep setting position of servo by the mode its on.
        if (blockMineral){
            robot.blockMineralServo.setPosition(robot.block);
        }
        else {
            robot.blockMineralServo.setPosition(robot.dontBlock);
        }
*/

            if (gamepad2.left_bumper) {
                robot.blockMineralServo.setPosition(robot.dontBlock);
            } else {
                robot.blockMineralServo.setPosition(robot.block);

            }


/*
        // Mineral blocker control. Game pad 2 bumper.
        if (gamepad1.x) {
            positionServo=0.2;
            //robot.mineralBoxServo.setPosition(0.4);
        } else if (gamepad1.y) {
            positionServo=0.5;
        }
            //robot.mineralBoxServo.setPosition(0.5); }
        else if(gamepad1.a){
            positionServo=0.8;
            //robot.mineralBoxServo.setPosition(0.55);
        }
        robot.mineralBoxServo.setPosition(positionServo);
*/

            // Mineral blocker control. Game pad 2 bumper.
            if (gamepad1.a) {
                robot.mineralBoxServo.setPosition(0.1);
            } else if (gamepad1.x) {
                robot.mineralBoxServo.setPosition(0.3);
            } else if (gamepad1.y) {
                robot.mineralBoxServo.setPosition(1);
            }

            pushPosition = robot.push.getCurrentPosition();

            //Mineral lift Control. Game pad 2, left stick.
            if (-gamepad2.right_stick_y > 0.2) {
                // If left Stick pushed forwards activate mineral push and lift to push out
                robot.push.setPower(1);
                //robot.lift.setPower(0.3);
                robot.blockMineralServo.setPosition(robot.block);
            } else if (-gamepad2.right_stick_y < -0.2 /* &&  pushPosition<4400 */) {
                //if (robot.touchPusher.getState()) {
                // If left Stick pushed backwards and touch sensor  is not pressed activate mineral push and lift to push in.
                //telemetry.addData("touch", "not pressed");
                robot.push.setPower(-1);
            } else {
                robot.push.setPower(0);
            }
            telemetry.addData("controler", -gamepad2.right_stick_y);
            telemetry.update();


            liftPosition = robot.lift.getCurrentPosition();
            if (-gamepad2.left_stick_y < 0 ) {
                robot.lift.setPower(-gamepad2.left_stick_y);
                liftencoderPosition = robot.lift.getCurrentPosition();
            } else if (-gamepad2.left_stick_y > 0) {
                robot.lift.setPower(-gamepad2.left_stick_y);
                liftencoderPosition = robot.lift.getCurrentPosition();
            } else {
                //robot.push.setPower(0);
                robot.lift.setPower(0);
                //robot.lift.setTargetPosition(liftencoderPosition);
                //robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            climbPosition = robot.climbMotor.getCurrentPosition();

            if (gamepad2.dpad_up && gamepad2.dpad_down) {
                robot.climbMotor.setPower(0);
            } else if (gamepad2.dpad_up) {
                robot.climbMotor.setPower(1);
            } else if (gamepad2.dpad_down) {
                robot.climbMotor.setPower(-1);
            } else {
                robot.climbMotor.setPower(0);
            }


            telemetry.addData("Sender Encoder lift", robot.lift.getCurrentPosition());
            telemetry.addData("Sender Encoder push", robot.push.getCurrentPosition());
            telemetry.addData("Sender Encoder climb", robot.climbMotor.getCurrentPosition());


            telemetry.update();
        }
    }
}

