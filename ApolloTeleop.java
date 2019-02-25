package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Teleop Apollo", group="Apollo")

public class ApolloTeleop extends LinearOpMode {

    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private MineralVision vision;

    private ElapsedTime runtime = new ElapsedTime();

    static double speedFactor = 1;  // Speed factor
    static double normalOrReversDrive = 1;  //

    static final double joyStickLimitPoints = 0.3;  // For better control


    static final double senderOpenEncoderLimitPoint = 7900; // Limit so the sender motors wont open to much, by encoder ticks.
    static final double senderCloseEncoderLimitPoint = 0; // Limit so the sender motors wont open to much, by encoder ticks.
    static final double liftOpenEncoderLimitPoint = 690;
    static final double liftCloseEncoderLimitPoint = 100;
    static final double pushOpenEncoderLimitPoint = 5000;
    static final double pushCloseEncoderLimitPoint = 0;


    // Encoder motors positions.
    int mineralSenderPosition;
    int climbPosition = 0;
    int liftPosition = 0;
    int pushPosition = 0;


    @Override
    public void runOpMode() {
        //Hardware init
        robot.init(hardwareMap);

        // Send telemetry message to signify robot is ready;
        telemetry.addData("Version", robot.Version);      // Program date.
        telemetry.addData("Apollo", "Init success");    // Telemetry message to signify the robot is ready.
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //Controllers drive sticks inputs
            double LeftStickX = gamepad1.left_stick_x * normalOrReversDrive;
            double LeftStickY = -gamepad1.left_stick_y * normalOrReversDrive;  // The joystick goes negative when pushed forwards, so negate it.
            double RightStickX = gamepad1.right_stick_x * normalOrReversDrive;
            double RightStickY = -gamepad1.right_stick_y * normalOrReversDrive;    // The joystick goes negative when pushed forwards, so negate it

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
                    (RightStickX < -joyStickLimitPoints && Math.abs(RightStickY) > joyStickLimitPoints))
            {       // If both joysticks are pushed to a left conner. drive diagonal left.
                robot.setDriveMotorsPower(LeftStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_LEFT);
                telemetry.addData("Drive", "DIAGONAL_LEFT");
            }
            else if (LeftStickX > joyStickLimitPoints && (Math.abs(LeftStickY) > joyStickLimitPoints) &&
                    (RightStickX > joyStickLimitPoints && Math.abs(RightStickY) > joyStickLimitPoints))
            {       // If both joysticks are pushed to a right conner. drive diagonal right.
                robot.setDriveMotorsPower(LeftStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_RIGHT);
                telemetry.addData("Drive", "DIAGONAL_RIGHT");
            }
            else if ((Math.abs(LeftStickX) > Math.abs(LeftStickY) && Math.abs(LeftStickX) > joyStickLimitPoints && Math.abs(LeftStickY) < 0.8) &&
                    (Math.abs(RightStickX) > Math.abs(RightStickY) && Math.abs(RightStickX) > joyStickLimitPoints && Math.abs(RightStickY) < 0.8 &&
                            (RightStickX < 0 && LeftStickX < 0 || RightStickX > 0 && LeftStickX > 0)))
            {       // If both joysticks are pushed to the side. drive sideways.
                robot.setDriveMotorsPower(LeftStickX * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
                telemetry.addData("Drive", "Side ways");
            }
            else
            {   // Drive Normally, tank mode.
                robot.setDriveMotorsPower(LeftStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
                robot.setDriveMotorsPower(RightStickY * speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
                telemetry.addData("Drive", "Normal");
            }



            // mineral push, game pad 1 dpad.
            if(gamepad1.dpad_left){
                robot.mineralPush.setPosition(0.2);
            }else if(gamepad1.dpad_right){
                robot.mineralPush.setPosition(0.8);
            }
            else if(gamepad1.dpad_up){
                robot.mineralPush.setPosition(0);
            }

            // mineral pass servos, game pad 1 right bumper.
            if(gamepad1.right_bumper){
                robot.mineralPassLeft.setPosition(robot.mineralPassLeftOpen);
                robot.mineralPassRight.setPosition(robot.mineralPassRightOpen);
            }else {
                robot.mineralPassLeft.setPosition(robot.mineralPassLeftClose);
                robot.mineralPassRight.setPosition(robot.mineralPassRightClose);
            }


            //Mineral graber control. Game pad 2, triggers.
            if (gamepad2.left_trigger < 0 && gamepad2.right_trigger < 0) {
                robot.setMineralGrabServos(0);
            }
            if (gamepad2.right_trigger > 0) {
                robot.setMineralGrabServos(0.8);
            } else if (gamepad2.left_trigger > 0) {
                robot.setMineralGrabServos(0.2);
            } else {
                robot.setMineralGrabServos(0);
            }


            // Game Pad 1, triggers. Extrusions control.
            mineralSenderPosition = robot.mineralSend.getCurrentPosition();
            if (gamepad1.right_trigger > 0.1 && mineralSenderPosition<senderOpenEncoderLimitPoint )
            {   // Right trigger pushed, open extrusions.
                robot.mineralSend.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1 && mineralSenderPosition> senderCloseEncoderLimitPoint)
            {   // Left trigger pushed, close extrusions.
                robot.mineralSend.setPower(-gamepad1.left_trigger);
            } else {
                robot.mineralSend.setPower(0);
            }


            // Mineral blocker control. Game pad 2 bumper.
            if (gamepad2.left_bumper) {
                robot.blockMineralServo.setPosition(robot.dontBlock);   // Set Mode of servo to not block minerals.
            } else {
                robot.blockMineralServo.setPosition(robot.block);   //Set Mode of servo to block minerals.
            }


            // Game pad 1, buttons. Mineral box control.
            if (gamepad1.y) {
                robot.mineralBoxServo.setPosition(robot.mineralBoxServoOpen);
            } else if (gamepad1.a) {
                robot.mineralBoxServo.setPosition(robot.mineralBoxServoClose);
            }


            // Mineral push Control. Game pad 2, right stick.
            pushPosition = robot.push.getCurrentPosition();
            if (-gamepad2.right_stick_y > 0.2 && pushPosition<pushOpenEncoderLimitPoint ) {
                robot.push.setPower(1);
                robot.blockMineralServo.setPosition(robot.block);
            } else if (-gamepad2.right_stick_y < -0.2  && pushPosition>pushCloseEncoderLimitPoint) {
                robot.push.setPower(-1);
            } else{
                robot.push.setPower(0);
            }


            // Mineral lift Control. Game pad 2, left stick.
            liftPosition = robot.lift.getCurrentPosition();
            if (-gamepad2.left_stick_y < 0 && liftPosition<liftOpenEncoderLimitPoint) {
                robot.lift.setPower(-gamepad2.left_stick_y );
            } else if (-gamepad2.left_stick_y > 0 && liftPosition>liftCloseEncoderLimitPoint ){
                robot.lift.setPower(-gamepad2.left_stick_y );
            } else {
                robot.lift.setPower(0);
            }


            // Climb Control. Game pad 2, up down buttons.
            climbPosition = robot.climbMotor.getCurrentPosition();
            if (gamepad2.dpad_up && gamepad2.dpad_down) {
                robot.climbMotor.setPower(0);
            } else if (gamepad2.dpad_up && climbPosition<robot.climbOpenPosition) {
                robot.climbMotor.setPower(1);
            } else if ((gamepad2.dpad_down) && climbPosition> 0 ){
                robot.climbMotor.setPower(-1);
            } else {
                robot.climbMotor.setPower(0);
            }


            telemetry.addData("Encoder lift", robot.lift.getCurrentPosition());
            telemetry.addData("Encoder push", robot.push.getCurrentPosition());
            telemetry.addData("Encoder climb", robot.climbMotor.getCurrentPosition());
            telemetry.addData("Encoder Sender", robot.mineralSend.getCurrentPosition());
            telemetry.update();
        }
    }
}

