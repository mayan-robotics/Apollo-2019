package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Teleop Apollo Terror", group="terror")

public class ApolloTeleopTerror extends LinearOpMode {

    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    private MineralVision vision;

    //private ElapsedTime runtime = new ElapsedTime();
    //private ElapsedTime time = new ElapsedTime();

    static double speedFactor = 1;  // Speed factor
    static double normalOrReversDrive = 1;  //

    static final double joyStickLimitPoints = 0.3;  // For better control


    boolean blockMineral = false;

    int liftencoderPosition = 0;
    int climbPosition = 0;
    int liftPosition = 0;
    int pushPosition = 0;

    double senderPosition;

    double positionServo = 0;


    @Override
    public void runOpMode() {
        //Hardware init
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Version", robot.Version);
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        waitForStart();
        robot.goldMineralLeftServo.setPosition(0.6);
        robot.goldMineralRightServo.setPosition(0.4);

        //runtime.reset();


        while (opModeIsActive()) {

            if(gamepad1.dpad_left){
                robot.mineralPush.setPosition(0.2);
            }else if(gamepad1.dpad_right){
                robot.mineralPush.setPosition(0.8);
            }
            else if(gamepad1.dpad_up){
                robot.mineralPush.setPosition(0);
            }

            if(gamepad1.right_bumper){
                robot.mineralPassLeft.setPosition(0.2);
                robot.mineralPassRight.setPosition(0.8);
            }else {
                robot.mineralPassLeft.setPosition(1);
                robot.mineralPassRight.setPosition(0);
            }


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
            //robot.mineralSend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad1.right_trigger > 0.1 )
            {   // Right trigger pushed, open extrusions.
                robot.mineralSend.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1)
            {   // Left trigger pushed, close extrusions.
                robot.mineralSend.setPower(-gamepad1.left_trigger);
            } else {
                robot.mineralSend.setPower(0);
                //robot.mineralSend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        // Yotam helped
            }


            // Mineral blocker control. Game pad 2 bumper.
            if (gamepad2.left_bumper) {
                robot.blockMineralServo.setPosition(robot.dontBlock);   // Set Mode of servo to not block minerals.
            } else {
                robot.blockMineralServo.setPosition(robot.block);   //Set Mode of servo to block minerals.
            }


            // Game pad 1, buttons. Mineral box control.
            if (gamepad1.y) {
                robot.mineralBoxServo.setPosition(0.4);
            } else if (gamepad1.a) {
                robot.mineralBoxServo.setPosition(1);
            }


            // Mineral push Control. Game pad 2, right stick.
            if (-gamepad2.right_stick_y > 0.2  ) {
                robot.push.setPower(1);
                robot.blockMineralServo.setPosition(robot.block);
            } else if (-gamepad2.right_stick_y < -0.2 ) {
                robot.push.setPower(-1);
            } else{
                robot.push.setPower(0);
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
                robot.climbMotor.setPower(0);
            } else if (gamepad2.dpad_up) {
                robot.climbMotor.setPower(1);
            } else if (gamepad2.dpad_down) {
                robot.climbMotor.setPower(-1);
            } else {
                robot.climbMotor.setPower(0);
            }
/*
            if(gamepad2.y){
                robot.goldMineralLeftServo.setPosition(robot.goldMineralServoCloseLeft);
                robot.goldMineralRightServo.setPosition(robot.goldMineralServoCloseRight);
            }else if(gamepad2.a){
                setGoldMineralServoOpenLeft();
                //robot.setGoldMineralServoOpenLeft();
                robot.setGoldMineralServoOpenRight();
                //robot.goldMineralLeftServo.setPosition(robot.goldMineralServoOpenLeft);
                //robot.goldMineralRightServo.setPosition(robot.goldMineralServoOpenRight);
            }else if(gamepad2.x){
                robot.goldMineralLeftServo.setPosition(robot.goldMineralServoCloseLeft);
            }else if(gamepad2.b){
                robot.goldMineralLeftServo.setPosition(robot.goldMineralServoCloseRight);
            }
*/
/*
            if(gamepad1.right_bumper) {
                if (opModeIsActive() && (runtime.seconds() < 0.5)) {
                    robot.mineralPassLeft.setPosition(robot.mineralPassLeftOpen);
                    robot.mineralPassRight.setPosition(robot.mineralPassRightOpen);
                } else if (opModeIsActive() && (runtime.seconds() < 1)) {
                    robot.mineralPassLeft.setPosition(robot.mineralPassLeftClose);
                    robot.mineralPassRight.setPosition(robot.mineralPassRightClose);
                } else {
                    runtime.reset();
                }
            }
*/

            //if (gamepad1.b){
            //    robot.setDriveMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //    robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //}


            telemetry.addData("Encoder lift", robot.lift.getCurrentPosition());
            telemetry.addData("Encoder push", robot.push.getCurrentPosition());
            telemetry.addData("Encoder climb", robot.climbMotor.getCurrentPosition());
            telemetry.addData("Encoder Sender", robot.mineralSend.getCurrentPosition());
            telemetry.addData("gyro", GetGyroAngle());


            telemetry.update();
        }

    }
    /*
    // Function to wait an amount of seconds.
    public void waitSeconds(double seconds)
    {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) { }
    }
    */
/*
    public void setGoldMineralServoOpenLeft(){
        //if (robot.goldMineralLeftServo.getPosition()!=0.5) {
            robot.goldMineralLeftServo.setPosition(0.2);
            telemetry.addData("test", "1");
            telemetry.update();
            runtime.reset();
            if (opModeIsActive() && (runtime.seconds() > 2)) { robot.goldMineralLeftServo.setPosition(0.5); }
            telemetry.addData("test", "2");
            telemetry.update();
            //else {
                //goldMineralLeftServo.setPosition(0.75);
                robot.goldMineralLeftServo.setPosition(0.5);
            //}
        //}
    }
*/
    public float GetGyroAngle(){
        Orientation angles =robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
    }
}

