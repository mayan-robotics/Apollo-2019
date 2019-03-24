package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

            if (gamepad1.y) {
                robot.mineralBoxServo.setPosition(robot.mineralBoxServoClose);
            } else if (gamepad1.a) {
                robot.mineralBoxServo.setPosition(robot.mineralBoxServoOpen);
            }else if(gamepad1.x){
                robot.mineralBoxServo.setPosition(1);
            }

            telemetry.addData("gamepad2.left_stick_y",-gamepad2.left_stick_y);
            telemetry.addData("Encoder lift", robot.lift.getCurrentPosition());
            telemetry.addData("Encoder push", robot.push.getCurrentPosition());
            telemetry.addData("Encoder climb", robot.climbMotor.getCurrentPosition());
            telemetry.addData("Encoder Sender", robot.mineralSend.getCurrentPosition());
            telemetry.update();
        }
    }
}

