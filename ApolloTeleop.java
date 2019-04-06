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
@Disabled

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

            // Game Pad 1, triggers. Extrusions control.
            if (gamepad1.right_trigger > 0.1 )
            {   // Right trigger pushed, open extrusions.
                robot.mineralSend.setPower(gamepad1.right_trigger);

            } else if (gamepad1.left_trigger > 0.1)
            {
                // Left trigger pushed, close extrusions.
                robot.mineralSend.setPower(-0.25);
                //robot.mineralBoxServo.setPosition(robot.mineralBoxServoOpen);
            } else {
                robot.mineralSend.setPower(0);
                robot.mineralSend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        // Yotam helped
            }

            telemetry.addData("Encoder lift", robot.lift.getCurrentPosition());
            telemetry.addData("Encoder push", robot.push.getCurrentPosition());
            telemetry.addData("Encoder climb", robot.climbMotor.getCurrentPosition());
            telemetry.addData("Encoder Sender", robot.mineralSend.getCurrentPosition());
            telemetry.update();
        }
    }
}

