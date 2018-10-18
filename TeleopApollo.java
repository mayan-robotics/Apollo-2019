package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Apollo Teleop", group="Apollo")
public class TeleopApollo extends OpMode{
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    static final double speedFactor = 0.5; // Decrease the speed factor
    static final double limitPoint = 0.5;


    @Override
    public void init() {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Apollo", "Ready");
    }

    @Override
    public void loop() {
        double LeftX = gamepad1.left_stick_x;
        double LeftY = -gamepad1.left_stick_y;
        double RightX = gamepad1.right_stick_x;
        double RightY = gamepad1.right_stick_y;

        // All the drive modes controls
        if (LeftX < -limitPoint && (Math.abs(LeftY) > limitPoint) &&
                (RightX < -limitPoint && Math.abs(RightY) < limitPoint)){
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_LEFT);
            telemetry.addData("Drive", "Diagonal LEFT");
        }
        else if (LeftX > limitPoint && (Math.abs(LeftY) > limitPoint) &&
                (RightX > limitPoint && Math.abs(RightY) < limitPoint)){
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.DIAGONAL_RIGHT);
            telemetry.addData("Drive", "Diagonal Right");
        }
        else if ((Math.abs(LeftX) > Math.abs(LeftY) && Math.abs(LeftX) > limitPoint) &&
                (Math.abs(RightX) > Math.abs(RightY) && Math.abs(RightX)> limitPoint)){
            telemetry.addData("Drive", "Side Ways");
            robot.setDriveMotorsPower(LeftX*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.SIDE_WAYS);
        }
        else{
            robot.setDriveMotorsPower(LeftY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
            robot.setDriveMotorsPower(RightY*speedFactor, HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);
            telemetry.addData("Drive", "normal");
        }

        telemetry.update();
    }
    public void stop() {
        robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
    }

}