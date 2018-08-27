package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Apollo Teleop driving.
 */

@TeleOp(name="Apollo Teleop", group="Apollo")
public class TeleopApollo extends OpMode {
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware
    static final double speedFactor = 0.5; // Decrease the speed

    @Override
    public void init() {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Apollo", "Ready");
    }

    @Override
    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double leftspeed = -gamepad1.left_stick_y;
        double rightspeed = -gamepad1.right_stick_y;

        robot.setDriveMotorsPower((leftspeed * speedFactor), HardwareApollo.DRIVE_MOTOR_TYPES.LEFT);
        robot.setDriveMotorsPower((rightspeed * speedFactor), HardwareApollo.DRIVE_MOTOR_TYPES.RIGHT);

    }

}
