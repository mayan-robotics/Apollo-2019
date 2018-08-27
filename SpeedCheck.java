package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="SpeedCheck", group="Apollo")
public class SpeedCheck extends LinearOpMode {
    HardwareApollo robot = new HardwareApollo(); // use Apollo's hardware

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Apollo", "Ready");
        waitForStart();

        robot.setDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setDriveMotorsPower(0.1, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);

        while (opModeIsActive()) {

        }
    }


}
