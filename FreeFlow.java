package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

@TeleOp(name="FreeFlow", group="Connection")
@Disabled

public class FreeFlow extends RobotFunctions {
    //public Hardware_Connection robot = new Hardware_Connection();
    double DriveY = 0;
    double DriveX = 0;
    double Degree = 0;
// ASDFSHGLRKJSDBNDFKSJDGNVKDJ,SBFGNVULKEJRDFSNXGFLUCJKER,DSNX LFKJVCNFD .LCXKJ,NV FKJD
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro


    public void runOpMode() throws InterruptedException {
        /* Declare OpMode members. */

        robot.init(hardwareMap);

        telemetry.addData("Finish", "done init ");//hi
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            DriveY = gamepad1.left_stick_y;
            DriveX = gamepad1.left_stick_x;

            Degree = (Math.toDegrees(Math.atan2(DriveY,DriveX)))+90;
            if(!JoysStickInDeadZone()) {
                onHeading(0.5, -Degree, P_TURN_COEFF);
            }
            else{
                robot.setDriveMotorsPower(0, HardwareApollo.DRIVE_MOTOR_TYPES.ALL);
            }
            telemetry.addData("RSX", gamepad1.right_stick_x);
            telemetry.addData("RSY", gamepad1.right_stick_y);
            telemetry.addData("Degree", Degree);
            telemetry.update();
        }
    }//hi

    int marijuan = 0;

}
