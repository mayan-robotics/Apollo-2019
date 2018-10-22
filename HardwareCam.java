package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * Apollo 2019
 * This class is used to define all the specific hardware for Apollo's robot.
 */

public class HardwareCam {

        /* Public OpMode members. */

        public WebcamName cam = null;

        //BNO055IMU imu;

        //Declaration of the drive motor types.
        public enum DRIVE_MOTOR_TYPES {
            LEFT,
            RIGHT,
            SIDE_WAYS,
            DIAGONAL_RIGHT,
            DIAGONAL_LEFT,
            ALL
        }

        static final double block = 0.5;
        static final double dontBlock = 0;

         /* local OpMode members. */
        HardwareMap hwMap  =  null;

        /* Constructor */
        public HardwareCam(){

        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize Motors
            //driveLeftFront  = hwMap.get(DcMotor.class, "dlf");
            //driveRightFront  = hwMap.get(DcMotor.class, "drf");
            //driveLeftBack  = hwMap.get(DcMotor.class, "dlb");
            //driveRightBack  = hwMap.get(DcMotor.class, "drb");
            ////lift  = hwMap.get(DcMotor.class, "l");
            //mainGraber  = hwMap.get(DcMotor.class, "m");
            //graberPusher  = hwMap.get(DcMotor.class, "p");
            ////mineralsUp  = hwMap.get(DcMotor.class, "mt");
            cam  = hwMap.get(WebcamName.class, "c");
            // Set all motors to zero power
            //driveRightFront.setPower(0);
            //driveLeftFront.setPower(0);
            //setDriveMotorsPower(0, DRIVE_MOTOR_TYPES.ALL);
            //mainGraber.setPower(0);
            //graberPusher.setPower(0);
            ////graberServo.setPosition(block);
            ////graberServo.setPosition(0);
            ////mineralsUp.setPower(0);
            //driveLeftFront.setDirection(DcMotor.Direction.REVERSE);
            //driveRightFront.setDirection(DcMotor.Direction.REVERSE);


            // Set all motors to run without encoders.
            //driveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //driveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //setDriveMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // May want to use RUN_USING_ENCODERS if encoders are installed.


            // Define and initialize ALL installed servos.

        }


}
