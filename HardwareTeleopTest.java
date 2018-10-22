package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Apollo 2019
 * This class is used to define all the specific hardware for Apollo's robot.
 */

public class HardwareTeleopTest {

        /* Public OpMode members. */
        public DcMotor  driveLeftFront = null;
        public DcMotor  driveLeftBack = null;
        public DcMotor  driveRightFront = null;
        public DcMotor  driveRightBack = null;
        public DcMotor  mainGraber = null;
        public DcMotor  graberPusher = null;
        public DcMotor  lift = null;
        public DcMotor  mineralsUp = null;
        public Servo    blockMineralServo = null;
        public Servo    mineralsDivider = null;

        BNO055IMU imu;

        //Declaration of the drive motor types.
        public enum DRIVE_MOTOR_TYPES {
            LEFT,
            RIGHT,
            SIDE_WAYS,
            DIAGONAL_RIGHT,
            DIAGONAL_LEFT,
            ALL
        }

        static final double block = 0;
        static final double dontBlock = 0.2;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER * 3.1415);

         /* local OpMode members. */
        HardwareMap hwMap  =  null;

        /* Constructor */
        public HardwareTeleopTest(){

        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize Motors
            driveLeftFront  = hwMap.get(DcMotor.class, "dlf");
            driveRightFront  = hwMap.get(DcMotor.class, "drf");
            driveLeftBack  = hwMap.get(DcMotor.class, "dlb");
            driveRightBack  = hwMap.get(DcMotor.class, "drb");
            //lift  = hwMap.get(DcMotor.class, "l");
            mainGraber  = hwMap.get(DcMotor.class, "m");
            graberPusher  = hwMap.get(DcMotor.class, "gp");
            //mineralsUp  = hwMap.get(DcMotor.class, "mt");
            blockMineralServo  = hwMap.get(Servo.class, "bs");
            mineralsDivider  = hwMap.get(Servo.class, "md");
            mineralsDivider.setPosition(0.5);
            // Set all motors to zero power
            //driveRightFront.setPower(0);
            //driveLeftFront.setPower(0);
            setDriveMotorsPower(0, DRIVE_MOTOR_TYPES.ALL);
            mainGraber.setPower(0);
            graberPusher.setPower(0);
            blockMineralServo.setPosition(block);
            //graberServo.setPosition(0);
            //mineralsUp.setPower(0);
            driveLeftFront.setDirection(DcMotor.Direction.REVERSE);
            driveRightFront.setDirection(DcMotor.Direction.REVERSE);

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);


            // Set all motors to run without encoders.
            //driveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //driveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            setDriveMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // May want to use RUN_USING_ENCODERS if encoders are installed.


            // Define and initialize ALL installed servos.

        }


    //Function to set the power to all the drive motors.
    public void setDriveMotorsPower(double power, DRIVE_MOTOR_TYPES driverMotorType){
        switch (driverMotorType){
            case LEFT:
                driveLeftFront.setPower(power);
                driveLeftBack.setPower(power);
                break;
            case RIGHT:
                driveRightFront.setPower(power);
                driveRightBack.setPower(power);
                break;
            case SIDE_WAYS:
                driveLeftBack.setPower(-power);
                driveLeftFront.setPower(power);
                driveRightBack.setPower(power);
                driveRightFront.setPower(-power);
                break;
            case DIAGONAL_LEFT:
                driveRightFront.setPower(power);
                driveLeftBack.setPower(power);
                break;
            case DIAGONAL_RIGHT:
                driveLeftFront.setPower(power);
                driveRightBack.setPower(power);
                break;

            case ALL:
            default:
                driveLeftFront.setPower(power);
                driveLeftBack.setPower(power);
                driveRightFront.setPower(power);
                driveRightBack.setPower(power);
                break;

        }

    }

    //Function to set the position to all the drive motors.
    public void setDriveMotorsPosition(double Cm, DRIVE_MOTOR_TYPES driverMotorType){
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;
        switch (driverMotorType){
            case LEFT:
                newLeftFrontTarget = driveLeftFront.getCurrentPosition() + (int)(Cm*COUNTS_PER_CM);
                newLeftBackTarget = driveLeftBack.getCurrentPosition() + (int)(Cm*COUNTS_PER_CM);

                driveLeftFront.setTargetPosition(newLeftFrontTarget);
                driveLeftBack.setTargetPosition(newLeftBackTarget);
                break;
            case RIGHT:
                newRightFrontTarget = driveRightFront.getCurrentPosition() + (int)(Cm*COUNTS_PER_CM);
                newRightBackTarget = driveRightBack.getCurrentPosition() + (int)(Cm*COUNTS_PER_CM);

                driveRightFront.setTargetPosition(newRightFrontTarget);
                driveRightBack.setTargetPosition(newRightBackTarget);
                break;
            case SIDE_WAYS:
                newLeftFrontTarget = driveLeftFront.getCurrentPosition() - (int)(Cm*COUNTS_PER_CM);
                newLeftBackTarget = driveLeftBack.getCurrentPosition() + (int)(Cm*COUNTS_PER_CM);
                newRightFrontTarget = driveRightFront.getCurrentPosition() + (int)(Cm*COUNTS_PER_CM);
                newRightBackTarget = driveRightBack.getCurrentPosition() - (int)(Cm*COUNTS_PER_CM);

                driveLeftBack.setTargetPosition(newLeftFrontTarget);
                driveLeftFront.setTargetPosition(newLeftBackTarget);
                driveRightBack.setTargetPosition(newRightFrontTarget);
                driveRightFront.setTargetPosition(newRightBackTarget);
                break;
            case ALL:
            default:
                newLeftFrontTarget = driveLeftFront.getCurrentPosition() + (int)(Cm*COUNTS_PER_CM);
                newLeftBackTarget = driveLeftBack.getCurrentPosition() + (int)(Cm*COUNTS_PER_CM);
                newRightFrontTarget = driveRightFront.getCurrentPosition() + (int)(Cm*COUNTS_PER_CM);
                newRightBackTarget = driveRightBack.getCurrentPosition() + (int)(Cm*COUNTS_PER_CM);

                driveLeftFront.setTargetPosition(newLeftFrontTarget);
                driveLeftBack.setTargetPosition(newLeftBackTarget);
                driveRightFront.setTargetPosition(newRightFrontTarget);
                driveRightBack.setTargetPosition(newRightBackTarget);
                break;

        }

    }

    //Function to set the run mode for all the drive motors.
    public void setDriveMotorsMode(DcMotor.RunMode runMode) {
        driveLeftFront.setMode(runMode);
        driveLeftBack.setMode(runMode);
        driveRightFront.setMode(runMode);
        driveRightBack.setMode(runMode);

    }

    //Function to set the power to all the minerals motors.
    public void runAllMineralsMotor(double power) {
        mainGraber.setPower(power);
        graberPusher.setPower(power);

    }

    //Function to set the run mode for all the minerals motors.
    public void setModeAllMineralsMotor(DcMotor.RunMode runMode) {
        mainGraber.setMode(runMode);
        graberPusher.setMode(runMode);

    }


}
