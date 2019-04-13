package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import static java.lang.Math.abs;


/**
 * Apollo 2019
 * This class is used to define all the specific hardware for Apollo's robot.
 */

public class HardwareApollo {
    /* Public OpMode members. */

    // Dc motors.
    public DcMotor  driveLeftFront = null;
    public DcMotor  driveLeftBack = null;
    public DcMotor  driveRightFront = null;
    public DcMotor  driveRightBack = null;
    public DcMotor  lift = null;
    public DcMotor  push = null;
    public DcMotor  mineralSend = null;
    public DcMotor  climbMotor = null;

    //Servos
    public Servo    blockMineralServo = null;
    public Servo    mineralBoxServo = null;
    public Servo    mineralGrab = null;


    BNO055IMU imu;
    DigitalChannel touchPusher ;

    //Declaration of the drive motor types.
    public enum DRIVE_MOTOR_TYPES {
        LEFT,
        RIGHT,
        SIDE_WAYS,
        DIAGONAL_RIGHT,
        DIAGONAL_LEFT,
        ALL
    }


    // Encoder
    static final double     COUNTS_PER_MOTOR_REV    = 280 ;     // PPR for NeverRest 40
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // Gear 2:1 , two motor cycle is to one wheel cycle
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);
    // Mineral Blocker Positions
    static final double block = 0;
    static final double dontBlock = 0.2;

    // Gold mineral X positions limits for camera.
    static final int MineralMiddleLimitLeft = 650 ;
    static final int MineralMiddleLimitRight = 600 ;

    // Gold Mineral servo position to be open or closed.
    static final double goldMineralServoCloseLeft = 0;
    static final double goldMineralServoCloseRight = 1;

    // climb open position.
    static final int climbOpenPosition = 37220;
    static final int climbLanderPosition = 25000;



    // mineral box positions.
    static final double mineralBoxServoOpen = 0.60 ; //A
    static final double mineralBoxServoClose = 0.2; //Y

    static final String Version = "1.4.13" ;

    /* local OpMode members. */
    HardwareMap hwMap  =  null;

    /* Constructor */
    public HardwareApollo(){    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize ALL Motors
        driveLeftFront  = hwMap.get(DcMotor.class, "dlf");
        driveLeftBack  = hwMap.get(DcMotor.class, "dlb");
        driveRightFront  = hwMap.get(DcMotor.class, "drf");
        driveRightBack  = hwMap.get(DcMotor.class, "drb");
        lift  = hwMap.get(DcMotor.class, "lift");
        push  = hwMap.get(DcMotor.class, "push");
        mineralSend  = hwMap.get(DcMotor.class, "send");
        climbMotor  = hwMap.get(DcMotor.class, "climb");

        // Define and initialize ALL servos.
        blockMineralServo  = hwMap.get(Servo.class, "block");
        mineralBoxServo  = hwMap.get(Servo.class, "mineralOpen");
        mineralGrab  = hwMap.get(Servo.class, "mineralGrab");
        //goldMineralLeftServo = hwMap.get(Servo.class, "goldMineralServo");

        // Imu
        imu = hwMap.get(BNO055IMU.class, "imu");

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
        imu.initialize(parameters);


        // Touch
        touchPusher = hwMap.get(DigitalChannel.class, "touchPush" );
        touchPusher.setMode(DigitalChannel.Mode.INPUT );

        // Set all motors directions.
        driveRightBack.setDirection(DcMotor.Direction.REVERSE);     // Reversed motor
        driveRightFront.setDirection(DcMotor.Direction.REVERSE);    // Reversed motor
        push.setDirection(DcMotor.Direction.REVERSE);               // Reversed motor
        lift.setDirection(DcMotor.Direction.REVERSE);               // Reversed motor
        mineralGrab.setDirection(Servo.Direction.REVERSE);     // Reversed motor
        //mineralSend.setDirection(DcMotor.Direction.REVERSE);        // Reversed motor
        climbMotor.setDirection(DcMotor.Direction.REVERSE);        // Reversed motor

        climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to zero power
        setDriveMotorsPower(0, DRIVE_MOTOR_TYPES.ALL);
        lift.setPower(0);
        push.setPower(0);
        climbMotor.setPower(0);
        mineralSend.setPower(0);
        mineralGrab.setPosition(0);

        // Rest all motors encoders.
        setAllMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run without encoders.
        setAllMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    // Function set positions for the servos.
    public void InitServoes(){
        // Set all servos positions
        blockMineralServo.setPosition(block);
        mineralBoxServo.setPosition(mineralBoxServoOpen);

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
                driveLeftFront.setPower(power*-0);
                driveRightBack.setPower(power*-0);
                break;
            case DIAGONAL_RIGHT:
                driveLeftFront.setPower(power);
                driveRightBack.setPower(power);
                driveLeftBack.setPower(power*-0.7);
                driveRightFront.setPower(power*-0.7);
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
        double inch;
        inch = cmToInch(Cm);

        switch (driverMotorType){
            case LEFT:
                newLeftFrontTarget = driveLeftFront.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);
                newLeftBackTarget = driveLeftBack.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);

                driveLeftFront.setTargetPosition(newLeftFrontTarget);
                driveLeftBack.setTargetPosition(newLeftBackTarget);
                break;
            case RIGHT:
                newRightFrontTarget = driveRightFront.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);
                newRightBackTarget = driveRightBack.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);

                driveRightFront.setTargetPosition(newRightFrontTarget);
                driveRightBack.setTargetPosition(newRightBackTarget);
                break;
            case SIDE_WAYS:

                newLeftBackTarget = driveLeftBack.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);
                newLeftFrontTarget = driveLeftFront.getCurrentPosition() - (int)(inch*COUNTS_PER_INCH);
                newRightBackTarget = driveRightBack.getCurrentPosition() - (int)(inch*COUNTS_PER_INCH);
                newRightFrontTarget = driveRightFront.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);

                driveLeftBack.setTargetPosition(newLeftFrontTarget);
                driveLeftFront.setTargetPosition(newLeftBackTarget);
                driveRightBack.setTargetPosition(newRightFrontTarget);
                driveRightFront.setTargetPosition(newRightBackTarget);
                break;
            case ALL:
            default:
                newLeftFrontTarget = driveLeftFront.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);
                newLeftBackTarget = driveLeftBack.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);
                newRightFrontTarget = driveRightFront.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);
                newRightBackTarget = driveRightBack.getCurrentPosition() + (int)(inch*COUNTS_PER_INCH);

                driveLeftFront.setTargetPosition(newLeftFrontTarget);
                driveLeftBack.setTargetPosition(newLeftBackTarget);
                driveRightFront.setTargetPosition(newRightFrontTarget);
                driveRightBack.setTargetPosition(newRightBackTarget);
                break;
        }
    }


    //Function to set the position to all the drive motors.
    public void setLiftMotorsPosition(double ticks) {
        int newLift;
        newLift = lift.getCurrentPosition() + (int)(ticks);
        lift.setTargetPosition(newLift);
    }

    //Function to set the run mode for all the motors.
    public void setAllMotorsMode(DcMotor.RunMode runMode) {
        setDriveMotorsMode(runMode);
        setNotDriveMotorsMode(runMode);
    }


    //Function to set the run mode for all the drive motors.
    public void setDriveMotorsMode(DcMotor.RunMode runMode) {
        driveLeftFront.setMode(runMode);
        driveLeftBack.setMode(runMode);
        driveRightFront.setMode(runMode);
        driveRightBack.setMode(runMode);
    }

    //Function to set the run mode for all the nun drive motors.
    public void setNotDriveMotorsMode(DcMotor.RunMode runMode) {
        mineralSend.setMode(runMode);
        climbMotor.setMode(runMode);
        lift.setMode(runMode);
        push.setMode(runMode);
    }

    //Function converts centimeters to inches.
    public double cmToInch(double cm){
        double inch;
        inch=cm*0.393700787;
        return inch;
    }

    // imu parameters init
    public void imuRestart(){
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
        imu.initialize(parameters);
    }

}
