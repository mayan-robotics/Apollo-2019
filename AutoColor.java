package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

/**
 * Apollo Autonomus.
 */
@Autonomous(name="Color test", group="Apollo")
public class AutoColor extends LinearOpMode {
    HardwareTest robot = new HardwareTest(); // use Apollo's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double dividerMiddle = 0.5;
    static final double dividerLeft = 0.3;
    static final double dividerRight = 0.7;
    private MineralVision vision;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system
        vision.enable();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        vision.setShowCountours(true);
        robot.mineralsDivider.setPosition(dividerMiddle);

        while (opModeIsActive()) {

            if(vision.goldMineralFound()== true){
                telemetry.addData("Apollo","found a gold mineral");
                robot.mineralsDivider.setPosition(dividerLeft);
                //waitSeconds(1);
            }else{
                telemetry.addData("Apollo"," did not found a gold mineral");
                robot.mineralsDivider.setPosition(dividerRight);
            }
            telemetry.update();
        }
        // stop the vision system
        vision.disable();
    }

    public void waitSeconds(double seconds){
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {}
    }

}
