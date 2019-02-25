package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.ArrayList;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * This is a sample opmode that demonstrates the use of an OpenCVPipeline with FTC code.
 * When the x button is pressed on controller one, the camera is set to show areas of the image
 * where a certain color is, in this case, blue.
 *
 * Additionally, the centers of the bounding rectangles of the contours are sent to telemetry.
 **/

/*
 * Vision Test.
 * This Class was made for checking and testing the vision is working properly.
 */

@Autonomous(name="Test: Vision Gold Position", group = "Test")

public class VisionGoldPositionTest extends OpMode {
    private MineralVision vision;

    public enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE,
        OUTOFRANGE
    }

    @Override
    public void init() {
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system
        vision.enable();
        telemetry.addData("Apollo", "Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        try {
            GetGoldLocation();
            telemetry.addData("Gold Mineral Position", getLocation());
            telemetry.update();
        }catch (Exception e){
            telemetry.addData("ERROR","error");
            telemetry.update();
        }
    }

    public void stop() {
        // stop the vision system
        vision.disable();
    }

    //Function returns the location of the gold mineral.
    public GoldPosition GetGoldLocation(){
        try {
        List<MatOfPoint> contoursGold = new ArrayList<>();
        vision.setShowCountours(true);

        contoursGold.clear();
        vision.getGoldContours(contoursGold);


            if ((vision.goldMineralFound() == true) && (contoursGold != null)) {
                if (!contoursGold.isEmpty()) {
                    if (contoursGold.size() >= 1) {
                        if (Imgproc.boundingRect(contoursGold.get(0)) != null) {
                            Rect GoldBoundingRect = Imgproc.boundingRect(contoursGold.get(0));
                            //int goldXPosition = GoldBoundingRect.x;
                            int goldYPosition = GoldBoundingRect.y;

                            //telemetry.addData("Gold Position X", goldXPosition);
                            telemetry.addData("Gold Position Y", goldYPosition);

                            if (goldYPosition < 300) {
                                telemetry.addData("Gold Position", "Left");
                                return GoldPosition.LEFT;
                            } else if (goldYPosition > 300) {
                                telemetry.addData("Gold Position", "Right");
                                return GoldPosition.RIGHT;
                            } //else if (goldYPosition > 450 && goldYPosition < 650) {
                              //  telemetry.addData("Gold Position", "Middle");
                              //  return GoldPosition.MIDDLE;
                            //}
                        }
                    }
                }
            } else {
                telemetry.addData("Apollo", "did not find a gold mineral");
                return GoldPosition.OUTOFRANGE;
            }
        }catch (Exception e){
            telemetry.addData("Vision", "Error");
            telemetry.update();
        }

        return null;}

    // Function converts the location of the gold mineral on the camera
    // to the real location of the gold mineral compare to the silver minerals.
    public GoldPosition getLocation() {
        switch ((GetGoldLocation())){
            case LEFT:
                return GoldPosition.LEFT;
            case RIGHT:
                return GoldPosition.MIDDLE;
            case MIDDLE:
                return GoldPosition.MIDDLE;
            case OUTOFRANGE:
                return GoldPosition.RIGHT;
        }
        return null;
    }
}