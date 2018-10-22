package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


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

@Autonomous(name="Vision Cam Test")
public class VisionCam extends OpMode {
    private MineralVision vision;
    HardwareCam camera = new HardwareCam(); // use Apollo's hardware
    VuforiaLocalizer vuforia;


    public enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE
    }

    @Override
    public void init() {

        vision = new MineralVision();


        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance(),2);
        vision.setShowCountours(false);

        //vuforia.getFrameOnce(camera.cam.);

        //camera.cam.vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        //camera.cam.vision.enable();
        //vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        vision.enable();
    }

    @Override
    public void loop() {
        //GetGoldLocation();
        telemetry.update();
    }

    public void stop() {
        // stop the vision system
        vision.disable();
        //camera.cam.vision.disable();
    }

    //Function returns the location of the gold mineral.
    public GoldPosition GetGoldLocation(){
        List<MatOfPoint> contoursGold = new ArrayList<>();
        vision.setShowCountours(true);

        contoursGold.clear();
        vision.getGoldContours(contoursGold);

        if ((vision.goldMineralFound() == true) && (contoursGold != null)) {
            if (!contoursGold.isEmpty())  {
                if (contoursGold.size() >= 1) {
                    Rect GoldBoundingRect = Imgproc.boundingRect(contoursGold.get(0));
                    int goldXPosition = GoldBoundingRect.x;

                    if (goldXPosition < 450) {
                        telemetry.addData("Gold Position", "Left");
                        return GoldPosition.LEFT;
                    } else if (goldXPosition > 650) {
                        telemetry.addData("Gold Position", "Right");
                        return GoldPosition.RIGHT;
                    } else if (goldXPosition > 450 && goldXPosition < 650) {
                        telemetry.addData("Gold Position", "Middle");
                        return GoldPosition.MIDDLE;
                    }
                }
            }
        } else {
            telemetry.addData("Apollo", "did not find a gold mineral");
        }

        return null;}

}