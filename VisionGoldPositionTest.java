package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

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
@Autonomous(name="Vision Gold Position Test")
public class VisionGoldPositionTest extends OpMode {
    private MineralVision vision;

    @Override
    public void init() {
        vision = new MineralVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        vision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        vision.setShowCountours(false);
        // start the vision system
        vision.enable();
    }

    @Override
    public void loop() {
        vision.setShowCountours(true);
        //blueVision.setShowCountours(true);

        List<MatOfPoint> goldContours = vision.getGoldContours();
        List<MatOfPoint> silverContours = vision.getSilverContours();

        if(vision.goldMineralFound()== true && goldContours.size()  >=1 ){
            Rect GoldBoundingRect = Imgproc.boundingRect(goldContours.get(0));
            int  goldXPosition = GoldBoundingRect.x;

            telemetry.addData("Apollo","found a gold mineral");
            telemetry.addData("Number", goldContours.size());
            telemetry.addData("X", goldXPosition);

            if (goldXPosition<150){
                telemetry.addData("Gold Position","Left");
            }else if(goldXPosition > 1000){
                telemetry.addData("Gold Position","Right");
            }else if (goldXPosition > 450 && goldXPosition < 650) {
                telemetry.addData("Gold Position","Middle");
            }
        }else{
            telemetry.addData("Apollo","did not find a gold mineral");
        }
        if(vision.silverMineralFound()== true){
            telemetry.addData("Apollo","found a silver mineral");
        }else{
            telemetry.addData("Apollo","did not find a silver mineral");
        }
        telemetry.update();
    }


    public void stop() {
        // stop the vision system
        vision.disable();
    }
}