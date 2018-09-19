package org.firstinspires.ftc.teamcode;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.KeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Point;

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
 * A nice demo class for using OpenCVPipeline. This one also demonstrates how to use OpenCV to threshold
 * for a certain color (blue) and find contours of objects of that color, which is very common in
 * robotics OpenCV applications.
 */

public class MineralVision extends OpenCVPipeline {
    private boolean showContours = true;
    GripGoldMineral gripGold   = new GripGoldMineral();
    GripSilverMineral gripSilver   = new GripSilverMineral();
    // To keep it such that we don't have to instantiate a new Mat every call to processFrame,
    // we declare the Mats up here and reuse them. This is easier on the garbage collector.
    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();
    private MatOfKeyPoint detectedTargets;
    private Rect targetRect = null;
    private  Rect[] targetRects = new Rect[2];
    private boolean findGoldMineral = false;
    private boolean findSilverMineral = false;

    // this is just here so we can expose it later thru getContours.
    //private List<MatOfPoint> contours = new ArrayList<>();

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }
    //public synchronized List<MatOfPoint> getContours() {
    //    return contours;
    //}

    public synchronized boolean goldMineralFound () {
        return (findGoldMineral);
    }
    public synchronized boolean silverMineralFound () {
        return (findSilverMineral);
    }

    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

        gripGold.process(rgba);
        detectedTargets = gripGold.findBlobsOutput();
        if ((detectedTargets != null) && (showContours)) {
            KeyPoint[] targets = detectedTargets.toArray();

            if ((targets.length > 1) && (showContours)) {
                double radius = targets[0].size / 2;
                targetRect = new Rect(
                        (int) (targets[0].pt.x - radius), (int) (targets[0].pt.y - radius),
                        (int) targets[0].size, (int) targets[0].size);
                targetRects[0] = targetRect;
                findGoldMineral = true;
            } else {
                findGoldMineral = false;
            }
            detectedTargets.release();
        } else {
            findGoldMineral = false;
        }

        gripSilver.process(rgba);
        detectedTargets = gripSilver.findBlobsOutput();
        if ((detectedTargets != null) && (showContours)) {
            KeyPoint[] targets = detectedTargets.toArray();

            if ((targets.length > 1) && (showContours)) {
                double radius = targets[0].size / 2;
                targetRect = new Rect(
                        (int) (targets[0].pt.x - radius), (int) (targets[0].pt.y - radius),
                        (int) targets[0].size, (int) targets[0].size);
                targetRects[1] = targetRect;
                synchronized (rgba) {
                    Imgproc.rectangle(
                            rgba, new Point(targetRect.x, targetRect.y),
                            new Point(targetRect.x + targetRect.width, targetRect.y + targetRect.height),
                            new Scalar(0, 255, 0), 5);
                }
                findSilverMineral = true;
            } else {
                findSilverMineral = false;
            }
            detectedTargets.release();
        } else {
            findGoldMineral = false;
        }
        if (findGoldMineral == true) {
            synchronized (rgba) {
                Imgproc.rectangle(
                        rgba, new Point(targetRects[0].x, targetRects[0].y),
                        new Point(targetRects[0].x + targetRects[0].width, targetRects[0].y + targetRects[0].height),
                        new Scalar(255, 210, 0), 5);
            }
        }
        if (findSilverMineral == true) {
            synchronized (rgba) {
                Imgproc.rectangle(
                        rgba, new Point(targetRects[1].x, targetRects[1].y),
                        new Point(targetRects[1].x + targetRects[1].width, targetRects[1].y + targetRects[1].height),
                        new Scalar(0, 230, 255), 5);
            }
        }
        return rgba; // display the image seen by the camera
    }

    private void drawRectangles(Mat image, Rect[] detectedObjectRects, Scalar color, int thickness)
    {
        {
            if (detectedObjectRects != null)
            {
                for (Rect r: detectedObjectRects)
                {

                    Imgproc.rectangle(
                            image, new Point(r.x, r.y), new Point(r.x + r.width, r.y + r.height), color, thickness);
                }
            }
        }
    }

}
