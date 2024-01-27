/*
 * Copyright (c) 2020 OpenFTC Team
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.ColorSpace;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class RedPipeline extends LinearOpMode
{
    OpenCvInternalCamera Cam;
    WilburR pipeline;

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new WilburR();
        Cam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        Cam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        Cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Cam.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
                //320,240
                //was SIDEWAYS_LEFT
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class WilburR extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum SkystonePosition
        {
            LEFT,
            CENTER,
            RIGHT
        }
        Mat rgb = new Mat();
        Mat leftcrop;
        Mat rightcrop;
        Mat midcrop;
        double leftavgfin;
        double rightavgfin;
        double midavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(128.0,0.0,0.0);

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SkystonePosition position = SkystonePosition.CENTER;

        @Override
        public Mat processFrame(Mat input)
        {

           // Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_BayerRG2BGRA);
            Imgproc.cvtColor(input,rgb,Imgproc.COLOR_BGR2RGB);
            //Imgproc.cvtColor(input, ColorSpace.Rgb,Imgproc.COLOR_BayerRG2BGR );

            Rect leftRect = new Rect(1,1,158,200);
            Rect midRect = new Rect(160,1,318,359);
            Rect rightRect = new Rect(480,1,158,200);
            input.copyTo(outPut);
            Imgproc.rectangle(outPut,leftRect,rectColor,2);
            Imgproc.rectangle(outPut,midRect,rectColor,2);
            Imgproc.rectangle(outPut,rightRect,rectColor,2);

            leftcrop = rgb.submat(leftRect);
            midcrop = rgb.submat(midRect);
            rightcrop = rgb.submat(rightRect);

            Core.extractChannel(leftcrop,leftcrop,1);
            Core.extractChannel(rightcrop,rightcrop,1);
            Core.extractChannel(midcrop,midcrop,1);

            Scalar leftavg = Core.mean(leftcrop);
            Scalar rightavg = Core.mean(rightcrop);
            Scalar midavg = Core.mean(midcrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];
            midavgfin = midavg.val[0];

            if((leftavgfin > rightavgfin) && (leftavgfin > midavgfin)) // Was it from region 1?
            {
                position = SkystonePosition.LEFT; // Record our analysis
            }
            else if((midavgfin > rightavgfin) && (midavgfin > leftavgfin)) // Was it from region 2?
            {
                position = SkystonePosition.CENTER; // Record our analysis
            }
            else if((rightavgfin > leftavgfin ) && (rightavgfin > midavgfin)) // Was it from region 3?
            {
                position = SkystonePosition.RIGHT; // Record our analysis
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public SkystonePosition getAnalysis()
        {
            return position;
        }
    }
}