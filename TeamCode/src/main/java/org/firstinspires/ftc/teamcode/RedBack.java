/*
 * Copyright (c) 2021 OpenFTC Team
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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
@Autonomous(name = "RED BACK", group = "Autonomous Main")
public class RedBack extends LinearOpMode
{
    OpenCvWebcam webcam;
    RedPipeline.WilburR pipeline;
    RedPipeline.WilburR.SkystonePosition snapshotAnalysis = RedPipeline.WilburR.SkystonePosition.CENTER;

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RedPipeline.WilburR(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

        }
        Webmap robot = new Webmap();
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(35, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
/**MID**/
        //middle forward
        TrajectorySequence Vietnam = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35, -34), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();
        //back,strafe
        TrajectorySequence Shrike = drive.trajectorySequenceBuilder(Vietnam.end())
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(35, -40), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(65, -40), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();
        /**left**/
        //right forward, strafe
        TrajectorySequence Canada = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35, -36), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(23, -36), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();
        //little back, strafe
        TrajectorySequence Goose = drive.trajectorySequenceBuilder(Canada.end())
                .lineToConstantHeading(new Vector2d(23, -37), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(65, -37), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();
        /**right**/
        //left forward, strafe
        TrajectorySequence America = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35, -36), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(48, -36), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();
        //little back, strafe
        TrajectorySequence Eagle = drive.trajectorySequenceBuilder(America.end())
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(48, -40), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(65, -40), SampleMecanumDrive.getVelocityConstraint(60, Math.toRadians(360), 14.75),
                        SampleMecanumDrive.getAccelerationConstraint(70))
                .build();




        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis)
        {
            case RIGHT:
            {
                telemetry.addLine("left");
                //close
                robot.CL(0);
                robot.CR(.5);
                sleep(500);
                //wrist down
                robot.W(0.01);
                sleep(1000);
                robot.UP(.03);
                sleep(500);
                //forward
                //strafe left
                drive.followTrajectorySequence(America);
                //open left claw
                robot.CR(0);
                //back alittle, strafe
                drive.followTrajectorySequence(Eagle);
                sleep(500);
                //drop yelow
                robot.CL(.5);
                break;
            }

            case LEFT:
            {
                telemetry.addLine("right");
                //close
                robot.CL(0);
                robot.CR(.5);
                sleep(500);
                //wrist down
                robot.W(0.01);
                sleep(1000);
                robot.UP(.03);
                sleep(500);
                //forward
                //turn right
                drive.followTrajectorySequence(Canada);
                //open left claw
                robot.CR(0);
                //back alittle, strafe
                drive.followTrajectorySequence(Goose);
                //drop yelow
                robot.CL(.5);
                break;
            }

            case CENTER:
            {
                telemetry.addLine("mid");
                //close
                robot.CL(0);
                robot.CR(.5);
                sleep(500);
                //wrist down
                robot.W(0.01);
                sleep(1000);
                robot.UP(.03);
                sleep(500);
                //forward
                drive.followTrajectorySequence(Vietnam);
                //open right claw
                robot.CR(0);
                sleep(500);
                robot.UP(.03);
                sleep(500);
                //back up
                //strafe left
                //go forward
                //drive.followTrajectorySequence(Shrike);
                //drop yellow
                robot.CL(.5);
                break;
            }
        }
        while (opModeIsActive())
        {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(5);
        }
    }
}