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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;

@TeleOp
public class AprilTagDemo extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();
    static final double FEET_PER_METER = 3.28084;
    //TODO: tweak the april tag distance formulas.
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!

    double fx = 822.317;
    double fy = 822.317;
    double cx = 319.495;
    double cy = 242.502;

    //double fx = 1417.61085783;
    //double fx = 578.272; //old
    //double fy = 1417.61085783;
    //double fy = 578.272; //old
    //double cx = 691.081281972;
    //double cx = 402.145; //old
    //double cy = 334.496879075;
    //double cy = 221.506; //old

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftFrontDrive = null;
    //private DcMotor rightFrontDrive = null;
    //private DcMotor leftBackDrive = null;
    //private DcMotor rightBackDrive = null;
    @Override
    public void runOpMode()
    {
        //leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        //rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        //leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        //rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

        dashboard = FtcDashboard.getInstance();
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 40);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        waitForStart();

        telemetry.setMsTransmissionInterval(50);



        while (opModeIsActive())
        {
            //double leftPower;
            //double rightPower;
            //double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;
            //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            //leftFrontDrive.setPower(leftPower/2);
            //leftBackDrive.setPower(leftPower/2);
            //rightFrontDrive.setPower(rightPower/2);
            //rightBackDrive.setPower(rightPower/2);

            packet = new TelemetryPacket();

            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if(detections != null)
            {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                        packet.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
                        packet.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
                        packet.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
                        packet.addLine(String.format("Red Pose: " + getPoseFromAprilTagRed(detections).toString()));
                        packet.addLine(String.format("Blue Pose: " + getPoseFromAprilTagBlue(detections).toString()));
                    }
                }
            }
                dashboard.sendTelemetryPacket(packet);
                telemetry.update();
            }
            //sleep(20);
        }
    public double getCameraZDistanceFromAprilTag(int id, ArrayList<AprilTagDetection> detections) {
        double distance = 0.00;
        for(AprilTagDetection detection : detections) {
            if (detection.id == id) {
                distance = ((detection.pose.z * FEET_PER_METER * 2.593) - 0.6827);
            }
        }
        return distance;
    }
    public double getRobotZDistanceFromAprilTagRed(ArrayList<AprilTagDetection> detections) {
        int id = 5;
        double distance = 0.00;
        for(AprilTagDetection detection : detections) {
            if (detection.id == id) {
                //accounts for ~7 inch distance from camera to center of robot, see dif in y intercept
                distance = ((detection.pose.z * FEET_PER_METER * 2.593) + 6.3173);
            }
        }
        return distance;
    }
    public double getRobotZDistanceFromAprilTagBlue(ArrayList<AprilTagDetection> detections) {
        int id = 2;
        double distance = 0.00;
        for(AprilTagDetection detection : detections) {
            if (detection.id == id) {
                //accounts for ~7 inch distance from camera to center of robot, see dif in y intercept
                distance = ((detection.pose.z * FEET_PER_METER * 2.593) + 6.3173);
            }
        }
        return distance;
    }
    public double getDistanceFromAprilTagX(int id, ArrayList<AprilTagDetection> detections) {
        double distance = 0.00;
        for(AprilTagDetection detection : detections) {
            if (detection.id == id) {
                distance = ((detection.pose.x * FEET_PER_METER * 2.374601021385254*1.603)-3.25);
            }
        }
        return distance;
    }
    public double getRobotRotationFromAprilTagRed(ArrayList<AprilTagDetection> detections) {
        int id = 5;
        double angle = 0.00;
        double x = getDistanceFromAprilTagX(id, detections);
        double z = getRobotZDistanceFromAprilTagRed(detections);
        for (AprilTagDetection detection : detections) {
            if (detection.id == id) {
                angle = (Math.atan(x/(z+7)));
            }
        }
        return angle;
    }
    public double getRobotRotationFromAprilTagBlue(ArrayList<AprilTagDetection> detections) {
        int id = 2;
        double angle = 0.00;
        double x = getDistanceFromAprilTagX(id, detections);
        double z = getRobotZDistanceFromAprilTagBlue(detections);
        for (AprilTagDetection detection : detections) {
            if (detection.id == id) {
                angle = (Math.atan(x/(z+7)));
            }
        }
        return angle;
    }
    public Pose2d getPoseFromAprilTagRed(ArrayList<AprilTagDetection> detections){
        Pose2d pose = new Pose2d(0,0,0);
        pose = new Pose2d(23-getDistanceFromAprilTagX(5, detections)+3.932,
                -95+getRobotZDistanceFromAprilTagRed(detections)-0.245,
                Math.toRadians(-90));
        return pose;
    }
    public Pose2d getPoseFromAprilTagBlue(ArrayList<AprilTagDetection> detections){
        Pose2d pose = new Pose2d(0,0,0);
        pose = new Pose2d(23-getDistanceFromAprilTagX(2, detections)-4.781,
                95-getRobotZDistanceFromAprilTagBlue(detections)+0.765,
                Math.toRadians(-90));
        return pose;
    }
    }

