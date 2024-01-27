package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AprilTagDemo.FEET_PER_METER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.ContoursPixelLocatorRED;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(group = "Auto Red DIST TEST")
public class Auto_Red_Close_CorrectedDistancesTest extends LinearOpMode {

    double fx = 822.317;
    double fy = 822.317;
    double cx = 319.495;
    double cy = 242.502;
    double tagsize = 0.166;

    OpenCvWebcam webcam; //add other code to get the camera set up
    Hardwarerobot robot = new Hardwarerobot();
    private ContoursPixelLocatorRED pipeline = new ContoursPixelLocatorRED(telemetry); //update this for new pipeline
    AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
    ContoursPixelLocatorRED.ConePosition conePosition = ContoursPixelLocatorRED.ConePosition.LEFT; //change this

    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.5;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415) / 2.29; //find out actual number
    static final double STRAFE_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415) / 1.86;
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static double ARM_COUNTS_PER_INCH = 80; //Figure out right number //114.75

    static final double HEADING_THRESHOLD = .5;
    static final double P_TURN_COEFF = 0.075;
    static final double P_DRIVE_COEFF = 0.05;

    int newTarget = 0;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    ArrayList<AprilTagDetection> detections = new ArrayList<AprilTagDetection>();

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(-4.75, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT); //change the image size to 1920 x 1080
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        robot.viperSlideLift.setDirection(DcMotor.Direction.REVERSE);
        robot.viperSlideLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.viperSlideLift.setTargetPosition(0);
        robot.viperSlideLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.viperSlideLift.setPower(1);

        while (opModeInInit()) {
            telemetry.addData("Realtime analysis", pipeline.getPropPosition());
            telemetry.update();
            sleep(50);

        }
        conePosition = pipeline.getPropPosition();

        telemetry.addData("Snapshot post-START analysis", conePosition);
        telemetry.update();

        if (!isStopRequested())
            conePosition = pipeline.getPropPosition();
        //webcam.closeCameraDevice();

        webcam.setPipeline(aprilTagDetectionPipeline);

        detections = aprilTagDetectionPipeline.getDetectionsUpdate();

        /*
        int numFramesWithoutDetection = 0;

        // If we don't see any tags
        if (detections.size() == 0) {
            numFramesWithoutDetection++;

            // If we haven't seen a tag for a few frames, lower the decimation
            // so we can hopefully pick one up if we're e.g. far back
            if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
            }
        }
        // We do see tags!
        else {
            numFramesWithoutDetection = 0;

            // If the target is within 1 meter, turn on high decimation to
            // increase the frame rate
            if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
            }
        }
        */


        sleep(1000);
        conePosition = ContoursPixelLocatorRED.ConePosition.RIGHT;

        robot.auto.setPosition(robot.AUTO_CLOSED_POS);

        TrajectorySequence trajSeq_left = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(30, 2, Math.toRadians(180)))
                .addDisplacementMarker(30, () -> robot.auto.setPosition(robot.AUTO_OPEN_POS))
                .addDisplacementMarker(this::deposit)
                .lineToSplineHeading(new Pose2d(30, -40, Math.toRadians(270)))
                .forward(5)
                .waitSeconds(.5)
                .addDisplacementMarker(this::retract)
                .back(5)
                .lineToSplineHeading(new Pose2d(5, -40, Math.toRadians(270)))
                .build();

        TrajectorySequence trajSeq_center = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(40, -2, Math.toRadians(180)))
                .addDisplacementMarker(40, () -> robot.auto.setPosition(robot.AUTO_OPEN_POS))
                .addDisplacementMarker(this::deposit)
                .lineToSplineHeading(new Pose2d(25, -40, Math.toRadians(270)))
                .waitSeconds(.5)
                .forward(5)
                .waitSeconds(.5)
                .addDisplacementMarker(this::retract)
                .back(5)
                .lineToSplineHeading(new Pose2d(5, -40, Math.toRadians(270)))
                .build();

        TrajectorySequence trajSeq_right = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(30, -2, Math.toRadians(0)))
                .addDisplacementMarker(30, () -> robot.auto.setPosition(robot.AUTO_OPEN_POS))
                .strafeLeft(2)
                .back(5)
                .addDisplacementMarker(this::deposit)
                .lineToSplineHeading(new Pose2d(17, -40, Math.toRadians(270)))
                .waitSeconds(.25)
                .forward(5)
                .addDisplacementMarker(this::retract)
                .back(5)
                .lineToSplineHeading(new Pose2d(5, -40, Math.toRadians(270)))
                .build();

        TrajectorySequence testTrajectory1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(24, 0, Math.toRadians(-90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(24,-72, Math.toRadians(-90)))
                .waitSeconds(5)
                .addTemporalMarker(()->detections = aprilTagDetectionPipeline.getDetectionsUpdate())
                .addTemporalMarker(()->telemetry.addData("x: ",getDistanceFromAprilTagX(5,detections)))
                .addTemporalMarker(()->telemetry.addData("z: ", getDistanceFromAprilTagZ(5,detections)))
                .addTemporalMarker(()->telemetry.update())
                .waitSeconds(5)
                .addTemporalMarker(()->drive.setPoseEstimate(
                        new Pose2d(23+getDistanceFromAprilTagX(5,detections),
                                -95+getDistanceFromAprilTagZ(5,detections),
                                Math.toRadians(-90))
                ))
                .strafeTo(new Vector2d(24,-84))
                .waitSeconds(10)
                .build();



            switch (conePosition) {


                case LEFT:
                    if (!isStopRequested())
                        drive.followTrajectorySequence(trajSeq_left);
                    break;


                case CENTER:
                    if (!isStopRequested())
                        drive.followTrajectorySequence(trajSeq_center);
                    break;


                case RIGHT:
                    if (!isStopRequested())
                        //drive.followTrajectorySequence(trajSeq_right);
                        drive.followTrajectorySequence(testTrajectory1);



                    break;


            }


    }
    public void deposit() {
        robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_DEPOSIT_POS);
    }

    public void retract(){
        robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_STORE_POS);
    }
    public double getDistanceFromAprilTagZ(int id, ArrayList<AprilTagDetection> detections) {
        double distance = 0.00;
        for(AprilTagDetection detection : detections) {
            if (detection.id == id) {
                distance = ((detection.pose.z * FEET_PER_METER * 2.593) - 0.6827);
            }
        }
        return distance;
    }
    public double getDistanceFromAprilTagX(int id, ArrayList<AprilTagDetection> detections) {
        double distance = 0.00;
        for(AprilTagDetection detection : detections) {
            if (detection.id == id) {
                distance = ((detection.pose.x * FEET_PER_METER * 1.395)-3.25);
            }
        }
        return distance;
    }


}

