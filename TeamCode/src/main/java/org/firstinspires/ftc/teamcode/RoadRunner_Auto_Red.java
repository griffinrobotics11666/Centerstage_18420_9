package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.ContoursPixelLocatorBLUE;
import org.firstinspires.ftc.teamcode.Pipelines.ContoursPixelLocatorRED;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "Auto Red")
public class RoadRunner_Auto_Red extends LinearOpMode {

    OpenCvWebcam webcam; //add other code to get the camera set up
    Hardwarerobot robot = new Hardwarerobot();
    private final ContoursPixelLocatorRED pipeline = new ContoursPixelLocatorRED(telemetry); //update this for new pipeline
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
    static double SLIDE_COUNTS_PER_INCH = 80;

    static final double HEADING_THRESHOLD = .5;
    static final double P_TURN_COEFF = 0.075;
    static final double P_DRIVE_COEFF = 0.05;

    int newTarget = 0;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT); //change the image size to 1920 x 1080
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
        webcam.closeCameraDevice();
        sleep(1000);
        conePosition = ContoursPixelLocatorRED.ConePosition.LEFT;


        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(22, 8, Math.toRadians(60)), Math.toRadians(60))
                .lineToSplineHeading(new Pose2d(22, -24, Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    goTo3();
                })
                .build();

        switch (conePosition) {


            case LEFT: {
                if (!isStopRequested())
                    drive.followTrajectorySequence(trajSeq);

                break;

            }


            case CENTER: {
                if (!isStopRequested())
                    break;
            }


            case RIGHT: {
                if (!isStopRequested())
                    break;
            }


        }


// The next segment is a simple example. Notice how all the trajectories start where the last one ended or the beginning point(startPose, traj1.end, ect).
// If you cant figure out command to use visit:https://learnroadrunner.com/trajectorybuilder-functions.html#forward-distance-double


//    Trajectory traj1 = drive.trajectoryBuilder(startPose)
//            .lineToLinearHeading(new Pose2d(46, 0, Math.toRadians(-90)))
//            .build();
//
//    Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//            .lineToLinearHeading(new Pose2d(46, -23, Math.toRadians(0)))
//            .build();
//
//    Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//            .lineToLinearHeading(new Pose2d(23, -23, Math.toRadians(180)))
//            .build();
//
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        drive.followTrajectory(traj1);
//
//        drive.followTrajectory(traj2);
//
//        drive.followTrajectory(traj3);


// It works but we have a lot of code. Using TrajectorySequence we can simplify it.
// Notice on the Dashboard, it is only shown as a single path unlike the previous example.
// Make sure to use .waitSeconds() NOT .wait() if you want a delay.

//    TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
//            .lineToLinearHeading(new Pose2d(46, 0, Math.toRadians(-90)))
//            .lineToLinearHeading(new Pose2d(46, -23, Math.toRadians(0)))
// Like this .waitSeconds(500)
//            .lineToLinearHeading(new Pose2d(23, -23, Math.toRadians(180)))
//            .strafeRight(23)
//            .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
//            .build();

//        waitForStart();

//        if (!isStopRequested())
//            drive.followTrajectorySequence(trajSeq);

//Here is a simple example with the webcam.
//
//    Trajectory traj1 = drive.trajectoryBuilder(startPose)
//            .lineToLinearHeading(new Pose2d(46, 0, Math.toRadians(-90)))
//            .build();
//
//    Trajectory traj2 = drive.trajectoryBuilder(startPose)
//            .lineToLinearHeading(new Pose2d(23, 0, Math.toRadians(-90)))
//            .build();
//
//    Trajectory traj3 = drive.trajectoryBuilder(startPose)
//            .splineToConstantHeading(new Vector2d(23,0), Math.toRadians(0))
//            .splineToConstantHeading(new Vector2d(23,-23), Math.toRadians(0))
//            .build();
//
//
//        waitForStart();
//
//        if (!isStopRequested())
//            getAnalysis = pipeline.getAnalysis();
//            sleep(1000);
//            switch (getAnalysis) {
//                case LEFT: { //one
//                    if (!isStopRequested())
//                        drive.followTrajectory(traj1);
//                    break;
//
//                }
//
//                case CENTER: { //two
//                    if (!isStopRequested())
//                        drive.followTrajectory(traj2);
//                   break;
//                }
//
//
//                case RIGHT: { //three
//                    if (!isStopRequested())
//                        drive.followTrajectory(traj3);
//                    break;
//                }
//
//            }
//
//




    }
    public void goTo0 () {
        double distance = 0;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.viperSlideLift.setTargetPosition(newTarget);
    }

    public void goTo3 () {
        double distance = 37;
        newTarget = (int) (distance * ARM_COUNTS_PER_INCH);
        robot.viperSlideLift.setTargetPosition(newTarget);
    }
}
