package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DriverControl2024.outTakeClosed;
import static org.firstinspires.ftc.teamcode.DriverControl2024.outTakeMiddle;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.ContoursPixelLocatorRED;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "Auto Red CLOSE")
public class Auto_Red_Close_2p2 extends LinearOpMode {

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

    static final double HEADING_THRESHOLD = .5;
    static final double P_TURN_COEFF = 0.075;
    static final double P_DRIVE_COEFF = 0.05;

    int newTarget = 0;

    @Override
    public void runOpMode()  {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
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
        //conePosition = ContoursPixelLocatorBLUE.ConePosition.RIGHT;
        robot.auto.setPosition(robot.AUTO_CLOSED_POS);

        TrajectorySequence trajSeq_left = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(30,2, Math.toRadians(180)))
                .addDisplacementMarker(30, () -> robot.auto.setPosition(robot.AUTO_OPEN_POS))
                .addDisplacementMarker(this::raise)
                .lineToSplineHeading(new Pose2d(32, -35, Math.toRadians(270)))
                .addDisplacementMarker(() -> robot.auto.setPosition(robot.AUTO_CLOSED_POS))
                .waitSeconds(.5)
                .addTemporalMarker(() -> this.drop())
                .waitSeconds(.5)
                .addTemporalMarker(() -> this.close())
                .addDisplacementMarker( this::retract)
                .back(5)
                // sussy wussy stuff here
                .lineToSplineHeading(new Pose2d(2, -5, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(2, 48, Math.toRadians(270)))
                //.lineToSplineHeading(new Pose2d(24, 48, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(24, 72, Math.toRadians(270)))
                //suck up pixels here
                .addTemporalMarker(() -> this.eat())
                .waitSeconds(.5)
                .addTemporalMarker(() -> this.moreEating())
                .waitSeconds(.2)
                .forward(6)
                .waitSeconds(.2)
                .back(5)
                .addTemporalMarker(() -> this.eat())
                .waitSeconds(.2)
                .addTemporalMarker(() -> this.moreEating())
                //does it suck
                .lineToSplineHeading(new Pose2d(2, 48, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(2, -5, Math.toRadians(270)))
                .addTemporalMarker(() -> this.raise())
                .addTemporalMarker(() -> this.stopEating())
                .lineToSplineHeading(new Pose2d(25, -35, Math.toRadians(270)))
                .waitSeconds(.5)
                .addTemporalMarker(() -> this.drop())
                .waitSeconds(.5)
                .addDisplacementMarker( this::retract)
                .back(5)
                .lineToSplineHeading(new Pose2d(5, -40, Math.toRadians(270)))
                .build();

        TrajectorySequence trajSeq_center = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(40,-5, Math.toRadians(180)))
                .addDisplacementMarker(40, () -> robot.auto.setPosition(robot.AUTO_OPEN_POS))
                .addDisplacementMarker(this::deposit)
                .lineToSplineHeading(new Pose2d(25, -35, Math.toRadians(270)))
                .addDisplacementMarker(() -> robot.auto.setPosition(robot.AUTO_CLOSED_POS))
                .waitSeconds(.5)
                .addTemporalMarker(() -> this.drop())
                .waitSeconds(.5)
                .addTemporalMarker(() -> this.close())
                .addDisplacementMarker( this::retract)
                .back(5)
                // sussy wussy stuff here
                .lineToSplineHeading(new Pose2d(2, -5, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(2, 48, Math.toRadians(270)))
                //.lineToSplineHeading(new Pose2d(24, 48, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(24, 72, Math.toRadians(270)))
                //suck up pixels here
                .addTemporalMarker(() -> this.eat())
                .waitSeconds(.5)
                .addTemporalMarker(() -> this.moreEating())
                .waitSeconds(.2)
                .forward(6)
                .waitSeconds(.2)
                .back(5)
                .addTemporalMarker(() -> this.eat())
                .waitSeconds(.2)
                .addTemporalMarker(() -> this.moreEating())
                //does it suck
                .lineToSplineHeading(new Pose2d(2, 48, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(2, -5, Math.toRadians(270)))
                .addTemporalMarker(() -> this.raise())
                .addTemporalMarker(() -> this.stopEating())
                .lineToSplineHeading(new Pose2d(25, -35, Math.toRadians(270)))
                .waitSeconds(.5)
                .addTemporalMarker(() -> this.drop())
                .waitSeconds(.5)
                .addDisplacementMarker( this::retract)
                .back(5)
                .lineToSplineHeading(new Pose2d(5, -40, Math.toRadians(270)))
                .build();

        TrajectorySequence trajSeq_right = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(30,-2, Math.toRadians(0)))
                .addDisplacementMarker( 30, () -> robot.auto.setPosition(robot.AUTO_OPEN_POS))
                .strafeLeft(2)
                .back(5)
                .addDisplacementMarker(this::raise)
                .lineToSplineHeading(new Pose2d(19, -35, Math.toRadians(270)))
                .addDisplacementMarker(() -> robot.auto.setPosition(robot.AUTO_CLOSED_POS))
                .waitSeconds(.5)
                .addTemporalMarker(() -> this.drop())
                .waitSeconds(.5)
                .addTemporalMarker(() -> this.close())
                .addDisplacementMarker(this::retract)
                .back(5)
                // sussy wussy stuff here
                .lineToSplineHeading(new Pose2d(2, -5, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(2, 48, Math.toRadians(270)))
                //.lineToSplineHeading(new Pose2d(24, 48, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(24, 72, Math.toRadians(270)))
                //suck up pixels here
                .addTemporalMarker(() -> this.eat())
                .waitSeconds(.5)
                .addTemporalMarker(() -> this.moreEating())
                .waitSeconds(.2)
                .forward(6)
                .waitSeconds(.2)
                .back(5)
                .addTemporalMarker(() -> this.eat())
                .waitSeconds(.2)
                .addTemporalMarker(() -> this.moreEating())
                //does it suck
                .lineToSplineHeading(new Pose2d(2, 48, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(2, -5, Math.toRadians(270)))
                .addTemporalMarker(() -> this.raise())
                .addTemporalMarker(() -> this.stopEating())
                .lineToSplineHeading(new Pose2d(25, -35, Math.toRadians(270)))
                .waitSeconds(.5)
                .addTemporalMarker(() -> this.drop())
                .waitSeconds(.5)
                .addDisplacementMarker( this::retract)
                .back(5)
                .lineToSplineHeading(new Pose2d(5, -40, Math.toRadians(270)))
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
                        drive.followTrajectorySequence(trajSeq_right);
                    break;


            }

    }
    public void deposit() {
        robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_REDCENTER_POS);
        robot.wrist.setPosition(robot.WRIST_REDCENTER_POS);
    }

    public void retract(){
        robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_STORE_POS);
        robot.wrist.setPosition(robot.WRIST_STORE_POS);
    }

    public void raise() {
        robot.pixelHolderRotator.setPosition(robot.PIXELHOLDERROTATOR_AUTO_POS);
        robot.wrist.setPosition(robot.WRIST_DEPOSIT_POS);
    }

    public void eat() {
        robot.intake.setPower(1);
        robot.pinchertheleft.setPosition(DriverControl2024.PINCHERLEFTCLOSE);
        robot.pinchertheright.setPosition(DriverControl2024.PINCHERRIGHTCLOSE);
    }
    public void moreEating() {
        robot.intake.setPower(1);
        robot.pinchertheleft.setPosition(DriverControl2024.PINCHERLEFTOPEN);
        robot.pinchertheright.setPosition(DriverControl2024.PINCHERRIGHTOPEN);
    }

    public void stopEating() {
        robot.intake.setPower(0);
    }
    public void drop() {robot.pixelBox.setPosition(outTakeMiddle); }
    public void close() {robot.pixelBox.setPosition(outTakeClosed);}
}

