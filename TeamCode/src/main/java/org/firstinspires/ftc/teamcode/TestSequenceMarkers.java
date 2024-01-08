package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;

@Autonomous(group="lol imagine")
public class TestSequenceMarkers extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardwarerobot robot = new Hardwarerobot();
        org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        robot.pixelHolderRotator.setPosition(0);
        robot.auto.setPosition(.5);
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
        TrajectorySequence trajSeq_left = builder
                .lineToSplineHeading(new Pose2d(30,2, Math.toRadians(0)))
                .turn(180)
                .addDisplacementMarker(4, () -> robot.auto.setPosition(robot.AUTO_OPEN_POS))
                .lineToSplineHeading(new Pose2d(30, -45, Math.toRadians(270)))
                .addDisplacementMarker(() -> telemetry.addLine("get wrecked"))
                .lineToSplineHeading(new Pose2d(40, -40, Math.toRadians(270)))
                .build();

        telemetry.addLine("Here are all the markers in this thing:");
        for (int i = 0; i < trajSeq_left.size() - 1; i++) {
            SequenceSegment segment = trajSeq_left.get(i);
            if (segment instanceof TrajectorySegment) {
                Trajectory trajectory = ((TrajectorySegment) segment).getTrajectory();
                telemetry.addLine("We have a trajectory segment with duration " + trajectory.duration());
                for (TrajectoryMarker marker : trajectory.getMarkers()) {
                    telemetry.addLine("trajectory marker with time " + marker.getTime());
                }
            }
            for (TrajectoryMarker marker : segment.getMarkers()) {
                telemetry.addLine("index " + i + ", " + marker.getTime());
            }
        }

        telemetry.addLine("Here are the logs:");
        for (String line : builder.logs) {
            telemetry.addLine(line);
        }

        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            sleep(500);
        }
    }
}
