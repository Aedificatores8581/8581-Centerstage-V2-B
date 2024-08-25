package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.ButtonDown;
import org.firstinspires.ftc.teamcode.vision.AprilTagManager;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp (name = "TEST: Trajectory To Tag")
public class trajectoryToTagTest extends LinearOpMode {
    public static int TARGET_ID = 5;
    public static int TARGET_READ_COUNT = 100;
    public static double OFFSET_X = 15;
    public static double OFFSET_Y = 5;
    public static double OFFSET_HEADING = 0;

    Pose2d OFFSET_POSE = new Pose2d(OFFSET_X,OFFSET_Y,OFFSET_HEADING);
    AprilTagManager at;
    SampleMecanumDrive drive;

    Pose2d aprilTagPose;
    Pose2d aprilTagPoseBase;
    Pose2d aprilTagAverageRead;
    Pose2d targetPosition;
    @Override
    public void runOpMode() {
        at = new AprilTagManager(hardwareMap);
        at.buildPortal();
        drive = new SampleMecanumDrive(hardwareMap);

        ButtonDown runToTagBD = new ButtonDown(() -> {
            driveToTag(TARGET_ID, OFFSET_POSE);
        });

        waitForStart();
        while (opModeIsActive()) {
            OFFSET_POSE = new Pose2d(OFFSET_X,OFFSET_Y,OFFSET_HEADING);
            Pose2d drivePower = new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            drive.setWeightedDrivePower(drivePower);
            drive.updatePoseEstimate();
            at.update(TARGET_ID);

            runToTagBD.update(gamepad1.a);

            handleTelemetry();
        }
    }
    private void driveToTagAsync(int tagId, Pose2d offset) {
        drive.updatePoseEstimate();
        Pose2d drivePoseTemp = drive.getPoseEstimate();

        Pose2d tagPose = getAverageTagPose(tagId, TARGET_READ_COUNT);
        aprilTagAverageRead = tagPose;

        tagPose = tagPose.minus(offset);
        //tagPose = new Pose2d(new Vector2d(tagPose.getX(),tagPose.getY()).rotated(Math.toRadians(drive.getPoseEstimate().getHeading())),tagPose.getHeading());
        aprilTagPoseBase = tagPose;
        //tagPose = drivePoseTemp.minus(tagPose);
        tagPose = new Pose2d(
                drivePoseTemp.getX() - tagPose.getY(),
                drivePoseTemp.getY() + tagPose.getY(),
                drivePoseTemp.getHeading() + tagPose.getHeading()
        );
        aprilTagPose = tagPose;
        targetPosition = tagPose;
        drive.followTrajectoryAsync(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(tagPose)
                        .build()
        );
    }
    private void driveToTag(int tagId, Pose2d offset) {
        driveToTagAsync(tagId, offset);
        driveToTagRunLoop();
    }
    private void driveToTagRunLoop() {
        while (drive.isBusy() && !Thread.interrupted()) {
            at.update(TARGET_ID);
            drive.update();

            telemetry.addLine("     Tag Info:");
            telemetry.addData("April Tag Pose Base:", aprilTagPoseBase);
            telemetry.addData("April Tag Pose:", aprilTagPose);
            handleTelemetry();

            if (isStopRequested()) {
                break;
            }
        }
    }
    private Pose2d getAverageTagPose(int tagId, int targetReads) {
        List<Pose2d> tagPoses = new ArrayList<Pose2d>();
        int reads = 0;
        while (!(reads >= targetReads)) {
            at.update(tagId);
            if (at.targetTag.id == tagId) {
                tagPoses.add(new Pose2d(at.targetTag.getZ(), at.targetTag.getX(), Math.toRadians(at.targetTag.getYaw()) ));
                reads ++;
            }
            if (isStopRequested()) {
                break;
            }

            telemetry.addLine("Reading April Tags...");
            telemetry.addData("Read Count", reads);
            telemetry.update();
        }
        double avgX = 0;
        double avgY = 0;
        double avgHeading = 0;
        for (Pose2d tagPoseItem : tagPoses) {
            avgX += tagPoseItem.getX();
            avgY += tagPoseItem.getY();
            avgHeading += tagPoseItem.getHeading();

            if (isStopRequested()) {
                break;
            }
        }
        avgX /= reads;
        avgY /= reads;
        avgHeading /= reads;
        return new Pose2d(avgX, avgY, avgHeading);
    }
    private void handleTelemetry() {
        telemetry.addLine("     Tag Info 1:");
        telemetry.addData("Average Tag Pose", aprilTagAverageRead);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addLine();
        telemetry.addLine("     Localization Info:");
        telemetry.addData("X:", drive.getPoseEstimate().getX());
        telemetry.addData("Y:", drive.getPoseEstimate().getY());
        telemetry.addData("Heading:", Math.toDegrees(drive.getPoseEstimate().getHeading()));
        telemetry.addLine();
        telemetry.addLine("     April Tag Info:");
        telemetry.addData("Tags Detected", at.getTagCount());
        telemetry.addData("id", at.targetTag.getId());
        telemetry.addData("x", at.targetTag.getX());
        telemetry.addData("y", at.targetTag.getY());
        telemetry.addData("z", at.targetTag.getZ());
        telemetry.addData("yaw", at.targetTag.getYaw());
        telemetry.addData("Last Tag ID", at.lastTag.getId());
        telemetry.update();
    }
}
