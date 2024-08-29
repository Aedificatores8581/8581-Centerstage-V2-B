package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.AprilTagDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Config
@TeleOp (name = "PID To April Tag Test")
public class pidToTagTest extends LinearOpMode {

    public static double TARGET_X = 0, TARGET_Y = 15, TARGET_HEADING = 0;
    public static int TARGET_ID = 4;
    AprilTagDrive atDrive;
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        atDrive = new AprilTagDrive(hardwareMap, drive);
        waitForStart();
        ElapsedTime runTime = new ElapsedTime();
        double lastRunTime = 0;
        double loopTime;
        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            if (gamepad1.a) {
                drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            } else {
                atDrive.runToTag(TARGET_ID, new Pose2d(TARGET_X, TARGET_Y, TARGET_HEADING));
            }

            loopTime = runTime.seconds() - lastRunTime;
            loopTime *= 1000;
            loopTime *= 100;
            loopTime = Math.round(loopTime);
            loopTime /= 100;
            lastRunTime = runTime.seconds();

            telemetry.addData("Error", atDrive.getError());
            telemetry.addData("ATDrive Status", atDrive.getCurrentStatus());
            telemetry.addData("ATDrive Busy", atDrive.isBusy());
            telemetry.addLine();
            telemetry.addData("Run Time", runTime.seconds());
            telemetry.addData("Loop Time (Milliseconds)", loopTime);
            telemetry.update();
        }
    }
}
