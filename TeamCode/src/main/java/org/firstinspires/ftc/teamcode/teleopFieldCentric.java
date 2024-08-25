package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.intake.teleopConstants;

@TeleOp (name = "TeleOp (Field Centric)", group = "!Important")
public class teleopFieldCentric extends LinearOpMode {
    SampleMecanumDrive drive;
    teleopConstants teleopConstants;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        teleopConstants = new teleopConstants(hardwareMap);
        waitForStart();
        teleopConstants.onStart();
        double headingOffset = 0;
        double heading;
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            heading = drive.getPoseEstimate().getHeading() - headingOffset;
            Vector2d drivePower = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-heading);
            drive.setWeightedDrivePower(new Pose2d(drivePower.getX(), drivePower.getY(), -gamepad1.right_stick_x));
            teleopConstants.update(gamepad1, gamepad2);

            if (gamepad1.dpad_down) {
                headingOffset = drive.getPoseEstimate().getHeading();
            }
        }
    }
}
