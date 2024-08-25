package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.intake.teleopConstants;

@TeleOp (name = "TeleOp (Normal Drive)", group = "!Important")
public class teleop extends LinearOpMode {
    SampleMecanumDrive drive;
    teleopConstants teleopConstants;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        teleopConstants = new teleopConstants(hardwareMap);
        waitForStart();
        teleopConstants.onStart();
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Vector2d drivePower = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            drive.setWeightedDrivePower(new Pose2d(drivePower.getX(), drivePower.getY(), -gamepad1.right_stick_x));
            teleopConstants.update(gamepad1, gamepad2);
        }
    }
}
