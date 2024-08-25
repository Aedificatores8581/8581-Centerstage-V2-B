package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class fieldCentricBasic extends LinearOpMode {
    SampleMecanumDrive drive;
    Vector2d input;
    Pose2d poseEstimate;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        double turnPower;
        waitForStart();
        while (opModeIsActive()) {
            turnPower = gamepad1.right_stick_x;
            drive.updatePoseEstimate(); //Update Robot Pose
            poseEstimate = drive.getPoseEstimate(); //Get robot pose
            input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading()); //Field Centric Input
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            turnPower
                    )
            ); // Setting Power
        }
    }
}
