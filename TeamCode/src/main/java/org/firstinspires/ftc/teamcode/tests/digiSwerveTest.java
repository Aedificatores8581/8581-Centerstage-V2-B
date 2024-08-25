package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@TeleOp
public class digiSwerveTest extends LinearOpMode {
    SampleMecanumDrive drive;
    Vector2d input;
    Pose2d poseEstimate;
    double joyDir;
    double turnPower;

    double prevHeading;

    TrajectorySequence toPickUp;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            drive.updatePoseEstimate(); //Update Robot Pose
            poseEstimate = drive.getPoseEstimate(); //Get robot pose
            joyDir = (Math.PI + Math.atan2(-gamepad1.right_stick_x, gamepad1.right_stick_y)) * 180 / Math.PI; //Converting X and Y to degrees
            joyDir = -joyDir + 360; //Conversion from Counterclockwise to clockwise

            if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y) < 0.3) {
                joyDir = Math.toDegrees(poseEstimate.getHeading());
                telemetry.addLine("Defaulting...");
            } //Preventing robot from defaulting to 180 degrees
            turnPower = proportionalTurnPower(joyDir, Math.toDegrees(poseEstimate.getHeading())); //Get Proportional Power for turning to target
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

            if (gamepad1.a) {
                toPickUp = drive.trajectorySequenceBuilder(poseEstimate)
                        .lineToLinearHeading(new Pose2d(0,0,0))
                        .build();
                drive.followTrajectorySequence(toPickUp);
            }
            if (Math.abs(poseEstimate.getHeading() - prevHeading) < 1 && Math.abs(turnPower) > 0.15) {telemetry.addLine("\n!!!IMU Crash Detected!!! Restart robot to fix\n");} //IMU Crash Detection Reporting
            telemetry.addLine("Drive Information: ");
            telemetry.addData("Joystick Angle", joyDir);
            telemetry.addData("Turn Power", turnPower);
            telemetry.addLine("\n Localization: ");
            telemetry.addData("X", poseEstimate.getX());
            telemetry.addData("Y", poseEstimate.getY());
            telemetry.addData("Heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
            prevHeading = poseEstimate.getHeading(); //Get heading for next loop

        }

    }
    private double proportionalTurnPower(double target, double current) {
        double diff = current - target;
        double speedDivider = 80;
        if (diff < 0)
            diff += 360;
         if (diff > 180) {
             diff = target-current;
             if (diff < 0)
                 diff += 360;
             return diff / speedDivider;
         } else {
             return 0 - (diff/speedDivider);
         }
    }
}
