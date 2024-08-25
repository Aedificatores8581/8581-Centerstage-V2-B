package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.ButtonDown;
import org.firstinspires.ftc.teamcode.hardware.PIDController;
import org.firstinspires.ftc.teamcode.vision.AprilTagManager;

@Config
@TeleOp
public class pidToTagTest extends LinearOpMode {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0.09,  0, 0),
            HEADING_PID = new PIDCoefficients(0.01, 0, 0);

    public static double TARGET_X = 0, TARGET_Y = 15, TARGET_HEADING = 0;

    public static double CONTINUOUS_DRIVE_TIME = 0.25;
    public static int TARGET_ID = 4;
    AprilTagManager at;
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() {
        at = new AprilTagManager(hardwareMap);
        at.buildPortal();
        drive = new SampleMecanumDrive(hardwareMap);

        PIDController xPIDControl = new PIDController();
        PIDController yPIDControl = new PIDController();
        PIDController headingPIDControl = new PIDController();

        ElapsedTime noTagTime = new ElapsedTime();
        ButtonDown noTag = new ButtonDown(() -> {
            noTagTime.reset();
        });


        waitForStart();
        ElapsedTime runTime = new ElapsedTime();
        double lastRunTime = 0;
        double loopTime;
        while (opModeIsActive()) {
            at.update(TARGET_ID);

            xPIDControl.setPID(TRANSLATIONAL_PID.kP,TRANSLATIONAL_PID.kI,TRANSLATIONAL_PID.kD);
            yPIDControl.setPID(TRANSLATIONAL_PID.kP,TRANSLATIONAL_PID.kI,TRANSLATIONAL_PID.kD);
            headingPIDControl.setPID(HEADING_PID.kP,HEADING_PID.kI,HEADING_PID.kD);

            Pose2d drivePower = new Pose2d(0,0,0);

            noTag.update(at.getTagCount() > 0);
            if (noTagTime.seconds() < CONTINUOUS_DRIVE_TIME || at.getTagCount() > 0) {
                double xPower = -xPIDControl.PIDControl(0, at.targetTag.getX() - TARGET_X);
                double yPower = yPIDControl.PIDControl(0, at.targetTag.getZ() - TARGET_Y);
                double pivotPower = headingPIDControl.PIDControl(0, -at.targetTag.getYaw() - TARGET_HEADING);

                drivePower = new Pose2d(yPower, xPower, pivotPower);
            }
            if (gamepad1.a)
                drivePower = new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            drive.setWeightedDrivePower(drivePower);

            loopTime = runTime.seconds() - lastRunTime;
            loopTime *= 1000;
            loopTime *= 100;
            loopTime = Math.round(loopTime);
            loopTime /= 100;
            lastRunTime = runTime.seconds();

            telemetry.addData("Tags Detected", at.getTagCount());
            telemetry.addData("id", at.targetTag.getId());
            telemetry.addData("x", at.targetTag.getX());
            telemetry.addData("y", at.targetTag.getY());
            telemetry.addData("z", at.targetTag.getZ());
            telemetry.addLine();
            telemetry.addData("Last Tag ID", at.lastTag.getId());
            telemetry.addLine();
            telemetry.addData("Run Time", runTime.seconds());
            telemetry.addData("Loop Time (Milliseconds)", loopTime);
            telemetry.update();
        }
    }
}