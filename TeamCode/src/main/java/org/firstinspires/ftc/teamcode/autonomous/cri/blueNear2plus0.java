package org.firstinspires.ftc.teamcode.autonomous.cri;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.intake.arm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagPortal;
import org.firstinspires.ftc.teamcode.vision.callNewPipeline;
import org.firstinspires.ftc.teamcode.vision.propProcessor;

@Autonomous (name = "(CRI) 🟦 Near 2+0")
public class blueNear2plus0 extends LinearOpMode {
    SampleMecanumDrive drive;
    callNewPipeline cp;
    AprilTagPortal at;
    arm arm;

    int armBackdrop = 2185;
    double armSetPower = 0.5;
    double configDelayTime = 0;
    String clawPos = "UP";
    int armBackdropPos = -4600;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        cp = new callNewPipeline(hardwareMap);
        arm = new arm(hardwareMap);

        Pose2d startPose = new Pose2d(-60, 17, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        Pose2d spike1 = new Pose2d(-28,36,Math.toRadians(270));
        Pose2d spike2 = new Pose2d(-24, 24,Math.toRadians(270));
        Pose2d spike3 = new Pose2d(-32,14,Math.toRadians(270));

        Pose2d backdrop1 = new Pose2d(-40,47.5,Math.toRadians(270));
        Pose2d backdrop2 = new Pose2d(-34,47.5,Math.toRadians(270));
        Pose2d backdrop3 = new Pose2d(-24,47.5,Math.toRadians(270));

        Pose2d cornerPark = new Pose2d(-58, 58, Math.toRadians(270));
        Pose2d farPark = new Pose2d(-10, 58, Math.toRadians(270));

        TrajectorySequence position1 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(1.5, () -> {
                    clawPos = "DOWN";
                })
                .lineToLinearHeading(spike1)
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    arm.claw.left.open();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    clawPos = "backdrop";
                    arm.runToPositionAsync(armBackdropPos);
                    arm.claw.backdropParallel(arm.motor.getPosition());
                })
                .lineToLinearHeading(backdrop1)
                .build();
        TrajectorySequence position2 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(1.5, () -> {
                    clawPos = "DOWN";
                })
                .lineToLinearHeading(spike2)
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    arm.claw.left.open();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    clawPos = "backdrop";
                    arm.runToPositionAsync(armBackdropPos);
                    arm.claw.backdropParallel(arm.motor.getPosition());
                })
                .lineToLinearHeading(backdrop2)
                .build();
        TrajectorySequence position3 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(1.5, () -> {
                    clawPos = "DOWN";
                })
                .lineToLinearHeading(new Pose2d(spike3.getX(), spike3.getY()+2, spike3.getHeading()))
                .lineToLinearHeading(spike3)
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    arm.claw.left.open();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    clawPos = "backdrop";
                    arm.runToPositionAsync(armBackdropPos);
                    arm.claw.backdropParallel(arm.motor.getPosition());
                })
                .lineToLinearHeading(backdrop3)
                .build();
        TrajectorySequence park;

        propProcessor.position elementPos = propProcessor.position.LEFT;

        double armPower;
        cp.start();
        cp.setAlliance("blue");
        arm.claw.wrist.goTo("UP");
        while (!opModeIsActive()) {
            elementPos = cp.getElementPos();

            armPower = (gamepad1.right_trigger - gamepad1.left_trigger)*0.5;
            arm.setPower(armPower);
            if (gamepad1.x) {
                arm.resetEncoder();
                break;
            }

            if (isStopRequested()) {
                break;
            }
            telemetry.addData("Element Pos", elementPos);
            telemetry.update();
        }
        double automationStartTime = getRuntime();
        double timeTillClaw = 5;
        boolean clawsClosed = false;
        String parkSpot = "corner";
        Pose2d parkEnd = cornerPark;
        while (!opModeIsActive()) {
            elementPos = cp.getElementPos();
            timeTillClaw = 5-(getRuntime()-automationStartTime);
            timeTillClaw = (double)(((long)(timeTillClaw * 1e1)) / 1e1);
            if (isStopRequested()) {
                break;
            }
            if (timeTillClaw < 0.01) {
                if (!clawsClosed) {
                    arm.claw.left.close();
                    arm.claw.right.close();
                }
                timeTillClaw = 0;
            }
            if (gamepad1.triangle) {
                parkEnd = farPark;
                parkSpot = "far";
            }
            if (gamepad1.cross) {
                parkEnd = cornerPark;
                parkSpot = "corner";
            }
            telemetry.addLine("Controls: ");
            telemetry.addLine("<△> park in far <x> park in corner");
            telemetry.addLine("\nInformation: ");
            telemetry.addData("Time Until Claws Close", timeTillClaw);
            telemetry.addData("Element Pos", elementPos);
            telemetry.addLine("\nConfiguration: ");
            telemetry.addData("Park Spot", parkSpot);
            telemetry.update();
        }

        Pose2d parkStart;
        if (elementPos == propProcessor.position.LEFT) {parkStart = backdrop1;}
        else if (elementPos == propProcessor.position.CENTER) {parkStart = backdrop2;}
        else if (elementPos == propProcessor.position.RIGHT) {parkStart = backdrop3;}
        else {parkStart = backdrop1;}
        park = drive.trajectorySequenceBuilder(parkStart)
                .addTemporalMarker(0, () -> {
                    arm.runToPositionAsync(-50);
                })
                .lineToLinearHeading(new Pose2d(parkEnd.getX(), parkStart.getY(), parkEnd.getHeading()))
                .lineToLinearHeading(parkEnd)
                .build();
        waitForStart();
        if (opModeIsActive()) {

            if (elementPos == propProcessor.position.CENTER) {
                drive.followTrajectorySequenceAsync(position2);
                trajectoryLoop();

            }
            if (elementPos == propProcessor.position.RIGHT) {
                drive.followTrajectorySequenceAsync(position3);
                trajectoryLoop();
            }
            else {
                drive.followTrajectorySequenceAsync(position1);
                trajectoryLoop();
            }
            ElapsedTime armTime = new ElapsedTime();
            while (arm.isBusy()) {
                arm.claw.backdropParallel(arm.motor.getPosition());
                if (isStopRequested()) break; if (armTime.seconds() >1.5) {break;}}
            arm.setPower(0);
            arm.claw.backdropParallel(arm.motor.getPosition());
            sleep(200);
            arm.claw.right.open();
            sleep(200);

            drive.followTrajectorySequenceAsync(park);
            trajectoryLoop();

            armTime.reset();
            while (arm.isBusy()) {if (isStopRequested()) break; if (armTime.seconds() >1.5) {break;}}
            arm.setPower(0);

        }
    }
    private void trajectoryLoop() {
        while (!Thread.currentThread().isInterrupted() && drive.isBusy() && !isStopRequested()) {
            drive.update();
            arm.update();
            if (!arm.isBusy()) {
                if (clawPos == "DOWN") {
                    arm.claw.wrist.goTo("DOWN");
                }
                if (clawPos == "UP") {
                    arm.claw.wrist.goTo("UP");
                }
                if (clawPos == "stack") {
                    double degreesOff = arm.getError(); //Get Initial Error
                    degreesOff *= 360.0 / 8192.0; //Convert to Degree
                    double wristPosition = degreesOff / 45.0 + 0.15;
                    arm.claw.wrist.setPositionRaw(wristPosition);
                }
            }
            if (clawPos == "backdrop") {
                arm.claw.left.close();
                arm.claw.backdropParallel(arm.motor.getPosition());
            }
            telemetry.addData("Element Pos", cp.getElementPos());
            telemetry.addLine("\n   Arm Data:");
            telemetry.addData("Arm Motor Position", arm.motor.getPosition());
            telemetry.addData("Arm Position", arm.getPos());
            telemetry.addLine("\n   Position Data:");
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}