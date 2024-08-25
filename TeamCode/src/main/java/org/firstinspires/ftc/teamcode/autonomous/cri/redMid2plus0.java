package org.firstinspires.ftc.teamcode.autonomous.cri;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.intake.arm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.callNewPipeline;
import org.firstinspires.ftc.teamcode.vision.propProcessor;

@Autonomous (name = "(CRI) 🔴 Middle 2+0")
public class redMid2plus0 extends LinearOpMode {

    SampleMecanumDrive drive;
    arm arm;
    callNewPipeline cp;
    String clawPos = "UP";
    int armBackdropPos = -4500;

    double configDelayTime = 3.0;
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        cp = new callNewPipeline(hardwareMap);
        arm = new arm(hardwareMap);

        Pose2d startPose = new Pose2d(60, -32, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Pose2d spike3 = new Pose2d(34,-36,Math.toRadians(90));
        Pose2d spike2 = new Pose2d(34,-34,Math.toRadians(190));
        Pose2d spike1 = new Pose2d(30,-34,Math.toRadians(255));
        Pose2d spikeCenter = new Pose2d(40, -34);

        Pose2d frontOfSpikes = new Pose2d(10,-40, Math.toRadians(270));
        Pose2d behindSpikes = new Pose2d(56,-32, Math.toRadians(270));
        Pose2d behindSpikes2 = new Pose2d(56,-30, Math.toRadians(270));
        Pose2d nearSide = new Pose2d(54, 35, Math.toRadians(270));

        Pose2d backdrop3 = new Pose2d(40,44,Math.toRadians(270));
        Pose2d backdrop2 = new Pose2d(32,44,Math.toRadians(270));
        Pose2d backdrop1 = new Pose2d(26,44,Math.toRadians(270));

        Pose2d cornerPark = new Pose2d(58, 58, Math.toRadians(270));
        Pose2d farPark = new Pose2d(10, 42, Math.toRadians(270));

        TrajectorySequence park;

        propProcessor.position elementPos = propProcessor.position.LEFT;

        double armPower;
        cp.start();
        cp.setAlliance("red");
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
        double automationStartTime = getRuntime(),
                timeTillClaw = 5,
                delayControlCooldown = -1;
        boolean clawsClosed = false;
        String parkSpot = "far";
        Pose2d parkEnd = farPark;
        while (!opModeIsActive()) {
            elementPos = cp.getElementPos();
            timeTillClaw = 5-(getRuntime()-automationStartTime);
            timeTillClaw = (((long)(timeTillClaw * 1e1)) / 1e1);
            if (isStopRequested()) {
                break;
            }
            if (timeTillClaw <= 0) {
                if (!clawsClosed) {
                    arm.claw.left.close();
                    arm.claw.right.close();
                    clawsClosed = true;
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
            if (getRuntime() > delayControlCooldown + 0.25) {
                if (gamepad1.dpad_up) {
                    configDelayTime += 0.5;
                    delayControlCooldown = getRuntime();
                }
                if (gamepad1.dpad_down) {
                    configDelayTime -= 0.5;
                    delayControlCooldown = getRuntime();
                }
            }
            if (configDelayTime < 0) {configDelayTime = 0;}
            telemetry.addLine("Controls: ");
            telemetry.addLine("<△> park in far <x> park in corner");
            telemetry.addLine("<↑> increase delay <↓> decrease delay");
            telemetry.addLine("<←> left pixel slot <→> right pixel slot");
            telemetry.addLine("\nInformation: ");
            telemetry.addData("Time Until Claws Close", timeTillClaw);
            telemetry.addData("Element Pos", elementPos);
            telemetry.addLine("\nConfiguration: ");
            telemetry.addData("Current Delay", configDelayTime);
            telemetry.addData("Park Spot", parkSpot);
            telemetry.update();
        }

        TrajectorySequence position3 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(0.75, () -> {
                    clawPos = "DOWN";
                })
                .lineTo(spikeCenter.vec())
                .lineToLinearHeading(spike3)
                .UNSTABLE_addTemporalMarkerOffset(0.125, () -> {
                    arm.claw.left.open();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.375, () -> {
                    clawPos = "UP";
                })
                .waitSeconds(0.5)
                .lineTo(behindSpikes.vec())
                .lineToLinearHeading(behindSpikes2)
                .waitSeconds(configDelayTime)
                .addDisplacementMarker(() -> {
                    clawPos = "backdrop";
                })
                .lineToLinearHeading(nearSide)
                .addDisplacementMarker(() -> {
                    arm.runToPositionAsync(armBackdropPos);
                })
                .lineToLinearHeading(backdrop3)
                .build();
        TrajectorySequence position2 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(0.75, () -> {
                    clawPos = "DOWN";
                })
                .lineTo(spikeCenter.vec())
                .lineToLinearHeading(new Pose2d(spike2.getX()+5, spike2.getY()+8, spike2.getHeading()))
                .lineToLinearHeading(spike2)
                .UNSTABLE_addTemporalMarkerOffset(0.125, () -> {
                    arm.claw.left.open();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.375, () -> {
                    clawPos = "UP";
                })
                .waitSeconds(0.5)
                .lineTo(behindSpikes.vec())
                .lineToLinearHeading(behindSpikes2)
                .waitSeconds(configDelayTime)
                .addDisplacementMarker(() -> {
                    clawPos = "backdrop";
                })
                .lineToLinearHeading(nearSide)
                .addDisplacementMarker(() -> {
                    arm.runToPositionAsync(armBackdropPos);
                })
                .lineToLinearHeading(backdrop2)
                .build();
        TrajectorySequence position1 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(0.75, () -> {
                    clawPos = "DOWN";
                })
                .lineTo(spikeCenter.vec())
                .lineToLinearHeading(new Pose2d(spike1.getX()+6, spike1.getY()+6, spike1.getHeading()))
                .lineToLinearHeading(spike1)
                .UNSTABLE_addTemporalMarkerOffset(0.125, () -> {
                    arm.claw.left.open();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.375, () -> {
                    clawPos = "UP";
                })
                .waitSeconds(0.5)
                .lineTo(behindSpikes.vec())
                .lineToLinearHeading(behindSpikes2)
                .waitSeconds(configDelayTime)
                .addDisplacementMarker(() -> {
                    clawPos = "backdrop";
                })
                .lineToLinearHeading(nearSide)
                .addDisplacementMarker(() -> {
                    arm.runToPositionAsync(armBackdropPos);
                })
                .lineToLinearHeading(backdrop1)
                .build();

        Pose2d parkStart;
        if (elementPos == propProcessor.position.LEFT) {parkStart = backdrop3;}
        else if (elementPos == propProcessor.position.CENTER) {parkStart = backdrop2;}
        else if (elementPos == propProcessor.position.RIGHT) {parkStart = backdrop1;}
        else {parkStart = backdrop3;}
        park = drive.trajectorySequenceBuilder(parkStart)
                .lineToLinearHeading(new Pose2d(parkEnd.getX(), parkStart.getY(), parkEnd.getHeading()))
                .lineToLinearHeading(parkEnd)
                .build();
        waitForStart();
        if (opModeIsActive()) {
            if (elementPos == propProcessor.position.LEFT) {
                drive.followTrajectorySequenceAsync(position1);
                trajectoryLoop();
            }
            if (elementPos == propProcessor.position.CENTER) {
                drive.followTrajectorySequenceAsync(position2);
                trajectoryLoop();
            }
            if (elementPos == propProcessor.position.RIGHT) {
                drive.followTrajectorySequenceAsync(position3);
                trajectoryLoop();
            }
            arm.claw.right.open();
            sleep(500);
            arm.runToPositionAsync(-50);
            drive.followTrajectorySequenceAsync(park);
            trajectoryLoop();
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
                if( clawPos == "stack") {
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
            telemetry.addData("Arm Motor Position",arm.motor.getPosition());
            telemetry.addData("Arm Position",arm.getPos());
            telemetry.addLine("\n   Position Data:");
            telemetry.addData("x",drive.getPoseEstimate().getX());
            telemetry.addData("y",drive.getPoseEstimate().getY());
            telemetry.addData("heading",drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}
