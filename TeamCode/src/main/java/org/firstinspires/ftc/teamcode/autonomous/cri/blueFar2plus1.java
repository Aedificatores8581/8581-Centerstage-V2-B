package org.firstinspires.ftc.teamcode.autonomous.cri;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.intake.Claw;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.callNewPipeline;
import org.firstinspires.ftc.teamcode.vision.propProcessor;
import org.firstinspires.ftc.teamcode.intake.arm;

@Autonomous (name = "(CRI) 🟦 Far 2+1")
public class blueFar2plus1 extends LinearOpMode {

    private enum Slot {LEFT, RIGHT};
    Slot selectedSlot = Slot.LEFT;

    SampleMecanumDrive drive;
    arm arm;
    callNewPipeline cp;
    String clawPos = "UP";
    int armBackdropPos = -4600;

    double configDelayTime = 1.0;
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        cp = new callNewPipeline(hardwareMap);
        arm = new arm(hardwareMap);
        Pose2d startPose = new Pose2d(-60, -38, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        Pose2d spike1 = new Pose2d(-31,-36,Math.toRadians(90)),
                spike2 = new Pose2d(-10,-36,Math.toRadians(170)),
                spike3 = new Pose2d(-16,-45,Math.toRadians(180));

        Pose2d stack = new Pose2d(-14,-56.5,Math.toRadians(270));

        Pose2d frontOfSpikes = new Pose2d(-11.5,-40, Math.toRadians(-90));
        Pose2d behindSpikes = new Pose2d(0,0,Math.toRadians(180));
        Pose2d nearSide = new Pose2d(-9.5, 83, Math.toRadians(-90));

        Pose2d backdrop1 = new Pose2d(-39,91,Math.toRadians(-90)),
                backdrop2 = new Pose2d(-33,91,Math.toRadians(-90)),
                backdrop3 = new Pose2d(-27,91,Math.toRadians(-90));
        Pose2d cornerPark = new Pose2d(-58, 100, Math.toRadians(-90));
        Pose2d farPark = new Pose2d(-10, 88, Math.toRadians(-90));

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
        double automationStartTime = getRuntime(),
                timeTillClaw = 5,
                delayControlCooldown = -1;
        boolean clawsClosed = false;
        String parkSpot = "far";
        Pose2d parkEnd = farPark;
        while (!opModeIsActive()) {
            elementPos = cp.getElementPos();
            timeTillClaw = 5-(getRuntime()-automationStartTime);
            timeTillClaw = (double)(((long)(timeTillClaw * 1e1)) / 1e1);
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
            if (gamepad1.dpad_left) {
                selectedSlot = Slot.LEFT;}
            if (gamepad1.dpad_right) {
                selectedSlot = Slot.RIGHT;}
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
            telemetry.addData("Pixel Spot", selectedSlot);
            telemetry.update();
        }



        TrajectorySequence position1 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(0.5, () -> {
                    clawPos = "DOWN";
                })
                .lineToLinearHeading(new Pose2d(spike1.getX(), spike1.getY()-4, spike1.getHeading()))
                .lineToLinearHeading(spike1)
                .UNSTABLE_addTemporalMarkerOffset(0.125, () -> {
                    arm.claw.left.open();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.375, () -> {
                    clawPos = "UP";
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(frontOfSpikes)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.runToPositionAsync(-193);
                    clawPos = "stack";
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(stack)
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    arm.claw.left.close();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(frontOfSpikes)
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
        TrajectorySequence position2 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(0.5, () -> {
                    clawPos = "DOWN";
                })
                .lineToLinearHeading(spike2)
                .UNSTABLE_addTemporalMarkerOffset(0.125, () -> {
                    arm.claw.left.open();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.375, () -> {
                    clawPos = "UP";
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(frontOfSpikes)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.runToPositionAsync(-193);
                    clawPos = "stack";
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(stack)
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    arm.claw.left.close();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(frontOfSpikes)
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
        TrajectorySequence position3 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(0.5, () -> {
                    clawPos = "DOWN";
                })
                .lineToLinearHeading(spike3)
                .UNSTABLE_addTemporalMarkerOffset(0.125, () -> {
                    arm.claw.left.open();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.375, () -> {
                    clawPos = "UP";
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(frontOfSpikes)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    arm.runToPositionAsync(-193);
                    clawPos = "stack";
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(stack)
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    arm.claw.left.close();
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(frontOfSpikes)
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

        Pose2d parkStart;
        if (elementPos == propProcessor.position.LEFT) {parkStart = backdrop1;}
        else if (elementPos == propProcessor.position.CENTER) {parkStart = backdrop2;}
        else if (elementPos == propProcessor.position.RIGHT) {parkStart = backdrop3;}
        else {parkStart = backdrop1;}
        park = drive.trajectorySequenceBuilder(parkStart)
                .lineToLinearHeading(new Pose2d(parkEnd.getX(), parkStart.getY(), parkEnd.getHeading()))
                .lineToLinearHeading(parkEnd)
                .build();
        waitForStart();
        if (opModeIsActive()) {
            Pose2d backdrop = backdrop1;
            if (elementPos == propProcessor.position.LEFT) {
                drive.followTrajectorySequenceAsync(position1);
                trajectoryLoop();
                backdrop = backdrop1;
            }
            if (elementPos == propProcessor.position.CENTER) {
                drive.followTrajectorySequenceAsync(position2);
                trajectoryLoop();
                backdrop = backdrop2;
            }
            if (elementPos == propProcessor.position.RIGHT) {
                drive.followTrajectorySequenceAsync(position3);
                trajectoryLoop();
                backdrop = backdrop3;
            }
            arm.claw.right.open();
            sleep(250);
            drive.followTrajectorySequenceAsync(
                    drive.trajectorySequenceBuilder(backdrop)
                            .lineToLinearHeading(new Pose2d(backdrop2.getX(), backdrop.getY()+2.5, backdrop.getHeading()))
                            .build()
            );
            trajectoryLoop();
            arm.claw.left.open();
            sleep(250);
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
                    double degreesOff = (double) arm.getError(); //Get Initial Error
                    degreesOff *= 360.0 / 8192.0; //Convert to Degree
                    double wristPosition = degreesOff / 45.0 + 0.15;
                    arm.claw.wrist.setPositionRaw(wristPosition);
                }
            }
            if (clawPos == "backdrop") {
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
