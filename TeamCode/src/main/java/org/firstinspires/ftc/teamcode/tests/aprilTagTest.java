package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.AprilTagManager;


@TeleOp
public class aprilTagTest extends LinearOpMode {
    AprilTagManager at;
    @Override
    public void runOpMode() {
        at = new AprilTagManager(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            at.update();
            telemetry.addData("Tags Detected", at.getTagCount());
            telemetry.addData("id", at.lastTag.getId());
            telemetry.addData("x", at.lastTag.getX());
            telemetry.addData("y", at.lastTag.getY());
            telemetry.addData("z", at.lastTag.getZ());
            telemetry.update();
        }
    }
}
