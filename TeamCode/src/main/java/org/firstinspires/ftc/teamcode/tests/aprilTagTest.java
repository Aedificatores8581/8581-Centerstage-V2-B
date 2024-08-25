package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.vision.AprilTagPortal;


@TeleOp
public class aprilTagTest extends LinearOpMode {
    AprilTagPortal at;
    @Override
    public void runOpMode() {
        at = new AprilTagPortal(hardwareMap);
        at.init("Webcam 1");

        //TODO: Use String Splitting for File Reading
        String str = " Hello I'm your String";
        String[] splitStr = str.split("\\s+");

        waitForStart();
        while (opModeIsActive()) {
            at.update();
            telemetry.addData("Tags Detected", at.getTagCount());
            telemetry.addData("id", at.getId());
            telemetry.addData("x", at.getX());
            telemetry.addData("y", at.getY());
            telemetry.addData("z", at.getZ());
            telemetry.update();
            sleep(20);
        }
    }
}
