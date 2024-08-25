package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.callNewPipeline;

@TeleOp
public class visionTest extends LinearOpMode {
    callNewPipeline cp;
    @Override
    public void runOpMode() {
        cp = new callNewPipeline(hardwareMap);
        cp.start();
        cp.propProcessor.visualizeRecognition = true;
        String selectedColor = "blue";
        while (!opModeIsActive()) {

            if (gamepad1.x) {selectedColor = "blue";}
            if (gamepad1.b) {selectedColor = "red";}
            if (selectedColor == "blue") {
                cp.setAlliance("blue");
                gamepad1.setLedColor(0,0,255,100);
            }
            if (selectedColor == "red") {
                cp.setAlliance("red");
                gamepad1.setLedColor(255,0,0,100);
            }
            telemetry.addLine("Press X/Square for the Blue Element\nPress B/Circle for the Red Element");
            telemetry.addData("Selected Color", selectedColor);
            telemetry.addLine();
            telemetry.addData("Element Position", cp.getElementPos());
            telemetry.update();
            if (isStopRequested()) {
                break;
            }
        }
        waitForStart();
    }
}
