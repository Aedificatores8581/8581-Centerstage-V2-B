package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.vision.callReadPipeline;

import java.io.File;

@TeleOp
public class calibrateVision extends LinearOpMode {

    @Override
    public void runOpMode() {
        callReadPipeline crp = new callReadPipeline(hardwareMap);

        String selectedColor = "None";
        crp.StartPipeline();
        while(!opModeIsActive()) {
            telemetry.addLine("Initialization Complete!");
            telemetry.update();
            if (isStopRequested()) {
                crp.StopPipeline();break;
            }
        }
        int RH = 0;
        int RS = 0;
        int RV = 0;
        int BH = 0;
        int BS = 0;
        int BV = 0;
        waitForStart();
        while (opModeIsActive()) {
            int HueVariation = 10,
                SaturationVariation = 40,
                ValueVariation = 30;
            if (gamepad1.a) {selectedColor = "Red";}
            if (gamepad1.b) {selectedColor = "Blue";}
            if (gamepad1.x) {selectedColor = "None";}
            if (gamepad1.y) {selectedColor = "Write";}
            if (selectedColor == "Red") {
                RH = (int) crp.getH() - HueVariation;
                RS = (int) crp.getS() - SaturationVariation;
                RV = (int) crp.getV() - ValueVariation;
            } else if (selectedColor == "Blue") {
                BH = (int) crp.getH() - HueVariation;
                BS = (int) crp.getS() - SaturationVariation;
                BV = (int) crp.getV() - ValueVariation;
            }
            if (selectedColor == "Write") {
                File cameraDetection = AppUtil.getInstance().getSettingsFile("propCalibration.txt");
                String RVals = (//Red
                        RH + "\n" +
                        RS + "\n" +
                        RV + "\n"
                );
                String BVals = (//Blue
                        BH + "\n" +
                        BS + "\n" +
                        BV + "\n"
                );
                String calibValues = (
                        RVals + "" + BVals
                );
                ReadWriteFile.writeFile(cameraDetection, calibValues);
                //String read = ReadWriteFile.readFile(cameraDetection);
                telemetry.addData("Values: ", ("Red\n" +
                        RVals + ",\n" +
                        "Blue\n" +
                        BVals + ",\n"));
                telemetry.update();
                break;
            }
            if (isStopRequested()) {
                crp.StopPipeline();break;
            }
            telemetry.addLine("Press A for Red, B for Blue, and X/Y for None");
            telemetry.addData("Selected Color", selectedColor);
            telemetry.addLine("\n   Values:");
            telemetry.addData("H Value", crp.getH());
            telemetry.addData("S Value", crp.getS());
            telemetry.addData("V Value", crp.getV());
            telemetry.update();
        }
    }
}
