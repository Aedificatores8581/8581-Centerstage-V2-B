package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.dualServoFinder;
import org.firstinspires.ftc.teamcode.hardware.servoFinder;

@TeleOp
public class servoPosFinderEasy extends LinearOpMode {
    servoFinder servo1;
    dualServoFinder servos2;
    private DcMotor leftArm;
    private DcMotor rightArm;
    @Override
    public void runOpMode() {
        leftArm = hardwareMap.get(DcMotor.class, "leftArm");
        rightArm = hardwareMap.get(DcMotor.class, "rightArm");
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo1 = new servoFinder(hardwareMap, "leftPivot");
        servos2 = new dualServoFinder(hardwareMap, "leftClaw", "rightClaw");
        double armPower;
        waitForStart();
        while (opModeIsActive()) {
            armPower = (gamepad1.right_trigger - gamepad1.left_trigger)*1;
            leftArm.setPower(armPower);
            rightArm.setPower(leftArm.getPower());

            servo1.update(gamepad1, telemetry, getRuntime());
            servos2.update(gamepad2, telemetry, getRuntime());
            telemetry.addData("leftArm", leftArm.getCurrentPosition());
            telemetry.addData("rightArm", rightArm.getCurrentPosition());
            telemetry.update();
        }
    }
}
