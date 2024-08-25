package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.intake.Extension;
import org.firstinspires.ftc.teamcode.hardware.ButtonDown;

@TeleOp
public class extensionTest extends LinearOpMode {
    private Extension extension;
    int targetPosition = 0;
    @Override
    public void runOpMode() {
        extension = new Extension(hardwareMap);
        ButtonDown buttonDown = new ButtonDown(() -> {targetPosition -= 100;});
        ButtonDown buttonUp = new ButtonDown(() -> {targetPosition += 100;});
        ButtonDown resetEncoder = new ButtonDown(() -> {extension.resetEncoder();});

        extension.resetEncoder();
        waitForStart();
        while (opModeIsActive()) {
            buttonDown.update(gamepad1.dpad_down);
            buttonUp.update(gamepad1.dpad_up);
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            if (!extension.isBusy()) {extension.setPower(power);}
            if (gamepad1.a) {extension.runToPositionAsync(targetPosition);}
            resetEncoder.update(gamepad1.b);
            extension.update();
            telemetry.addData("Servo Position", extension.getPosition());
            telemetry.addData("Target Pos (To be set)", targetPosition);
            telemetry.addData("error",extension.getError());
            telemetry.addData("Servo Power", extension.getPower());
            telemetry.update();
        }
    }
}
