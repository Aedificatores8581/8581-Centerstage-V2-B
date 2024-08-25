package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.intake.arm;

@TeleOp (name = "*ARM_CONTROL")
public class ARM_CONTROL extends LinearOpMode {
    arm arm;
    public void runOpMode() {
        arm = new arm(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            double armPower = (gamepad1.right_trigger-gamepad1.left_trigger);
            arm.setPower(armPower);
        }
    }
}
