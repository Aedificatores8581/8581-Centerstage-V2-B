package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.ButtonDown;
import org.firstinspires.ftc.teamcode.intake.Claw;
@TeleOp
public class servoTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        Claw claw = new Claw(hardwareMap);
        claw.left.open();
        claw.right.open();
        ButtonDown left_bumper = new ButtonDown(() -> {claw.left.toggle();});
        ButtonDown right_bumper = new ButtonDown(() -> {claw.right.toggle();});
        waitForStart();
        while (opModeIsActive()) {
            left_bumper.update(gamepad1.left_bumper);
            right_bumper.update(gamepad1.right_bumper);
            if (gamepad1.a) {claw.wrist.goTo("UP");}
            if (gamepad1.b) {claw.wrist.goTo("DOWN");}
        }
    }
}