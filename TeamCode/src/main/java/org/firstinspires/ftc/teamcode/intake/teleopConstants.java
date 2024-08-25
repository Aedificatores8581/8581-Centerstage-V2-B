package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.ButtonDown;

public class teleopConstants {
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    arm arm;
    Extension extension;
    ButtonDown left_claw_toggle;
    ButtonDown right_claw_toggle;
    ButtonDown extension_out;
    ButtonDown extension_mid;
    ButtonDown extension_in;
    ButtonDown wrist_toggle;

    private Servo droneLauncher;
    private double dLauncherInit = 0.4, dLauncherOpen = 0.8;
    public teleopConstants(HardwareMap hardwareMap) {
        arm = new arm(hardwareMap);
        extension = new Extension(hardwareMap);
        arm.claw.left.open();
        arm.claw.right.open();
        left_claw_toggle = new ButtonDown(() -> {arm.claw.left.toggle();});
        right_claw_toggle = new ButtonDown(() -> {arm.claw.right.toggle();});
        extension_out = new ButtonDown(() -> {extension.runToPositionAsync(extension.maxPosition);});
        extension_mid = new ButtonDown(() -> {extension.runToPositionAsync(extension.maxPosition/2);});
        extension_in = new ButtonDown(() -> {extension.runToPositionAsync(0);});
        wrist_toggle = new ButtonDown(() -> {toggleWrist();});
        arm.claw.wrist.goTo("UP");
        droneLauncher = hardwareMap.get(Servo.class, "shooter");
        droneLauncher.setPosition(dLauncherInit);
    }
    public void onStart() {
        extension.resetEncoder();
        arm.resetEncoder();
        arm.motor.resetEncoder();
    }
    public void update(Gamepad gm1, Gamepad gm2) {
        gamepad1 = gm1;
        gamepad2 = gm2;


        if (gamepad1.dpad_up && gamepad2.dpad_up) {
            droneLauncher.setPosition(dLauncherOpen);
        }

        double armPower = (gamepad1.right_trigger-gamepad1.left_trigger);
        arm.setPower(armPower);

        left_claw_toggle.update(gamepad2.left_bumper);
        right_claw_toggle.update(gamepad2.right_bumper);
        wrist_toggle.update(gamepad2.square);
        extension_out.update(gamepad2.dpad_up);
        //extension_mid.update(gamepad2.dpad_right || gamepad2.dpad_left);
        extension_in.update(gamepad2.dpad_down);
        if (arm.motor.getPosition() > 1600) {
            arm.claw.backdropParallel(arm.motor.getPosition());
        }
        else if (arm.claw.wrist.getPositionName() == "BACKDROP") {
            arm.claw.wrist.goTo("UP");
        }
        if (gamepad1.dpad_up && gamepad2.triangle) {
            droneLauncher.setPosition(dLauncherOpen);
        }
        extension.update();
    }
    private void toggleWrist() {
        if (arm.claw.wrist.getPositionName() == "UP" ||
                arm.claw.wrist.getPositionName() == "{RAW POSITION}" ||
                arm.claw.wrist.getPositionName() == "BACKDROP_PERPENDICULAR") {
            arm.claw.wrist.goTo("DOWN");
        }
        else if (arm.claw.wrist.getPositionName() == "DOWN") {
            arm.claw.wrist.goTo("UP");
        }
    }
}
