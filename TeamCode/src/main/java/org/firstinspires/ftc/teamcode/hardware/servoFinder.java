package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class servoFinder {
    HardwareMap hardwareMap;
    Gamepad gamepad;

    private Servo servo;
    double servoPos = 0;

    double cooldownEnd = 0;
    double cooldownTime = 0.2;
    String name;
    public servoFinder(HardwareMap hardwareMap, String servoName) {
        this.hardwareMap = hardwareMap;
        servo = hardwareMap.get(Servo.class, servoName);
        this.name = servoName;
    }
    public void update(Gamepad gamepad, Telemetry telemetry, double runTime) {
        this.gamepad = gamepad;
        if (gamepad.dpad_up && runTime > cooldownEnd) {servoPos += 0.05; cooldownEnd = runTime + cooldownTime;}
        if (gamepad.dpad_down && runTime > cooldownEnd) {servoPos -= 0.05; cooldownEnd = runTime + cooldownTime;}
        servo.setPosition(servoPos);
        telemetry.addData((name+" Position"), servoPos);
    }
}