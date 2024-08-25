package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class dualServoFinder {
    HardwareMap hardwareMap;
    Gamepad gamepad;

    private Servo servo1;
    private Servo servo2;
    double servoPos1 = 0;
    double servoPos2 = 0;

    double cooldown1End = 0;
    double cooldown2End = 0;
    double cooldownTime = 0.2;
    String name1;
    String name2;
    public dualServoFinder(HardwareMap hardwareMap, String servo1Name, String servo2Name) {
        this.hardwareMap = hardwareMap;
        servo1 = hardwareMap.get(Servo.class, servo1Name);
        servo2 = hardwareMap.get(Servo.class, servo2Name);
        this.name1 = servo1Name;
        this.name2 = servo2Name;
    }
    public void update(Gamepad gamepad, Telemetry telemetry, double runTime) {
        this.gamepad = gamepad;
        if (gamepad.dpad_up && runTime > cooldown1End) {
            servoPos1 += 0.05; cooldown1End = runTime + cooldownTime;}
        if (gamepad.dpad_down && runTime > cooldown1End) {
            servoPos1 -= 0.05; cooldown1End = runTime + cooldownTime;}

        if (gamepad.dpad_right && runTime > cooldown2End) {
            servoPos2 += 0.05; cooldown2End = runTime + cooldownTime;}
        if (gamepad.dpad_left && runTime > cooldown2End) {
            servoPos2 -= 0.05; cooldown2End = runTime + cooldownTime;}

        servo1.setPosition(servoPos1);
        servo2.setPosition(servoPos2);
        telemetry.addData((name1+" Position"), servoPos1);
        telemetry.addData((name2+" Position"), servoPos2);
    }
}