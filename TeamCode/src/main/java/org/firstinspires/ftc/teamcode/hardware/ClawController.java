package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawController {
    private Servo claw1;
    enum position {
        OPEN,
        CLOSED
    }
    double openPos;
    double closedPos;
    position currentPosition = position.OPEN;
    public ClawController(HardwareMap hardwareMap, String device1Name, double openPos, double closedPos) {
        claw1 = hardwareMap.get(Servo.class, device1Name);

        this.openPos = openPos;
        this.closedPos = closedPos;
    }

    public void open() {claw1.setPosition(openPos);currentPosition = position.OPEN;}
    public void close() {claw1.setPosition(closedPos);currentPosition = position.CLOSED;}

    public void toggle() {
        switch(currentPosition) {
            case OPEN:
                close();
                return;
            case CLOSED:
                open();
                return;
        }
    }
    public position getState() {return currentPosition;}
}
