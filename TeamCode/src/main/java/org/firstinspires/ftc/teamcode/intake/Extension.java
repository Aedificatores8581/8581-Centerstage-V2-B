package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Extension {
    HardwareMap hardwareMap;
    private CRServo servo;
    private AnalogInput encoder;

    public static final double maxPosition = 1030;

    private double rawPosition = 0;
    private double power = 0;
    private double lastPosition;
    private double positionOffset = 0;
    private int positionIteration = 0;
    private double targetPosition = 0;
    private boolean isBusy = false;
    private boolean encoderHasReset = true;
    public Extension(HardwareMap hw) {
        this.hardwareMap = hw;
        servo = hardwareMap.get(CRServo.class, "extensionServo");
        encoder = hardwareMap.get(AnalogInput.class, "extensionServoEncoder");
    }
    public void resetEncoder() {encoderHasReset = false;} //Scheduled in Update Function
    public void setPower(double power) { servo.setPower(power); this.power = power; }
    public void runToPositionAsync(double position) {this.targetPosition = position;} //Scheduled in Update Function
    private void powerToPos(double position) {
        //Variable Power to Ensure Efficiency and Accuracy
        if (Math.abs(getError()) > 360) {
            power = 1;
        }
        else if (Math.abs(getError()) > 90) {
            power = 0.5;
        }
        else if (Math.abs(getError()) > 45) {
            power = 0.25;
        }
        else {power = 0.05;}
        if (getError() > 0) {power *= -1;}
        setPower(power);
    }
    public boolean isBusy() {return isBusy;}
    public boolean inError() {return Math.abs(targetPosition- getPosition()) > 15;}
    public double getPosition() {return rawPosition + (positionIteration * 360) + positionOffset;}
    public double getRawPosition() {return rawPosition;}
    public double getPower() {return power;}
    public double getError() {return targetPosition - getPosition();}
    private double encoderRead() {return encoder.getVoltage() / 3.3 * 360;}
    public void update() {
        //Run to Position Updating
        if (!inError()) {isBusy = false;}
        else {isBusy = true;}
        if (isBusy()) {powerToPos(targetPosition);}
        //Storing Position
        lastPosition = rawPosition;
        rawPosition = encoderRead();
        //Iteration Measuring
        if (Math.abs(rawPosition -lastPosition) > 180) {
            if (lastPosition > 180) {
                positionIteration++;
            } else {
                positionIteration--;
            }
        }
        //Scheduling Encoder Reset because of Read Latency
        if (!encoderHasReset && Math.abs(rawPosition) > 0) {
            positionOffset = -rawPosition;
            positionIteration = 0;
            encoderHasReset = true;
        }
    }
}
