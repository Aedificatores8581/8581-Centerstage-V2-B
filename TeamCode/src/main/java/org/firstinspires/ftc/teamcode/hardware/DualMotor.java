package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class DualMotor {
    HardwareMap hardwareMap;
    MyDcMotor motor1;
    MyDcMotor motor2;

    public enum encoder {ONE, TWO, BOTH}
    encoder defaultEncoder;
    public DualMotor(HardwareMap hw, String motor1Name, String motor2Name) {
        this.hardwareMap = hw;
        motor1 = new MyDcMotor(hardwareMap, motor1Name);
        motor2 = new MyDcMotor(hardwareMap, motor2Name);
        setDefaultEncoder(encoder.BOTH);
    }
    public void setDefaultEncoder(encoder encoder) {
        defaultEncoder = encoder;
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {motor1.setZeroPowerBehavior(behavior); motor2.setZeroPowerBehavior(behavior);}
    public void runToPosition(int position, double power) {
        motor1.runToPosition(position, power);
        motor2.runToPosition(position, power);
    }
    public List<Integer> getPositions() {
        List<Integer> positions = new ArrayList<>();
        positions.add(motor1.getPosition());
        positions.add(motor2.getPosition());
        return positions;}
    public int getPosition() {
        switch(defaultEncoder) {
            case BOTH:
                return (motor1.getPosition() + motor2.getPosition()) / 2;
            case ONE:
                return motor1.getPosition();
            case TWO:
                return motor2.getPosition();
        }
        return (motor1.getPosition() + motor2.getPosition()) / 2;
    }
    public List<Integer> getTargetPositions() {
        List<Integer> positions = new ArrayList<>();
        positions.add(motor1.getTargetPosition());
        positions.add(motor2.getTargetPosition());
        return positions;
    }
    public int getTargetPosition() {
        switch(defaultEncoder) {
            case BOTH:
                return (motor1.getTargetPosition() + motor2.getTargetPosition())/2;
            case ONE:
                return motor1.getTargetPosition();
            case TWO:
                return motor2.getTargetPosition();
        }
        return (motor1.getTargetPosition() + motor2.getTargetPosition())/2;
    }
    public void setPower(double power) {motor1.setPower(power);motor2.setPower(power);}
    public void resetEncoder() {motor1.resetEncoder();motor2.resetEncoder();}
}
