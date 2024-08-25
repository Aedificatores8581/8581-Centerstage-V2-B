package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MyDcMotor {
    HardwareMap hardwareMap;
    DcMotorEx motor;
    public MyDcMotor(HardwareMap hw, String deviceName) {
        this.hardwareMap = hw;
        motor = hardwareMap.get(DcMotorEx.class, deviceName);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {motor.setZeroPowerBehavior(behavior);}
    public void runToPosition(int position, double power) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        setPower(power);
    }
    public int getPosition() {return motor.getCurrentPosition();}
    public int getTargetPosition() {return motor.getTargetPosition();}
    public void setPower(double power) {motor.setPower(power);}
    public void resetEncoder() {
        DcMotor.RunMode runMode = motor.getMode();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);
    }
}
