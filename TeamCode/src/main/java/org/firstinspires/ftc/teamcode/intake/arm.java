package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.hardware.MyDcMotor;
import org.firstinspires.ftc.teamcode.hardware.PIDController;
import org.firstinspires.ftc.teamcode.hardware.DualMotor;

import java.util.ArrayList;

@Config
public class arm {
    HardwareMap hardwareMap;
    public DualMotor motor;
    public MyDcMotor encoder;
    int targetPosition;
    int currentPosition;
    int currentError;
    int initialError = 0;
    public static int error = 40;
    public static double maxPower = 0.6;
    public static double kp = 0.005, ki = 0, kd = 0.0001, kf = 0;

    //double lastPower = 0;
    //double lastPower2 = 0;
    ArrayList<Double> previousPowers = new ArrayList<Double>();

    public Claw claw;
    enum runToMethods {
        PID,
        P
    }
    public runToMethods runToMethod = runToMethods.PID;
    PIDController armPID = new PIDController(kp,ki,kd, kf);
    //public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.00131,/*0.0014*/0.0025,0.0011);
    public arm(HardwareMap hw) {
        this.hardwareMap = hw;
        claw = new Claw(hardwareMap);
        motor = new DualMotor(hardwareMap, "leftArm", "rightArm");
        encoder = new MyDcMotor(hardwareMap, "arm");
        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDefaultEncoder(DualMotor.encoder.ONE);
        previousPowers.add(0.0);
        previousPowers.add(0.0);
        previousPowers.add(0.0);
        previousPowers.add(0.0);
    }
    public void runToPosition(int position) {
        runToPositionAsync(position);
        runTilIdle();
        updatePosition();
        if (isBusy()) {runToPosition(position);}
    }
    public void runToPositionAsync(int position) {
        targetPosition = position;updatePosition();initialError = currentError;
    }
    public void runTilIdle() {
        while (isBusy()) {
            update();
        }
    }
    public double powerToTarget() {return powerToPos(targetPosition);}
    public double powerToPos(int targetPosition) {
        double power;
        /*if (Math.abs(currentError) < Math.round(100+Math.abs(initialError/4))) {
            runToMethod=runToMethods.PID;
            power = -armPID.PIDControl(targetPosition, encoder.getPosition());

            if (pwrIsDown(power)) {power/=2;}
            else {power = (power*raiseMath()+power)/2;}
        } else {
            runToMethod=runToMethods.P;
            power = targetPosition - encoder.getPosition();
            power = power/32;
            power *= -1;
        }*/
        power = -armPID.PIDControl(targetPosition, encoder.getPosition());

        if (pwrIsDown(power)) {power/=2;}
        else {power = (power*raiseMath()+power)/2;}
        //power = (power+sumOfArray(previousPowers))/(previousPowers.toArray().length+1);
        if (Math.abs(power) > maxPower) {
            power = maxPower * (power / Math.abs(power));
        }

        motor.setPower(power);
        previousPowers.set(3, previousPowers.get(2));
        previousPowers.set(2, previousPowers.get(1));
        previousPowers.set(1, previousPowers.get(0));
        previousPowers.set(0, power);
        return power;
    }
    public void resetEncoder() {encoder.resetEncoder();motor.resetEncoder();}
    public boolean isBusy() {
        if (Math.abs(getTargetPos() - getPos()) < error) {return false;}
        else {return true;}
    }
    public void update() {
        updatePosition();
        powerToTarget();
        armPID.setPID(kp,ki,kd, kf);
    }
    public void updatePosition() {currentPosition = encoder.getPosition(); currentError = getPos() - getTargetPos();}
    public int getPos() {return currentPosition;}
    public int getError() {return currentError;}
    public int getTargetPos() {return targetPosition;}
    public void setPower(double power) {
        motor.setPower(power);}
    public double sumOfArray(ArrayList<Double> m) {
        double sum = 0;
        for(Double d : m)
            sum += d;
        return sum;
    }
    public double raiseMath() {
        final double tpd = 8192/360;
        double degree = (currentPosition+2900)/tpd;
        double multiplier = 0;
        multiplier = Math.abs(degree)-90;
        multiplier = 1-Math.abs(multiplier/90);
        return multiplier;
    }
    public int downPwr() {
        final double tpd = 8192/360;
        double degree = (currentPosition+2900)/tpd;

        double diff = degree - 180;
        if (Math.abs(degree) > 180)
            diff = 180 - degree;

        int downPower = (int) ((Math.floor(diff/180)+1)*-2-1);

        return downPower;
    }
    public boolean pwrIsDown(double power) {
        if ((downPwr() < 0 && power < 0) ||
                (downPwr() > 0 && power > 0)) {
            return true;
        }
        return false;
    }
}
