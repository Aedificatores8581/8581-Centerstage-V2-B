package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Misc;

public class PIDController {
    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;
    public PIDController() {this.Kp = this.Ki = this.Kd = 0;}
    public PIDController(double p, double i, double d, double... Kf) {
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
        if (Kf.length > 0) {this.Kf = Kf[0];}
    }
    @Override public String toString()
    {
        return Misc.formatForUser("%s(Kp=%f Ki=%f Kd=%f)", getClass().getSimpleName(), Kp, Ki, Kd);
    }
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        if (Kf != 0) {output += (reference*Kf);}
        return output;
    }
    public void setPID(double p, double i, double d, double... Kf) {
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
        if (Kf.length > 0) {this.Kf = Kf[0];}
    }
}
