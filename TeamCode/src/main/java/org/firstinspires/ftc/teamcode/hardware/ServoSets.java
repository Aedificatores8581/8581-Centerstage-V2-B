package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class ServoSets {
    private Servo servo;
    ArrayList<String> posNames = new ArrayList<String>();
    ArrayList<Double> positions = new ArrayList<Double>();
    HardwareMap hardwareMap;
    String currentPositionName;
    double currentPosition;
    public ServoSets(servoPositionsBuilder builder) {
        this.servo = builder.servo;
        this.hardwareMap = builder.hardwareMap;
        this.posNames = builder.posNames;
        this.positions = builder.positions;
    }
    public void goTo(String position) {
        int index = posNames.indexOf(position);
        servo.setPosition(positions.get(index));
        currentPositionName = position;
    }
    public String getPositionName() {
        return currentPositionName;
    }
    public double getPosition(String... posName) {
        if (posName.length > 0) {
            int index = posNames.indexOf(posName[0]);
            return positions.get(index);
        }
        return currentPosition;
    }
    public void setPositionRaw(double position, String... positionName) {
        servo.setPosition(position);
        currentPosition = position;
        currentPositionName = "{RAW_POSITION}";
        if (positionName.length > 0) {
            currentPositionName = positionName[0];
        }
    }

    //Builder Class
    public static class servoPositionsBuilder{
        private Servo servo;
        ArrayList<String> posNames = new ArrayList<String>();
        ArrayList<Double> positions = new ArrayList<Double>();
        HardwareMap hardwareMap;
        public servoPositionsBuilder(HardwareMap hardwareMap, String deviceName){
            servo = hardwareMap.get(Servo.class, deviceName);
        }

        public servoPositionsBuilder addPosition(String name, double position) {
            this.posNames.add(name);
            this.positions.add(position);
            return this;
        }

        public ServoSets build(){
            return new ServoSets(this);
        }

    }
}
