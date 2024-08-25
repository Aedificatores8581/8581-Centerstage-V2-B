package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.ClawController;
import org.firstinspires.ftc.teamcode.hardware.ServoSets;

public class Claw {
    public ClawController left;
    public ClawController right;
    public ServoSets wrist;
    HardwareMap hardwareMap;
    public Claw(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        left = new ClawController(hardwareMap, "leftClaw", 0.04, 0.18);
        right = new ClawController(hardwareMap, "rightClaw", 0.15, 0.02);
        wrist = new ServoSets.servoPositionsBuilder(hardwareMap, "leftPivot")
                .addPosition("UP", 0.75)
                .addPosition("DOWN", 0.2)
                .addPosition("BACKDROP_PERPENDICULAR", 0.4)
                .build();
    }
    public void backdropParallel(int armPos) {
        double startPos = 1.00;
        double interval = -0.04;
        double pivotPosition = (armPos-1850);

        pivotPosition /= (50/interval);
        pivotPosition += startPos;
        wrist.setPositionRaw(pivotPosition, "BACKDROP");
    }
}
