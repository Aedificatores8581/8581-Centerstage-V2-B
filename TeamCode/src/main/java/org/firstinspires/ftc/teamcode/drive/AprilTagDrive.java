package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.PIDController;
import org.firstinspires.ftc.teamcode.vision.AprilTagManager;

@Config
public class AprilTagDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0.09,  0.015, 0.0025),
            HEADING_PID = new PIDCoefficients(0.03, 0, 0.0001);
    HardwareMap hardwareMap;
    AprilTagManager at;
    SampleMecanumDrive drive;

    PIDController xPIDControl,
            yPIDControl,
            headingPIDControl;


    boolean isBusy = false;
    public AprilTagDrive(HardwareMap hw, SampleMecanumDrive drive) {
        this.hardwareMap = hw;
        this.drive = drive;
        at = new AprilTagManager(hardwareMap);
        at.buildPortal();

        xPIDControl = new PIDController();
        yPIDControl = new PIDController();
        headingPIDControl = new PIDController();
    }
    private void updatePIDFromConfig() {
        xPIDControl.setPID(TRANSLATIONAL_PID.kP,TRANSLATIONAL_PID.kI,TRANSLATIONAL_PID.kD);
        yPIDControl.setPID(TRANSLATIONAL_PID.kP,TRANSLATIONAL_PID.kI,TRANSLATIONAL_PID.kD);
        headingPIDControl.setPID(HEADING_PID.kP,HEADING_PID.kI,HEADING_PID.kD);
    }
    public void runToTag(int id, Pose2d offset) {
        double robotHeading = drive.getPoseEstimate().getHeading();
    }
}
