package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ButtonDown;
import org.firstinspires.ftc.teamcode.hardware.PIDController;
import org.firstinspires.ftc.teamcode.vision.AprilTagManager;

@Config
public class AprilTagDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0.09,  0.015, 0.0025),
            HEADING_PID = new PIDCoefficients(0.03, 0, 0.0001);
    public static double CONTINUOUS_DRIVE_TIME = 0.25;
    public static boolean STRAFE_COMPENSATION = true;
    HardwareMap hardwareMap;
    AprilTagManager at;
    SampleMecanumDrive drive;

    PIDController xPIDControl = new PIDController(),
            yPIDControl = new PIDController(),
            headingPIDControl = new PIDController();
    ElapsedTime noTagTime = new ElapsedTime();
    ButtonDown noTag = new ButtonDown(() -> {
        noTagTime.reset();
    });

    boolean isBusy = false;
    public enum robotStatus {
        DRIVING_TO_TAG,
        TAG_NOT_FOUND,
        IDLE
    }
    robotStatus currentStatus = robotStatus.IDLE;
    Pose2d error = new Pose2d(0,0,0);
    public AprilTagDrive(HardwareMap hw, SampleMecanumDrive drive) {
        this.drive = drive;
        this.hardwareMap = hw;
        at = new AprilTagManager(hardwareMap);
        at.buildPortal();
    }
    private void updatePIDFromConfig() {
        xPIDControl.setPID(TRANSLATIONAL_PID.kP,TRANSLATIONAL_PID.kI,TRANSLATIONAL_PID.kD);
        yPIDControl.setPID(TRANSLATIONAL_PID.kP,TRANSLATIONAL_PID.kI,TRANSLATIONAL_PID.kD);
        headingPIDControl.setPID(HEADING_PID.kP,HEADING_PID.kI,HEADING_PID.kD);
    }
    public void runToTag(int id, Pose2d offset) {
        at.update(id);
        double robotHeading = Math.toDegrees(drive.getPoseEstimate().getHeading());
        if (robotHeading > 180) {robotHeading = (robotHeading - 360);}
        final boolean tagFound = at.getTagCount() > 0;

        Pose2d drivePower = new Pose2d(0,0,0);

        if (noTagTime.seconds() < CONTINUOUS_DRIVE_TIME || tagFound) {
            updatePIDFromConfig();
            double xMultiplier = 1;
            if (STRAFE_COMPENSATION) {
                xMultiplier = 1.1;
            }
            error = new Pose2d(
                    at.targetTag.getX() - offset.getX(),
                    at.targetTag.getZ() - offset.getY(),
                    robotHeading - offset.getHeading());
            drivePower = new Pose2d(
                    yPIDControl.PIDControl(0, error.getY()),
                    -xPIDControl.PIDControl(0, error.getX()) * xMultiplier,
                    headingPIDControl.PIDControl(0, error.getHeading())
            );
            currentStatus = robotStatus.DRIVING_TO_TAG;
            isBusy = true;
        }
        else {
            currentStatus = robotStatus.TAG_NOT_FOUND;
        }
        drive.setWeightedDrivePower(drivePower);

        if (drivePower.getX() < 0.05 && drivePower.getY() < 0.05 && drivePower.getHeading() < 0.05) {
            isBusy = false; currentStatus = robotStatus.IDLE;
        }
    }
    public final AprilTagManager getAt() {return at;}
    public void stop() {
        currentStatus = robotStatus.IDLE;
        drive.setWeightedDrivePower(new Pose2d(0,0,0));
        isBusy = false;
    }

    public robotStatus getCurrentStatus() {
        return currentStatus;
    }
    public boolean isBusy() {return isBusy;}
    public Pose2d getError() {return error;}
}
