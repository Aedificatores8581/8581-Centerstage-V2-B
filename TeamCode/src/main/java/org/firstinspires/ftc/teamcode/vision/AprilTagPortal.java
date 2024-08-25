package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagPortal {
    boolean USE_WEBCAM;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;

    int id = -1;
    int tagCount = 0;

    double x = 0;
    double y = 0;
    double z = 0;
    double yaw = 0;
    double pitch = 0;
    double roll = 0;

    HardwareMap hardwareMap;

    public AprilTagPortal(HardwareMap hw) {
        hardwareMap = hw;
    }
    public void init(String webcamName) {
        USE_WEBCAM = true;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create an AprilTagProcessor.Builder.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        myAprilTagProcessorBuilder.setTagLibrary(new AprilTagLibrary.Builder()
                .addTag(0, "April Tag Name",
                8.125, new VectorF(0,0,0), DistanceUnit.INCH,
                Quaternion.identityQuaternion())
                .build());

        // Create an AprilTagProcessor by calling build.
        myAprilTagProcessor = myAprilTagProcessorBuilder
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, webcamName));

            myVisionPortalBuilder.setCameraResolution(new Size(640, 480));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }
    public void update() {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();

        tagCount = JavaUtil.listLength(myAprilTagDetections);
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            // Display info about the detection.
            if (myAprilTagDetection.metadata != null) {
                id = myAprilTagDetection.id;
                x = myAprilTagDetection.rawPose.x;
                y = myAprilTagDetection.rawPose.y;
                z = myAprilTagDetection.rawPose.z;
                yaw = myAprilTagDetection.ftcPose.yaw;
                pitch = myAprilTagDetection.ftcPose.pitch;
                roll = myAprilTagDetection.ftcPose.roll;
            } else {
                id = myAprilTagDetection.id;
            }
        }
        if (tagCount == 0) {
            id = -1;
        }
    }

    public int getId() {
        return id;
    }
    public int getTagCount() {
        return tagCount;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getZ() {
        return z;
    }
    public double getYaw() {
        return yaw;
    }
    public double getPitch() {
        return pitch;
    }
    public double getRoll() {
        return roll;
    }
}
