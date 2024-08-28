package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagManager {
    AprilTagProcessor tagProcessor;
    VisionPortal portal;
    int tagCount = 0;
    public tag lastTag = new tag();
    public tag targetTag = new tag();
    List<tag> tags = new ArrayList<tag>();
    HardwareMap hardwareMap;
    WebcamName webcam;
    public AprilTagManager(HardwareMap hw) {
        hardwareMap = hw;
        webcam = hardwareMap.get(WebcamName.class, "Global Cam");
        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        tagProcessor = myAprilTagProcessorBuilder
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(907.659, 907.659, 659.985, 357.874) // For Global Shutter Camera
                .build();
    }
    public void buildPortal() {
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true);
        portal = portalBuilder.build();
    }
    public void setPortal(VisionPortal visionPortal) {
        this.portal = visionPortal;
    }
    public void update(int... targetId) {
        int targId = -1;
        if (targetId.length > 0) targId = targetId[0];

        List<AprilTagDetection> tagDetectionList;
        AprilTagDetection tagDetection;
        tagDetectionList = tagProcessor.getDetections();
        tags.clear();
        tagCount = JavaUtil.listLength(tagDetectionList);

        int listIndex = 0;
        for (AprilTagDetection myAprilTagDetection_item : tagDetectionList) {
            tagDetection = myAprilTagDetection_item;

            lastTag.id = tagDetection.id;
            if (tagDetection.metadata != null) {
                lastTag = detectionToTag(tagDetection);
                if (tagDetection.id == targId) {
                    targetTag = detectionToTag(tagDetection);
                }
            }
            listIndex++;
        }
        if (tagCount == 0) {
            lastTag.id = -1;
        }
    }
    public int getTagCount() {
        return tagCount;
    }


    public class tag {
        public int id = -1;
        public double x = 0,
                y = 0,
                z = 0;
        public double yaw = 0,
                pitch = 0,
                roll = 0;
        public int getId() {return this.id;}
        public double getX() {return this.x;}
        public double getY() {return this.y;}
        public double getZ() {return this.z;}
        public double getYaw() {return this.yaw;}
        public double getPitch() {return this.pitch;}
        public double getRoll() {return this.roll;}
    }
    public tag detectionToTag(AprilTagDetection detection) {
        tag thisTag = new tag();
        thisTag.id = detection.id;
        thisTag.x = detection.rawPose.x;
        thisTag.y = detection.rawPose.y;
        thisTag.z = detection.rawPose.z;
        thisTag.yaw = detection.ftcPose.yaw;
        thisTag.pitch = detection.ftcPose.pitch;
        thisTag.roll = detection.ftcPose.roll;
        return thisTag;
    }
}
