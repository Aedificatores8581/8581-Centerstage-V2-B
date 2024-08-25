package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class callNewPipeline {
    HardwareMap hardwareMap;

    VisionPortal.Builder myVisionPortalBuilder;
    VisionPortal myVisionPortal;

    public AprilTagProcessor tagProcessor;
    public propProcessor propProcessor;

    int screenWidth = 640;
    int screenHeight = 360;
    public callNewPipeline(HardwareMap hm) {hardwareMap = hm;}

    public void start() {
        propProcessor = new propProcessor(screenWidth, screenHeight);
        //at = new aprilTagProcessor(hardwareMap);
        // Create a VisionPortal, with the specified camera and AprilTag processor, and assign it to a variable.
        //myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), propProcessor);
        VisionPortal.Builder myVisionPortalBuilder;

// Create a new VisionPortal Builder object.
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        //AprilTagProcessor tagProcessor = at.init("Webcam 1");
        //myVisionPortalBuilder.addProcessor(tagProcessor);
        myVisionPortalBuilder.addProcessor(propProcessor);

        myVisionPortalBuilder.setCameraResolution(new Size(640, 360));
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        myVisionPortalBuilder.setAutoStopLiveView(true);
        myVisionPortal = myVisionPortalBuilder.build();
    }
    public propProcessor.position getElementPos() {
        return propProcessor.getElementPosition();
    }
    public void setAlliance(String alliance) {propProcessor.setAlliance(alliance);}
    public void stop() {
        myVisionPortal.stopStreaming();
        myVisionPortal.close();
    }

}
