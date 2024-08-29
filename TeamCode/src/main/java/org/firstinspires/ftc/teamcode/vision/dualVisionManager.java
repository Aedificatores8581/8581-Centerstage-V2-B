package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.AprilTagDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

public class dualVisionManager {
    HardwareMap hardwareMap;
    VisionPortal myVisionPortal;

    public AprilTagDrive atDrive;
    public propProcessor propProcessor;

    int screenWidth = 640;
    int screenHeight = 360;
    public dualVisionManager(HardwareMap hm) {hardwareMap = hm;}

    public void start(SampleMecanumDrive drive) {
        List myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        int Portal_1_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false)).intValue();
        int Portal_2_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 1, false)).intValue();

        propProcessor = new propProcessor(screenWidth, screenHeight);
        atDrive = new AprilTagDrive(hardwareMap, drive,Portal_1_View_ID);
        VisionPortal.Builder myVisionPortalBuilder = myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortalBuilder.addProcessor(propProcessor);
        myVisionPortalBuilder.setCameraResolution(new Size(640, 360));
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        myVisionPortalBuilder.setAutoStopLiveView(true);
        myVisionPortalBuilder.setLiveViewContainerId(Portal_2_View_ID);
        myVisionPortal = myVisionPortalBuilder.build();
    }
    public propProcessor.position getElementPos() {
        return propProcessor.getElementPosition();
    }
    public void setAlliance(String alliance) {propProcessor.setAlliance(alliance);}
    public AprilTagDrive atDrive() {return atDrive;}
    public void stop() {
        myVisionPortal.stopStreaming();
        myVisionPortal.close();
    }

}
