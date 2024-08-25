package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;
import java.util.concurrent.atomic.AtomicReference;

public class propProcessor implements VisionProcessor, CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private Mat hsvImage;
    private Mat bgrImage;
    private Mat threshold;
    private Mat thresholdAfterROI;
    private Mat dilation;
    private Mat erosion;
    private Mat kernel;
    private Rect roi;

    private Mat colSum;

    private Mat leftImage;
    private Mat centerImage;
    private Mat rightImage;

    private Mat leftBinary;
    private Mat centerBinary;
    private Mat rightBinary;

    //private int elementPos = 0;
    public enum position {
        LEFT,
        RIGHT,
        CENTER
    }
    public boolean visualizeRecognition = false;
    position defaultElementPosition = position.LEFT;
    position currentElementPosition;

    private int[] colSumArray;

    private long imageSum = 0;


    private int screenWidth, screenHeight;

    public String allianceColor = "red";
    public static int
            RH_MIN = 0,
            RS_MIN = 0,
            RV_MIN = 0,
            RH_MAX = 0,
            RS_MAX = 0,
            RV_MAX = 0;
    public static int
            BH_MIN = 0,
            BS_MIN = 0,
            BV_MIN = 0,
            BH_MAX = 0,
            BS_MAX = 0,
            BV_MAX = 0;

    public propProcessor(int screenWidth, int screenHeight) {
        this.screenWidth = screenWidth;
        this.screenHeight = screenHeight;

        int HueVariation = 35,
                SaturationVariation = 55,
                ValueVariation = 35;
        File cameraDetection = AppUtil.getInstance().getSettingsFile("propCalibration.txt");
        try {
//Yellow
            Scanner scan = new Scanner(cameraDetection);
            RH_MIN = Integer.parseInt(scan.nextLine()) - HueVariation;
            RS_MIN = Integer.parseInt(scan.nextLine()) - SaturationVariation;
            RV_MIN = Integer.parseInt(scan.nextLine()) - ValueVariation;
            RH_MAX = RH_MIN + HueVariation*2;
            RS_MAX = RS_MIN + SaturationVariation*2;
            RV_MAX = RV_MIN + ValueVariation*2;
//Purple
            BH_MIN = Integer.parseInt(scan.nextLine()) - HueVariation;
            BS_MIN = Integer.parseInt(scan.nextLine()) - SaturationVariation;
            BV_MIN = Integer.parseInt(scan.nextLine()) - ValueVariation;
            BH_MAX = BH_MIN + HueVariation*2;
            BS_MAX = BS_MIN + SaturationVariation*2;
            BV_MAX = BV_MIN + ValueVariation*2;
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        hsvImage = new Mat();
        bgrImage = new Mat();
        threshold = new Mat();
        thresholdAfterROI = new Mat();
        dilation = new Mat();
        erosion = new Mat();

        leftImage = new Mat();
        centerImage = new Mat();
        rightImage = new Mat();
        leftBinary = new Mat();
        centerBinary = new Mat();
        rightBinary = new Mat();

        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));

        colSum = new Mat();
        colSumArray = new int[screenWidth];
        roi = new Rect(screenWidth/3, screenHeight/3, (int)(1.0/3*screenWidth - 1), 3*screenHeight/12);
    }
    public void setAlliance(String alliance) {allianceColor = alliance;}

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Mat inputTemp = new Mat();
        input.copyTo(inputTemp);
        //Core.rotate(input, input, ROTATE_90_COUNTERCLOCKWISE);
        // For whatever reason, OpenCV requires you to do some weird acrobatics to convert
        // RGBA to HSV, that's what is done here
        Imgproc.cvtColor(inputTemp, bgrImage, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(bgrImage, hsvImage, Imgproc.COLOR_BGR2HSV_FULL);

        // Threshold hsv values
        if (allianceColor.toLowerCase() == "red") {
            Core.inRange(hsvImage,
                    new Scalar(RH_MIN, RS_MIN, RV_MIN),
                    new Scalar(RH_MAX, RS_MAX, RV_MAX),
                    threshold);
        } else if (allianceColor.toLowerCase() == "blue") {
            Core.inRange(hsvImage,
                    new Scalar(BH_MIN, BS_MIN, BV_MIN),
                    new Scalar(BH_MAX, BS_MAX, BV_MAX),
                    threshold);
        }
        Imgproc.erode(threshold, erosion, kernel);
        Imgproc.dilate(erosion, dilation, kernel);
        Imgproc.erode(dilation, erosion, kernel);
        Imgproc.dilate(erosion, dilation, kernel);
        thresholdAfterROI = dilation.submat(roi);

        Imgproc.cvtColor(dilation, bgrImage, Imgproc.COLOR_GRAY2BGR);
        Imgproc.cvtColor(bgrImage, inputTemp, Imgproc.COLOR_BGR2RGBA);
        //Imgproc.rectangle(input, roi, new Scalar(0, 255, 0), 4);

        Core.reduce(thresholdAfterROI, colSum, 0, Core.REDUCE_SUM, CvType.CV_32S);

        long sum = 0;
        colSum.get(0, 0, colSumArray);
        for (int i = 0; i < screenWidth; ++i) {
            sum += colSumArray[i];
        }

        this.setImageSum(sum);

        int imageWidth = inputTemp.cols() / 3;
        leftImage = new Mat(inputTemp, new Rect(0, 0, imageWidth, inputTemp.rows()));
        centerImage = new Mat(inputTemp, new Rect(imageWidth, 0, imageWidth, inputTemp.rows()));
        rightImage = new Mat(inputTemp, new Rect(imageWidth * 2, 0, imageWidth, inputTemp.rows()));

        Core.extractChannel(leftImage, leftBinary, 0);
        Core.extractChannel(centerImage, centerBinary, 0);
        Core.extractChannel(rightImage, rightBinary, 0);

        int leftCount = Core.countNonZero(leftBinary);
        int centerCount = Core.countNonZero(centerBinary);
        int rightCount = Core.countNonZero(rightBinary);
        if (visualizeRecognition)
            inputTemp.copyTo(input); //(DISPLAY DOTS)
        if (leftCount > centerCount && leftCount > rightCount) {
            currentElementPosition = position.LEFT;
            return leftImage;
        } else if (centerCount > rightCount) {
            currentElementPosition = position.CENTER;
            return centerImage;
        } else if (centerCount < rightCount) {
            currentElementPosition = position.RIGHT;
            return rightImage;
        } else {
            currentElementPosition = defaultElementPosition;
            return inputTemp;
        }

    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    protected void finalize() throws Throwable {
        close();
        super.finalize();
    }

    public void setImageSum(long imageSum) {
        this.imageSum = imageSum;
    }

    public long getImageSum() {
        return imageSum;
    }

    public position getElementPosition() {
        return currentElementPosition;
    }

    public void close() {
        hsvImage.release();
        bgrImage.release();
        threshold.release();
        thresholdAfterROI.release();
        dilation.release();
        erosion.release();
        kernel.release();
        colSum.release();
    }
}
