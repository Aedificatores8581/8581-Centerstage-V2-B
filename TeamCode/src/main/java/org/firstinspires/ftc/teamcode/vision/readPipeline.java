package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Color;
import android.webkit.HttpAuthHandler;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class readPipeline extends OpenCvPipeline
{
    Mat output = new Mat();
    Mat temp = new Mat();
    double HValue, SValue, VValue;
    @Override
    public Mat processFrame(Mat input)
    {
        //Imgproc.blur(input,output,new Size(3,30));

        Point topLeft = new Point( input.cols() / 2-20, input.rows() / 2-20 );
        Point bottomRight = new Point(topLeft.x + 20, topLeft.y+20 );

        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(temp, output, Imgproc.COLOR_RGB2HSV_FULL);

        Mat a = new Mat( output, new Rect( topLeft, bottomRight));
        Scalar b = Core.mean(a);

        String HVal = String.format("%1.2f", b.val[0]);
        String SVal = String.format("%1.2f",  b.val[1]);
        String VVal = String.format("%1.2f",  b.val[2]);
        String c = HVal + "," + SVal + "," + VVal;

        HValue = Double.parseDouble(HVal);
        SValue = Double.parseDouble(SVal);
        VValue = Double.parseDouble(VVal);

        Imgproc.putText( input, c, new Point(40,40), 0, 1, new Scalar( 255, 0, 0), 2 );

        Imgproc.rectangle(
                input,
                topLeft,
                bottomRight,
                getContrastingScalar(Float.parseFloat(HVal), Float.parseFloat(SVal), Float.parseFloat(VVal)), 4);
        /*topLeft.x += 2;topLeft.y += 2;
        bottomRight.x -= 2;bottomRight.y -= 2;
        int[] rgbValues = hsvRgb(Float.parseFloat(HVal)/255, Float.parseFloat(SVal)/255, Float.parseFloat(VVal)/255);
        Scalar fillColor = new Scalar(rgbValues[0], rgbValues[1], rgbValues[2]);
        Imgproc.rectangle(
                input,
                topLeft,
                bottomRight,
                fillColor, -1);*///Code for Color Average Display (Error was Wrong Color)

        return input;
    }
    public int getElementPosition() {
        return 0;
    }
    public double getH() {return HValue;}
    public double getS() {return SValue;}
    public double getV() {return VValue;}
    public String hsvToRgb(float hue, float saturation, float value) {

        int h = (int)(hue * 6);
        float f = hue * 6 - h;
        float p = value * (1 - saturation);
        float q = value * (1 - f * saturation);
        float t = value * (1 - (1 - f) * saturation);

        switch (h) {
            case 0: return rgbToString(value, t, p);
            case 1: return rgbToString(q, value, p);
            case 2: return rgbToString(p, value, t);
            case 3: return rgbToString(p, q, value);
            case 4: return rgbToString(t, p, value);
            case 5: return rgbToString(value, p, q);
            default: throw new RuntimeException("Something went wrong when converting from HSV to RGB. Input was " + hue + ", " + saturation + ", " + value);
        }
    }
    public String rgbToString(float r, float g, float b) {
        String rs = Integer.toHexString((int)(r * 256));
        String gs = Integer.toHexString((int)(g * 256));
        String bs = Integer.toHexString((int)(b * 256));
        return rs + gs + bs;
    }
    public static int[] hsvRgb(float hue, float saturation, float value) {
        int r, g, b;
        int i;
        float f, p, q, t;

        if (saturation == 0) {
            r = g = b = (int) (value * 255.0f);
        } else {
            hue = hue % 1.0f;
            i = (int) (hue * 6);
            f = hue * 6 - (int) hue;
            p = value * (1.0f - saturation);
            q = value * (1.0f - saturation * f);
            t = value * (1.0f - saturation * (1 - f));

            switch (i) {
                case 0:
                    r = (int) (value * 255.0f);
                    g = (int) (q * 255.0f);
                    b = (int) (p * 255.0f);
                    break;
                case 1:
                    r = (int) (t * 255.0f);
                    g = (int) (value * 255.0f);
                    b = (int) (p * 255.0f);
                    break;
                case 2:
                    r = (int) (p * 255.0f);
                    g = (int) (value * 255.0f);
                    b = (int) (q * 255.0f);
                    break;
                case 3:
                    r = (int) (p * 255.0f);
                    g = (int) (t * 255.0f);
                    b = (int) (value * 255.0f);
                    break;
                case 4:
                    r = (int) (q * 255.0f);
                    g = (int) (p * 255.0f);
                    b = (int) (value * 255.0f);
                    break;
                case 5:
                default:
                    r = (int) (value * 255.0f);
                    g = (int) (p * 255.0f);
                    b = (int) (t * 255.0f);
            }
        }

        return new int[]{r, g, b};
    }
    public static float[] hexToRgb(String hex) {
        // Remove any leading '#' character
        hex = hex.replace("#", "");

        // Parse the hexadecimal string and extract RGB components
        int red = Integer.parseInt(hex.substring(0, 2), 16);
        int green = Integer.parseInt(hex.substring(2, 4), 16);
        int blue = Integer.parseInt(hex.substring(4, 6), 16);

        // Convert RGB values to floats and return
        return new float[] {red / 255f, green / 255f, blue / 255f};
    }
    public Scalar getContrastingScalar(float hue, float saturation, float value) {
        // Convert HSV to RGB
        //String rgbHexString = hsvToRgb(hue, saturation, value);
        //float[] rgbValues = hexToRgb(rgbHexString);
        int[] rgbValues = hsvRgb(hue/255, saturation/255, value/255);

        // Get the contrasting RGB color
        float contrastR = 255 - rgbValues[0];
        float contrastG = 255 - rgbValues[1];
        float contrastB = 255 - rgbValues[2];

        // Convert the contrasting RGB color to scalar
        return new Scalar(contrastR, contrastG, contrastB);
    }
}