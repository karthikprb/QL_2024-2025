package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PixelDetector extends OpenCvPipeline {
    Mat HSVmat = new Mat();
    Mat contoursOnFrameMat = new Mat();

    List<MatOfPoint> contoursList = new ArrayList<>();

    int numContoursFound = 0;
    //OLD FREIGHT FRENZY DUCK YELLOW
    public Scalar lowerHSV = new Scalar(46.8, 66.6, 80.8);
    public Scalar upperHSV = new Scalar(102.0, 255, 255);

    public static double lowerH = 160;
    public static double lowerS = 110;
    public static double lowerV = 80;
    public static double upperH = 180;
    public static double upperS = 255;
    public static double upperV = 255;
    //GOOD RED SCALAR
    public Scalar lowerRedHsv = new Scalar (160, 110, 80);
    public Scalar upperRedHsv = new Scalar (180,255,255);

    public double threshold = 500;
    public double horizon = 100;

    public double blurConstant = 1;

    public double dilationConstant = 2;

    int pixelPosition = 0;

    Telemetry telemetryOpenCV = null;

    public PixelDetector(Telemetry OpModeTelemetry){
        telemetryOpenCV = OpModeTelemetry;
    }

    public int getPixelPosition(){
        return pixelPosition;
    }

    @Override
    public Mat processFrame(Mat input) {
        //clear
        contoursList.clear();
        Imgproc.cvtColor(input, HSVmat, Imgproc.COLOR_RGB2HSV_FULL);

        //filters out all the colors but the yellow
        Core.inRange(HSVmat,lowerRedHsv, upperRedHsv, HSVmat);

        Size kernelSize = new Size(blurConstant, blurConstant);

        //blurs the image
        Imgproc.GaussianBlur(HSVmat, HSVmat, kernelSize, 0);

        Size kernelSize2 =  new  Size(2 * dilationConstant + 1, 2 * dilationConstant + 1);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize2);

        Imgproc.dilate(HSVmat, HSVmat, kernel);

        //finding the contours
        Imgproc.findContours(HSVmat, contoursList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        numContoursFound = contoursList.size();
        input.copyTo(contoursOnFrameMat);

        for(MatOfPoint contour : contoursList){
            Rect rect = Imgproc.boundingRect(contour);

            if(rect.area() >= threshold && rect.y >= horizon){
                Imgproc.rectangle(contoursOnFrameMat, rect.tl(), rect.br(), new Scalar(255, 0, 0), 2);
                Imgproc.putText(contoursOnFrameMat, String.valueOf(rect.x), rect.tl(), 0, 0.5, new Scalar(255, 255, 255));

                if(rect.x > 400){
                    pixelPosition = 2;
                }else if(rect.x >= 300){
                    pixelPosition = 1;
                }else{
                    pixelPosition = 0;
                }
            }
        }

        return contoursOnFrameMat;
    }
}