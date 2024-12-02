package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;


public class stackLocalizer extends OpenCvPipeline {
    Mat HSVMat = new Mat();
    Mat contoursOnFrameMat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    int numContoursFound = 0;


    public double pixelX = 0;
    public double blurConstant = 1;
    public double dilationConstant = 4;

    ArrayList<MatOfPoint> contThree = new ArrayList<>();

    public static Point calculateCentroid(Moments moments) {
        double x = moments.m10 / moments.m00;
        double y = moments.m01 / moments.m00;
        return new Point(x, y);
    }




    public double getStackPosition(){
        return pixelX;
    }


    @Override
    public Mat processFrame(Mat input) {

        //Config Scalars
        Scalar lowerBlue = new Scalar(stackLocalizerVals.lowerBlueH,stackLocalizerVals.lowerBlueS,stackLocalizerVals.lowerBlueV);
        Scalar upperBlue = new Scalar(stackLocalizerVals.upperBlueH,stackLocalizerVals.upperBlueS,stackLocalizerVals.upperBlueV);

        //List clears
        contoursList.clear();
        contThree.clear();

        //HSV conversion
        Imgproc.cvtColor(input,HSVMat, Imgproc.COLOR_BGR2HSV_FULL);

        //Dilate -> Erode -> Blur -> Mask

        //kernels
        Size kernelSize2 =  new  Size(2 * dilationConstant + 1, 2 * dilationConstant + 1);
        Size kernelSize3 = new Size(dilationConstant*2/3+1,dilationConstant*2/3+1);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize2);
        Mat kernel2 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,kernelSize3);
        Size kernelSize = new Size(blurConstant, blurConstant);
        /*
        //Dilation
        Imgproc.dilate(HSVMat, HSVMat, kernel);

        //Erosion

        Imgproc.erode(HSVMat,HSVMat,kernel2);

        //Blurs the image
        Imgproc.GaussianBlur(HSVMat, HSVMat, kernelSize, 0);
        */
        //Mask
        Mat blueMask = new Mat();

        Core.inRange(HSVMat, lowerBlue, upperBlue, blueMask);

        Imgproc.findContours(blueMask, contoursList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


        numContoursFound = contoursList.size();

        //for debug
        input.copyTo(contoursOnFrameMat);


        //adding eligible contours to contThree arrayList
        for(MatOfPoint contour : contoursList){
            Moments moments = Imgproc.moments(contour);

            double area = moments.m00;

            Point center = calculateCentroid(moments);
            if(area>= stackLocalizerVals.threshold && center.y>=stackLocalizerVals.horizon){
                contThree.add(contour);
            }
        }

        //sort by X val
        contThree.sort(new sortStack());


            Moments mom_1 = Imgproc.moments(contThree.get(0));


            Point centro1 = calculateCentroid(mom_1);


            Imgproc.putText(contoursOnFrameMat,"1(" + centro1.x + ")", centro1, 0, 0.5, new Scalar(255, 255, 255));

            Imgproc.drawContours(contoursOnFrameMat, contThree, 0, new Scalar(255,255,255));


            pixelX = centro1.x;




        return contoursOnFrameMat;

    }

}

//Comparator
class sortStack implements Comparator<MatOfPoint> {
    public int compare(MatOfPoint a, MatOfPoint b)
    {
        Moments sortA = Imgproc.moments(a);
        Moments sortB = Imgproc.moments(b);

        double aArea = sortA.m00;
        double bArea = sortB.m00;


        if(aArea > bArea){
            return 1;
        }else{
            return -1;
        }
    }
}

//dash values
@Config
class stackLocalizerVals{
    public static int lowerBlueH = 0;
    public static int lowerBlueS = 130;
    public static int lowerBlueV = 20;
    public static int upperBlueH = 140;
    public static int upperBlueS = 255;
    public static int upperBlueV =255;

    public static int threshold = 150;

    public static int horizon = 130;

}