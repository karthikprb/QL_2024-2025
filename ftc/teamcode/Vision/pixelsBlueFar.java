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


public class pixelsBlueFar extends OpenCvPipeline {
    Mat HSVMat = new Mat();
    Mat contoursOnFrameMat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    int numContoursFound = 0;


    public int pixelCase = 0;
    public double blurConstant = 1;
    public double dilationConstant = 4;

    ArrayList<MatOfPoint> contThree = new ArrayList<>();

    public static Point calculateCentroid(Moments moments) {
        double x = moments.m10 / moments.m00;
        double y = moments.m01 / moments.m00;
        return new Point(x, y);
    }




    public int getPixelPosition(){
        return pixelCase;
    }


    @Override
    public Mat processFrame(Mat input) {

        //Config Scalars
        Scalar lowerBlue = new Scalar(pixelssBlueFar.lowerBlueH,pixelssBlueFar.lowerBlueS,pixelssBlueFar.lowerBlueV);
        Scalar upperBlue = new Scalar(pixelssBlueFar.upperBlueH,pixelssBlueFar.upperBlueS,pixelssBlueFar.upperBlueV);

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
            if(area>= pixelssBlueFar.threshold && center.y>=pixelssBlueFar.horizon){
                contThree.add(contour);
            }
        }

        //sort by X val
        contThree.sort(new sortArrBlue());




        if(contThree.size() >= 3){
            Moments mom_1 = Imgproc.moments(contThree.get(0));
            Moments mom_2 = Imgproc.moments(contThree.get(1));
            Moments mom_3 = Imgproc.moments(contThree.get(2));

            Point centro1 = calculateCentroid(mom_1);
            Point centro2 = calculateCentroid(mom_2);
            Point centro3 = calculateCentroid(mom_3);

            Imgproc.putText(contoursOnFrameMat,"1(" + centro1.x + ")", centro1, 0, 0.5, new Scalar(255, 255, 255));
            Imgproc.putText(contoursOnFrameMat,"2(" + centro2.x + ")", centro2, 0, 0.5, new Scalar(255, 255, 255));
            Imgproc.putText(contoursOnFrameMat,"3(" + centro3.x + ")", centro3, 0, 0.5, new Scalar(255, 255, 255));

            Imgproc.drawContours(contoursOnFrameMat, contThree, 0, new Scalar(255,255,255));
            Imgproc.drawContours(contoursOnFrameMat, contThree, 1, new Scalar(255,255,255));
            Imgproc.drawContours(contoursOnFrameMat, contThree, 2, new Scalar(255,255,255));


            if(Imgproc.contourArea(contThree.get(0))>4900){
                pixelCase = 2;
            }else if(Imgproc.contourArea(contThree.get(1))>2000){
                pixelCase = 1;
            }else{
                pixelCase = 0;
            }
        }


        return contoursOnFrameMat;

    }

}

//Comparator
class sortArrBlueFar implements Comparator<MatOfPoint> {
    public int compare(MatOfPoint a, MatOfPoint b)
    {
        Moments aMom = Imgproc.moments(a);
        Moments bMom = Imgproc.moments(b);
        if(pixelsBlueFar.calculateCentroid(aMom).x > pixelsBlueFar.calculateCentroid(bMom).x){
            return 1;
        }else{
            return -1;
        }
    }
}

//dash values
@Config
class pixelssBlueFar{
    public static int lowerBlueH = 0;
    public static int lowerBlueS = 110;
    public static int lowerBlueV = 20;
    public static int upperBlueH = 140;
    public static int upperBlueS = 255;
    public static int upperBlueV =255;

    public static int threshold = 150;

    public static int horizon = 130;

}