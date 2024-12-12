package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.Vision.SleeveDetectorV2Constants.BOUNDING_BOX;
import static org.firstinspires.ftc.teamcode.Vision.VisionConstants.horizon;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;


public class pixels extends OpenCvPipeline {
    Mat HSVMat = new Mat();
    Mat contoursOnFrameMat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    int numContoursFound = 0;

    public int pixelCase;

    Rect pixelRect = new Rect();


    int area1 =0;
    int area2 =0;
    int area3 =0;

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
        Scalar lowerBlue = new Scalar(pixelss.lowerBlueH,pixelss.lowerBlueS,pixelss.lowerBlueV);
        Scalar upperBlue = new Scalar(pixelss.upperBlueH,pixelss.upperBlueS,pixelss.upperBlueV);
        contoursList.clear();
        contThree.clear();
        Imgproc.cvtColor(input,HSVMat, Imgproc.COLOR_BGR2HSV_FULL);

        Mat blueMask = new Mat();

        Core.inRange(HSVMat, lowerBlue, upperBlue, blueMask);

        Imgproc.findContours(blueMask, contoursList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        numContoursFound = contoursList.size();
        input.copyTo(contoursOnFrameMat);

        for(MatOfPoint contour : contoursList){
            Moments moments = Imgproc.moments(contour);

            double area = Imgproc.contourArea(contour);

            Point center = calculateCentroid(moments);
            if(area>= pixelss.threshold){
                contThree.add(contour);
            }
        }

        contThree.sort(new sortArr());




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

            if(Imgproc.contourArea(contThree.get(1))>600){
                pixelCase = 1;
            } else if (Imgproc.contourArea(contThree.get(0))>600) {
                pixelCase = 2;
            }else{
                pixelCase = 3;
            }

        }

        return contoursOnFrameMat;

    }

}
class sortArr implements Comparator<MatOfPoint> {
    public int compare(MatOfPoint a, MatOfPoint b)
    {
        Moments aMom = Imgproc.moments(a);
        Moments bMom = Imgproc.moments(b);
        return Double.compare(pixels.calculateCentroid(aMom).x, pixels.calculateCentroid(bMom).x);
    }
}
@Config
class pixelss{
    public static int lowerBlueH = 0;
    public static int lowerBlueS = 110;
    public static int lowerBlueV = 10;
    public static int upperBlueH = 130;
    public static int upperBlueS = 255;
    public static int upperBlueV =255;

    public static int threshold = 300;
}
