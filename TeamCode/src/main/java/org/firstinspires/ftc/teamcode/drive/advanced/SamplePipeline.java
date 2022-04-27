package org.firstinspires.ftc.teamcode.drive.advanced;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SamplePipeline extends OpenCvPipeline {
    private static final Scalar BLUE = new Scalar(0, 0, 255);

    private static final int THRESHOLD = 120;
    private static int zone = 0;
    private static int min_avg;

    Mat extract = new Mat();
    Mat luminosityMat = new Mat();

    private volatile int average;
    private volatile int average2;
    private volatile int average3;

    private static Point topLeft;
    private static Point bottomRight;
    private static Point topLeft1;
    private static Point bottomRight1;
    private static Point topLeft2;
    private static Point bottomRight2;

    Mat region1;
    Mat region2;
    Mat region3;
    
    private volatile TYPE type;

    private void inputToCb(Mat input) {
        Imgproc.cvtColor(input, extract, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(extract, luminosityMat, 2);

    }

    @Override
    public void init(Mat input) {
        inputToCb(input);

        topLeft = new Point(input.cols()*(0.5f/12f), input.rows()*(2f/12f));
        bottomRight = new Point(input.cols()*(3.5f/12f), input.rows()*(8f/12f));
        topLeft1 = new Point(input.cols()*(4.5f/12f), input.rows()*(2f/12f));
        bottomRight1 = new Point(input.cols()*(7.5f/12f), input.rows()*(8f/12f));
        topLeft2 = new Point(input.cols()*(8.5f/12f), input.rows()*(2f/12f));
        bottomRight2 = new Point(input.cols()*(11.5f/12f), input.rows()*(8f/12f));

        region1 = luminosityMat.submat(new Rect(topLeft, bottomRight));
        region2 = luminosityMat.submat(new Rect(topLeft1,bottomRight1));
        region3 = luminosityMat.submat(new Rect(topLeft2,bottomRight2));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);

        average = (int) Core.mean(region1).val[0];
        average2 = (int) Core.mean(region2).val[0];
        average3 = (int) Core.mean(region3).val[0];

        Imgproc.rectangle(input, topLeft, bottomRight, BLUE, 2);
        Imgproc.rectangle(input, topLeft1, bottomRight1, BLUE, 2);
        Imgproc.rectangle(input, topLeft2, bottomRight2, BLUE, 2);

        min_avg = Math.min(average,Math.min(average2,average3));

        if(min_avg > THRESHOLD)
        {
            type = TYPE.NONE;
            zone = 0;
        }
        else if(min_avg == average)
        {
            type = TYPE.ZONE1;
            zone = 1;
        }
        else if(min_avg == average2)
        {
            type = TYPE.ZONE2;
            zone = 2;
        }
        else if(min_avg == average3)
        {
            type = TYPE.ZONE3;
            zone = 3;
        }

        return input;
    }

    public int getZone() {
        return zone;
    }

    public enum TYPE {
        ZONE1,ZONE2,ZONE3,NONE
    }
}
