package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

/**
 * Used to detect cone position.
 */
public class ObjectDetection extends OpenCvPipeline {

    // TOP-LEFT anchor point for the bounding box
    public static Point Object_TOPLEFT_POINT = new Point(0, 0);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 320;
    public static int REGION_HEIGHT = 240;

    // Color definitions
    private final Scalar
            RED   = new Scalar(255, 0, 0),
            GREEN = new Scalar(0, 255, 0),
            BLUE  = new Scalar(0, 0, 255);

    // Anchor point definitions
    Point pointA = new Point(
            Object_TOPLEFT_POINT.x,
            Object_TOPLEFT_POINT.y);
    Point pointB = new Point(
            Object_TOPLEFT_POINT.x + REGION_WIDTH,
            Object_TOPLEFT_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile double objectPosition = 0;

    @Override
    public Mat processFrame(Mat input) {
        Logging.log("Start Opcv process to detect cone.");

        // Get the submat frame, and then sum all the values

        Mat areaMat = input.submat(new Rect(pointA, pointB));

        Logging.log("areaMat has %d channels, and type is %d, depth is %d.",
                areaMat.channels(), areaMat.type(), areaMat.depth());

        Mat doubleAreaMat = new Mat(areaMat.size(), 30); // CV_64FC3, CV_64FC4=30
        areaMat.convertTo(doubleAreaMat, 30);
        Logging.log("areaMat has %d channels, and type is %d, depth is %d.",
                doubleAreaMat.channels(), doubleAreaMat.type(), doubleAreaMat.depth());

        int kernelSize = 32;
        Mat destMat = new Mat(doubleAreaMat.rows(), doubleAreaMat.cols(), doubleAreaMat.type());

        double [] filterK = new double[doubleAreaMat.channels()];
        for (int i = 0; i < doubleAreaMat.channels(); i++)
        {
            filterK[i] = 1.0/kernelSize/kernelSize;
        }


        Mat kernel = new Mat(kernelSize, kernelSize, doubleAreaMat.type() ) {
            {
                for (int i = 0; i < kernelSize; i++) {
                    for (int j = 0; j < kernelSize; j++) {
                        put(i, j, filterK);
                    }
                }
            }
        };
        Logging.log("kernel(16,16)[0] = %.4f", kernel.get(16,16)[0]);
        Logging.log("kernel(16,16)[1] = %.4f", kernel.get(16,16)[1]);

        Imgproc.filter2D(doubleAreaMat, destMat, -1, kernel);

        Mat lineMatA = destMat.row(0);
        Mat lineM = new Mat(lineMatA.size(), lineMatA.type());
        for (int i = 1; i < destMat.rows(); i++) {
            Mat lineMatB = destMat.row(i);
            Core.add(lineMatA, lineMatB, lineM);
            lineMatA = lineM;
        }
        Logging.log("lineM has %d channel, and type is %d, size = %d x %d, depth = %d.",
                lineM.channels(), lineM.type(), lineM.rows(), lineM.cols(),lineM.depth());

        Scalar sumColors = Core.sumElems(lineM);

        double [][] lineArray= new double[lineM.cols()][lineM.channels()];

        // lineM should only have one row.
        double [] maxPixelVal = new double[lineM.channels()];
        double [] minPixelVal = {lineM.get(0, 0)[0], lineM.get(0, 0)[1],
                lineM.get(0, 0)[2], lineM.get(0, 0)[3]};
        double [] avePixelVal = new double[lineM.channels()];
        int [] maxPixelLoc = new int[lineM.channels()];

        // find max value and its index in array for each channel
        for (int j = 0; j < lineM.cols(); j++) {
            for (int k = 0; k < lineM.channels(); k++) {
                lineArray[j][k] = lineM.get(0, j)[k];
                if (maxPixelVal[k] < lineM.get(0, j)[k]) {
                    maxPixelVal[k] = lineM.get(0, j)[k];
                    maxPixelLoc[k] = j;
                }

                if (minPixelVal[k] > lineM.get(0, j)[k]) {
                    minPixelVal[k] = lineM.get(0, j)[k];
                }

                Logging.log("Pixel values(%d, %d)[%d] = %.2f", 0, j, k, lineM.get(0, j)[k]);
            }
        }

        double maxChannel = sumColors.val[0];
        int maxCh = 0;
        for (int k = 0; k < lineM.channels(); k++) {
            avePixelVal[k] = sumColors.val[k]/lineM.cols()/lineM.rows();
            Logging.log("Channel[%d] min = %.2f, max = %.2f, sum = %.2f",
                    k, minPixelVal[k], maxPixelVal[k], avePixelVal[k]);
            Logging.log("Channel[%d] max location = %d", k, maxPixelLoc[k]);

            if((maxChannel < sumColors.val[k]) && (k < (lineM.channels()-1))) {
                maxChannel = sumColors.val[k];
                maxCh = k;
            }
        }


        //check if cone has been detected.
        double cameraToConeDistance = 13.0; // inch
        double cameraViewAngle = 0.67;


        Logging.log("detected channel is %d, location pixel is %d", maxCh, maxPixelLoc[maxCh] );
        objectPosition = (maxPixelLoc[maxCh] * 2.0 / lineM.cols()) * Math.tan(cameraViewAngle/2.0) * cameraToConeDistance;
        Logging.log("detected channel is %d, location pixel is %d, distance is %.2f", maxCh, maxPixelLoc[maxCh], objectPosition );

        Imgproc.rectangle(
                input,
                pointA,
                pointB,
                GREEN,
                2
        );


        Imgproc.line(input, new Point(maxPixelLoc[0], 0), new Point(maxPixelLoc[0], input.rows()), RED);
        Imgproc.line(input, new Point(maxPixelLoc[2], 0), new Point(maxPixelLoc[2], input.rows()), BLUE);

        // Release and return input
        areaMat.release();
        lineM.release();
        kernel.release();
        doubleAreaMat.release();
        destMat.release();

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public double getPosition() {
        return objectPosition;
    }
}
