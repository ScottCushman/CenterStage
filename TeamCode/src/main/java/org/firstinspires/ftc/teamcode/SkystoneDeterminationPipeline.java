package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDeterminationPipeline extends OpenCvPipeline {
  //  public SkystoneDeterminationPipeline(HardwareMap hardwareMap, OpMode opMode) {}
    /*
     * An enum to define the skystone position
     */
     boolean square1Activated = false;
     boolean square2Activated = false;

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);


    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(300, 550);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(250, 100);
    //  static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(630,98);
    static final int REGION_WIDTH = 150;
    static final int REGION_HEIGHT = 150;
    boolean isRed;
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    /*
      Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_Cb, region2_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1, avg2;


    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input) {
        if (isRed) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        } else {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }
    }

    @Override
    public void init(Mat firstFrame) {

        inputToCb(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {

        inputToCb(input);


        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
        if (avg1 > 140) {
            square1Activated = true;
            square2Activated = false;
        } else if (avg2 > 140) {
            square2Activated = true;
            square1Activated = false;
        } else {
            square1Activated = false;
            square2Activated = false;
        }
        return input;
    }
    public int getAnalysis()
    {
        return avg1;
        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */

        /*
         * Draw a solid rectangle on top of the chosen region.
         * Simply a visual aid. Serves no functional purpose.
         */

            /*
            else if(max == avg3) // Was it from region 3?
            {
                position = SkystonePosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
            /*
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */

    }
}