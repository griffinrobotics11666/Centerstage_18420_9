package org.firstinspires.ftc.teamcode.Pipelines;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.sql.Array;
import java.util.ArrayList;

/***
 * This example shows how to use an OpenCvPipeline to process an image to draw a contour around an object.  It works by doing the following steps.
 * 0. Establish variables to hold onto a series of images (Mats) at different stages of analysis.
 * 1. Establish Scalar variables to establish upper and lower values for a range mask
 * 2. Make erode and dilate elements for the erosion and dilation of an image (to blur it to remove artifacts).
 *
 * PROCESSFRAME
 * 3. Make an ArrayList to hold onto all contours
 * 4. Take input image and convert it into a different color space.  HSV color space is often easier for some colors.
 * 5. Make an image that is a mask of the converted image.  Each pixel is either 255 or 0 depending on if it is in range, isolating important parts from background.
 * 6. Erodes an image thereby reducing noise, but shrinks any edges down
 * 7. Dilates an image which blows the edges back up to the previous size.
 * 8. Finds contours around the mask.
 * 9. Copies the original input image to a new place to later draw on top of it for viewing.
 * 10. Calculates the "moments" of the contours and uses them to find the center of the object.
 * 11. Calculates a bounding box around the contours.
 * 12. Draws the bounding box and outputs data about where the center is.
 */
@Config
public class ContoursPixelLocatorBLUE extends OpenCvPipeline {
    //TODO
    //do all of the things for WebcamTestBLUE and ContoursPixelLocatorBLUE

    //necessary bits for making telemetry work in a pipeline!
    Telemetry telemetry;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    TelemetryPacket packet = new TelemetryPacket();

    public ContoursPixelLocatorBLUE(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    //imgs
    Mat clrConvertedMat = new Mat(); //mat after color conversion
    Mat thresholdMat1 = new Mat(); //mask -- y pixels are part of thing or n they are not
    Mat morphedThreshold1 = new Mat();  //denoise
    Mat contoursOnPlainImageMat = new Mat(); //copy of original image to vandalize with contours
    Mat narrowedThresholdMat = new Mat();

    //config variable(s) for images
    public static Point submatBound1 = new Point(639,479);
    public static Point submatBound2 = new Point(0,220);
    public static Point testPoint = new Point(0,0);
    //Boolean for team (Red = T, Blue = F)
    public Boolean teamColor = true;
    public Boolean toggleReturnedMat = true;
    public enum ConePosition {LEFT,CENTER,RIGHT}
    ConePosition coneposition = ConePosition.CENTER;

    //upper and lower Scalar values for changing the range for the mask
    public static int lowerH = 80; //TODO NEED TO CHANGE THIS
    public static int lowerS = 180;
    public static int lowerV = 80;

    public static int upperH = 130;
    public static int upperS = 255;
    public static int upperV = 220;

    //moved the variables to make them editable in FTC Dashboard.
    //Upper and Lower Scalar values are in the process frame so that they are updated each frame rather than set once at the start of the code.


    public static int LEFT_BOUND = 640/3;
    public static int RIGHT_BOUND = 1280/3;

    //for list size sorting:
    public int currentEsize;
    public int biggestEsize;
    public int currentIsize;
    public int biggestIsize;


    //sizes to adjust for the erosion and dilation of the mask
    public int size = 3;
    public int mult = 2;

    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(size, size));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(mult*size, mult*size));

    @Override
    public Mat processFrame(Mat input) {
        Scalar lower1 = new Scalar(lowerH,lowerS,lowerV);
        Scalar upper1 = new Scalar(upperH,upperS,upperV);
        teamColor = true; //reinit teamColor default RED
        // Executed every time a new frame is dispatched
        //Array to hold the contours
        ArrayList<MatOfPoint> contoursList = new ArrayList<>();
        ArrayList<Moments> momentsList = new ArrayList<>();
        ArrayList<Point> centers = new ArrayList<>();
        ArrayList<Rect> rectList = new ArrayList<>();


        //convert color space to limit what you are seeing

        Imgproc.cvtColor(input, clrConvertedMat, Imgproc.COLOR_RGB2HSV);
        //input.copyTo(grayMat);
        //Make a mask using a range.  Includes only the values within that range
        Core.inRange(clrConvertedMat,lower1,upper1,thresholdMat1);
        //erode the image - reducing the noisy pixels from the mask
        //by taking the lowest value in a 3x3 box (erodeElement)
        Imgproc.erode(thresholdMat1, morphedThreshold1, erodeElement);
        Imgproc.erode(morphedThreshold1, morphedThreshold1, erodeElement);
        //expands the edges of the mask out again (needed because erosion reduces both noise and signal
        //and this boosts the signal of the edges)
        Imgproc.dilate(morphedThreshold1, morphedThreshold1, dilateElement);
        Imgproc.dilate(morphedThreshold1, morphedThreshold1, dilateElement);
        //use function to find contours of each object and put them into contoursList.
        //find contours on the mask of all of the edges of the same value that make a closed shape.
        //These contours are points indicated on an image of their own, stored in contoursList.
        //Each object that is detected is stored as a separate object in contoursList.
        //We want to reduce this down to 1 object.
        narrowedThresholdMat = morphedThreshold1.submat(new Rect(submatBound1, submatBound2));

        //                  Mask Image        ListToStore  Optional   Mode                  Method
        //Mode and method are standard in example code.  Play around with other options
        //Imgproc.findContours(morphedThreshold1,contoursList, new Mat(),Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(narrowedThresholdMat, contoursList, new Mat(), Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_NONE);

        //copy "input" image to draw contours on for display purposes.  This just copies input to that place.
        input.copyTo(contoursOnPlainImageMat);

        //draw contours using the contours List
        //                  Image to draw on        contours2draw           ?       Color           Thickness
        Imgproc.drawContours(contoursOnPlainImageMat,contoursList,-1,new Scalar(0,0,255),2);

        //counter for the number of closed contours
        telemetry.addData("Number of objects", contoursList.size());

        //Sometimes images have no contours!  The list is empty, so if you try to grab an object from it
        //It yells at you.  This makes sure the list isn't empty before you access it!
        //this grabs the first contour and finds the moments of it.

        for (MatOfPoint contour : contoursList) {
            momentsList.add(Imgproc.moments(contour));  //make a moment for each contour and put it in the list
            centers.add(new Point(
                    (int) (momentsList.get(momentsList.size() - 1).m10 / momentsList.get(momentsList.size() - 1).m00),
                    (int) (momentsList.get(momentsList.size() - 1).m01 / momentsList.get(momentsList.size() - 1).m00))
            );  //calculates the centers and puts them into the centers list
            rectList.add(Imgproc.boundingRect(contour));
        }

        currentEsize = 0;
        biggestEsize = 0;
        currentIsize = 0;
        biggestIsize = 0;

        //check for biggest object height
        for (Rect r : rectList) {
            currentEsize = r.height;
            if (currentEsize > biggestEsize) {
                biggestEsize = currentEsize;
                biggestIsize = currentIsize;
            }
            currentIsize++;
        }

        //to do:
        //display the center of the biggest object
        //Do something cool with that info
        //Output relevant telemetry

        //check x location of the tallest obj
        //ignores left 5 pixels to split into 3 sections evenly

        if(contoursList.size() != 0) {
            if (centers.get(biggestIsize).x < LEFT_BOUND) {
                coneposition = ConePosition.LEFT;
            } else if (centers.get(biggestIsize).x >= LEFT_BOUND && centers.get(biggestIsize).x <= RIGHT_BOUND) {
                coneposition = ConePosition.CENTER;
            } else if (centers.get(biggestIsize).x > RIGHT_BOUND) {
                coneposition = ConePosition.RIGHT;
            }
            Imgproc.circle(contoursOnPlainImageMat, centers.get(biggestIsize), 3, new Scalar(0, 255, 0), 2);

            Imgproc.boundingRect(contoursList.get(biggestIsize));
            Imgproc.rectangle(contoursOnPlainImageMat, submatBound1, submatBound2, new Scalar(255, 0, 0), 2);
            telemetry.addData("width of box", rectList.get(biggestIsize).width);
            //At this point, we could try to estimate the distance of the box by a ratio of apparent size to expected.
            //The bigger the box width, the smaller the distance.
            //if we know a size of the box at a specific distance, if the box width doubles, then the distance halfs.

            //this draws a box around the image to show you where the computer things the object is based on "center"

            //String centers = "Center 1: " + center1.x + ", " + center1.y + " Center 2: " + center2.x + ", " + center2.y;

            telemetry.addData("center x location: ", centers.get(biggestIsize).x);
            telemetry.addData("center y location: ", centers.get(biggestIsize).y);
            telemetry.addData("getAnalysis result (center point): ", centers.get(biggestIsize));

        }
        telemetry.addData("Cone Position: ", coneposition);
        packet.put("Cone Position", coneposition);
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);


        Imgproc.line(contoursOnPlainImageMat,new Point(0,220),new Point(639,220),new Scalar(255,0,0), 2);
        Imgproc.line(contoursOnPlainImageMat,new Point(0,479),new Point(639,479),new Scalar(255,0,0), 2);
        Imgproc.line(contoursOnPlainImageMat,new Point(LEFT_BOUND,0),new Point(LEFT_BOUND,479),new Scalar(255,0,0), 2);
        Imgproc.line(contoursOnPlainImageMat,new Point(RIGHT_BOUND,0),new Point(RIGHT_BOUND,479),new Scalar(255,0,0), 2);
        //this returns the image with the contours
        if (toggleReturnedMat) {
            return contoursOnPlainImageMat; // Return the image that will be displayed in the viewport
        }
        else {
            return contoursOnPlainImageMat.submat(new Rect(submatBound1, submatBound2));
        }

        //uncomment these to see what they look like
        //return input;
        //return thresholdMat;
        //return morphedThreshold;

    }

    public ConePosition getPropPosition() {
        return coneposition;
    }



    Mat getContoursOnPlainImageMat() {
        return contoursOnPlainImageMat;
    }
    @Override
    public void onViewportTapped() {
        // Executed when the image display is clicked by the mouse or tapped
        // This method is executed from the UI thread, so be careful to not
        // perform any sort heavy processing here! Your app might hang otherwise
    }

    //make an analysis output thing with an enum

}