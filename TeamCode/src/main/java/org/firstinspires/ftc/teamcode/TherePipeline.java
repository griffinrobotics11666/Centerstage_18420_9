package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.teamcode.EasyOpenCVExamples.PipelineStageSwitchingExample;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TherePipeline extends OpenCvPipeline {
    private static final Scalar BLUE = new Scalar (0,0,255);
    private static final Scalar GREEN = new Scalar (0,255,0);
    private static final Scalar RED = new Scalar (255,0,0);
    public enum PowerPlayPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    boolean viewportPaused = false;
    static final Point THEREBOX_TOPRIGHT_ANCHOR_POINT = new Point (175,110);
    static final int THEREBOX_WIDTH = 40;
    static final int THEREBOX_HEIGHT = 40;//was 20
    public String color = "RED";
    PowerPlayPosition coneState = PowerPlayPosition.LEFT; //default


    Point THEREBOX_point2 = new Point (
            THEREBOX_TOPRIGHT_ANCHOR_POINT.x + THEREBOX_WIDTH,
            THEREBOX_TOPRIGHT_ANCHOR_POINT.y + THEREBOX_HEIGHT);

    Mat THEREBOX_B;
    Mat RGB = new Mat();
    Mat B = new Mat();
    int avgB;

    Mat THEREBOX_G;
    Mat G = new Mat();
    int avgG;

    Mat THEREBOX_R;
    Mat R = new Mat();
    int avgR;

    void splitChannels(Mat input)
    {
        Core.extractChannel(input,R,0);
        Core.extractChannel(input,G,1);
        Core.extractChannel(input,B,2);
    }
    private volatile PowerPlayPosition WHERE = PowerPlayPosition.LEFT; //default = LEFT [hi this code isn't disgusting shut up (respectfully) jk lol]

    @Override
    public void init(Mat firstImage){
        splitChannels(firstImage);

        THEREBOX_R = R.submat(new Rect(THEREBOX_TOPRIGHT_ANCHOR_POINT, THEREBOX_point2));
        THEREBOX_G = G.submat(new Rect(THEREBOX_TOPRIGHT_ANCHOR_POINT, THEREBOX_point2));
        THEREBOX_B = B.submat(new Rect(THEREBOX_TOPRIGHT_ANCHOR_POINT, THEREBOX_point2));
    }

    @Override
    public Mat processFrame(Mat input){
        /* wow we're so cool */
        splitChannels(input);
        int[] avgs = new int[3];

        avgR = (int) Core.mean(THEREBOX_R).val[0];
        avgG = (int) Core.mean(THEREBOX_G).val[0];
        avgB = (int) Core.mean(THEREBOX_B).val[0];

        avgs[0] = avgR;
        avgs[1] = avgG;
        avgs[2] = avgB;

        Scalar thereColor = RED;

        int big1 = Math.max(avgR,avgG);
        int big2 = Math.max(big1,avgB);

        if (big2 == avgR){
            thereColor = RED;
            color = "RED";
            coneState = PowerPlayPosition.LEFT;

        }
        else if (big2 == avgG) {
            thereColor = GREEN;
            color = "GREEN";
            coneState = PowerPlayPosition.RIGHT;

        }
        else {
            thereColor = BLUE;
            color = "BLUE";
            coneState = PowerPlayPosition.CENTER;

        }

        Imgproc.rectangle(
                input,
                THEREBOX_TOPRIGHT_ANCHOR_POINT, THEREBOX_point2,
                thereColor , 2);

        return input;
    }
    public PowerPlayPosition getAnalysis (){
        return coneState;
    }
}