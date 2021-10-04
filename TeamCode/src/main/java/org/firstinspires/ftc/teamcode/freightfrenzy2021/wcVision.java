package org.firstinspires.ftc.teamcode.freightfrenzy2021;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.BlockingQueue;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;


public class wcVision {

    private LinearOpMode opMode;
    private VuforiaLocalizer vuforia;
    private Parameters parameters;
    private CameraDirection CAMERA_CHOICE = CameraDirection.BACK;

    private final int RED_THRESHOLD = 20;
    private final int GREEN_THRESHOLD = 20;
    private final int BLUE_THRESHOLD = 20;

    private final double widthFactor = 1280.0 / 3264;
    private final double heightFactor = 720.0 / 1836;
    private final int xdiff = 650;
    private final int yheight = 300;


    private static final String VUFORIA_KEY = "AeFSc8v/////AAABmX+Pinx9j0k6hQOygQElLa1v9CMRX9dH3SsSruSA5mg7UiNXNkpeWWSVy9dezEwHXnc1l4ppFWFGjhlE8IUM6WR7LPMsX5PpNItWn3imKy2iLgQbw1/vU3zmNaItZJPUwDH/TCJW+ifvorBYRL6Ohu63V6z4tL8UABgktWq05oXJ4I7RwFbO47nYuA5CZNe6eHXmWaC/kECXH8B1n8gjsvrwD8NLj0SJcn21Auw/SxcxnpoVs079o9p7Sw/CuKJwqDa1fWfiWzTnsWfVTyxdP/VU2ahsGhg/6LEtAmvx3SewpDsqlmgEIYQnh6RnDPZ74G5CgamQ1XZKR73HVJio7gKaAXcNhxxZ10iqcFoQihj1";

    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frame;

    public static String bitmapSkyStonePosition;

    public wcVision(LinearOpMode opMode) {

        this.opMode = opMode;

        int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        Parameters params = new Parameters(cameraMonitorViewId);

        params.vuforiaLicenseKey = VUFORIA_KEY;
        params.cameraDirection = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(params);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
        vuforia.enableConvertFrameToBitmap();

    }
/*
    public BitMapVision(LinearOpMode opMode) {
        this.opMode = opMode;
        int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = VUFORIA_KEY;
        params.cameraDirection = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(4); //tells VuforiaLocalizer to only store one frame at a time
    }*/

    public Bitmap getImage() throws InterruptedException {
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        long numImages = frame.getNumImages();
        Image rgb = null;
        for (int i = 0; i < numImages; i++) {
            Image img = frame.getImage(i);
            int fmt = img.getFormat();
            if (fmt == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());
        return bm;
    }

    public Bitmap getBitmap() throws InterruptedException {

        VuforiaLocalizer.CloseableFrame picture;
        picture = vuforia.getFrameQueue().take();
        Image rgb = picture.getImage(1);

        long numImages = picture.getNumImages();

        opMode.telemetry.addData("Num Images", numImages);
        opMode.telemetry.update();

        for (int i = 0; i < numImages; i++) {
            int format = picture.getImage(i).getFormat();
            if (format == PIXEL_FORMAT.RGB565) {
                rgb = picture.getImage(i);
                break;

            } else {
                opMode.telemetry.addLine("Didn't find correct RGB format");
                opMode.telemetry.update();


            }

        }

        Bitmap imageBitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        imageBitmap.copyPixelsFromBuffer(rgb.getPixels());

        opMode.telemetry.addData("Image width", imageBitmap.getWidth());
        opMode.telemetry.addData("Image height", imageBitmap.getHeight());
        opMode.telemetry.update();


        picture.close();

        opMode.telemetry.addLine("Got bitmap");
        opMode.telemetry.update();

        return imageBitmap;
    }

    public String sample() throws InterruptedException {
        Bitmap bitmap = getBitmap();
        String bitmapCubePosition;

        ArrayList<Integer> xValues = new ArrayList<>();

        int avgX = 0;

        //top left = (0,0)
        for (int colNum = 0; colNum < bitmap.getWidth(); colNum += 2) {

            for (int rowNum = 0; rowNum < (int) (bitmap.getHeight()); rowNum += 3) {
                int pixel = bitmap.getPixel(colNum, rowNum);

                int redPixel = red(pixel);
                int greenPixel = green(pixel);
                int bluePixel = blue(pixel);

                if (redPixel <= RED_THRESHOLD && greenPixel <= GREEN_THRESHOLD && bluePixel <= BLUE_THRESHOLD) {
                    xValues.add(colNum);

                }

            }

        }

        for (int x : xValues) {
            avgX += x;
        }

        avgX /= xValues.size();

        if (avgX < (bitmap.getWidth() / 3.0)) {
            bitmapCubePosition = "left";

        } else if (avgX > (bitmap.getWidth() / 3.0) && avgX < (bitmap.getWidth() * 2.0 / 3)) {
            bitmapCubePosition = "center";

        } else {
            bitmapCubePosition = "right";

        }

        opMode.telemetry.addData("Cube Position", bitmapCubePosition);
        opMode.telemetry.update();
        return bitmapCubePosition;
    }

    public String findRedSkystones() throws InterruptedException {
        Bitmap bitmap = getBitmap();
        String bitmapCubePosition;

        int stone1 = bitmap.getPixel((int) (603 * widthFactor), (int) (1670 * heightFactor));//bitmap.getWidth() * 2/5, 20
        int redVal1 = red(stone1);

        int stone2 = bitmap.getPixel((int) (1350 * widthFactor), (int) (1670 * heightFactor));//bitmap.getWidth()/2, 20
        int redVal2 = red(stone2);

        int stone3 = bitmap.getPixel((int) (2090 * widthFactor), (int) (1670 * heightFactor));//bitmap.getWidth() * 3/5, 20
        int redVal3 = red(stone3);

        ArrayList<Integer> vals = new ArrayList<Integer>();
        vals.add(redVal1);
        vals.add(redVal2);
        vals.add(redVal3);

        int min = Collections.min(vals);
        int pos = vals.indexOf(min);

        if (pos == 0) {
            bitmapCubePosition = "left";
        } else if (pos == 1) {
            bitmapCubePosition = "center";
        } else if (pos == 2) {
            bitmapCubePosition = "right";
        } else {
            bitmapCubePosition = "yikes";
        }
        /*
        telemetry.addData("redval1", redVal1);
        telemetry.addData("redval2", redVal2);
        telemetry.addData("redval3", redVal3);
        telemetry.addData("left", vals.get(0));
        telemetry.addData("center", vals.get(1));
        telemetry.addData("right", vals.get(2));
        telemetry.update();
        sleep(5000);*/
        return bitmapCubePosition;
    }

    public String findBlueSkystones() throws InterruptedException {
        Bitmap bitmap = getBitmap();
        String bitmapCubePosition;

        int stone1 = bitmap.getPixel((int) (989 * widthFactor), (int) (1677 * heightFactor));//bitmap.getWidth() * 2/5, 20
        int redVal1 = red(stone1);

        int stone2 = bitmap.getPixel((int) (1745 * widthFactor), (int) (1677 * heightFactor));//bitmap.getWidth()/2, 20
        int redVal2 = red(stone2);

        int stone3 = bitmap.getPixel((int) (2500 * widthFactor), (int) (1677 * heightFactor));//bitmap.getWidth() * 3/5, 20
        int redVal3 = red(stone3);

        ArrayList<Integer> vals = new ArrayList<Integer>();
        vals.add(redVal1);
        vals.add(redVal2);
        vals.add(redVal3);

        int min = Collections.min(vals);
        int pos = vals.indexOf(min);

        if (pos == 0) {
            bitmapCubePosition = "left";
        } else if (pos == 1) {
            bitmapCubePosition = "center";
        } else if (pos == 2) {
            bitmapCubePosition = "right";
        } else {
            bitmapCubePosition = "yikes";
        }
        /*
        telemetry.addData("redval1", redVal1);
        telemetry.addData("redval2", redVal2);
        telemetry.addData("redval3", redVal3);
        telemetry.addData("left", vals.get(0));
        telemetry.addData("center", vals.get(1));
        telemetry.addData("right", vals.get(2));
        telemetry.update();
        sleep(5000);*/
        return bitmapCubePosition;
    }

    public Bitmap vufConvertToBitmap(Frame frame) {
        return vuforia.convertFrameToBitmap(frame);
    }

    public void initBitmap() throws InterruptedException {
        Bitmap bitmap = getBitmap();
    }

    public String rbgVals(int xCoord, int yCoord) throws InterruptedException {
        Bitmap bitmap = getBitmap();
        int stone1 = bitmap.getPixel((int) (xCoord * widthFactor), (int) (yCoord * heightFactor));//bitmap.getWidth() * 2/5, 20
        int redVal1 = red(stone1);
        int greenVal1 = green(stone1);
        int blueVal1 = blue(stone1);
        String color = null;
        if (redVal1 >= 120 && redVal1 <= 160 && greenVal1 >= 120 && greenVal1 <= 160 && blueVal1 >= 120 && blueVal1 <= 160) {
            color = "yellow";
        } else {
            color = "not yellow";
        }

        return "red: " + redVal1 + " blue: " + blueVal1 + " green: " + greenVal1 + " the color is " + color;
    }

    public String floorAvg() throws InterruptedException {
        Bitmap bitmap = getBitmap();
        int floor1 = bitmap.getPixel((int) (953 * widthFactor), (int) (473 * heightFactor));//bitmap.getWidth()/2, 20
        int redFloor1 = red(floor1);
        int greenFloor1 = green(floor1);
        int blueFloor1 = blue(floor1);

        int floor2 = bitmap.getPixel((int) (840 * widthFactor), (int) (460 * heightFactor));//bitmap.getWidth()/2, 20
        int redFloor2 = red(floor2);
        int greenFloor2 = green(floor2);
        int blueFloor2 = blue(floor2);

        int floor3 = bitmap.getPixel((int) (810 * widthFactor), (int) (510 * heightFactor));//bitmap.getWidth()/2, 20
        int redFloor3 = red(floor3);
        int greenFloor3 = green(floor3);
        int blueFloor3 = blue(floor3);


        double floorAvgRed = (double) (redFloor1 + redFloor2 + redFloor3) / 3;
        double floorAvgBlue = (double) (blueFloor1 + blueFloor2 + blueFloor3) / 3;

        return "red floor: " + floorAvgRed + "blue floor: " + floorAvgBlue;
    }
}
