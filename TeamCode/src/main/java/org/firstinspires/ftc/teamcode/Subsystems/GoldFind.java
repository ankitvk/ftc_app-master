package org.firstinspires.ftc.teamcode.Subsystems;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import org.firstinspires.ftc.teamcode.Subsystems.Components.Drivetrain;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class GoldFind extends DogeCVDetector implements Constants {

    public Dogeforia dogeForia;
    public GoldAlignDetector detector;

    public AutonomousOpMode auto;
    public Hardware hardware;
    private Telemetry telemetry;
    private Drivetrain drivetrain;
    private HardwareMap hardwareMap;

    // Defining Mats to be used.
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)
    private Mat workingMat = new Mat(); // Used for preprocessing and working with (blurring as an example)
    private Mat Yellow = new Mat();
    private Mat hierarchy  = new Mat(); // hierarchy used by contours

    // Results of the detector
    private boolean found    = false; // Is the gold mineral found
    private boolean aligned  = false; // Is the gold mineral aligned
    private double  goldXPos = 0;     // X Position (in pixels) of the gold element

    // Detector settings
    private double alignPosOffset  = 0; // How far from center frame is aligned
    private double alignSize       = 1000;  // How wide is the margin of error for alignment

    private DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer


    //Create the default filters and scorers
    private DogeCVColorFilter yellowFilter      = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 85); //Default Yellow filter

    private RatioScorer ratioScorer       = new RatioScorer(1.0, 3);          // Used to find perfect squares
    private MaxAreaScorer maxAreaScorer     = new MaxAreaScorer( 0.01);                    // Used to find largest objects
    private PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000,0.05); // Used to find objects near a tuned area value

    /**
     * Simple constructor
     */
    public GoldFind(AutonomousOpMode auto, Hardware hardware) {
        super();
        detectorName = "GoldFinder"; // Set the detector name
        this.auto = auto;
        this.hardware = hardware;
        this.hardwareMap = hardware.getHwMap();
        telemetry = hardware.telemetry;
        drivetrain = new Drivetrain(hardware);
    }

    public GoldFind(Hardware hardware) {
        super();
        detectorName = "GoldFinder"; // Set the detector name
        this.hardware = hardware;
        telemetry = hardware.telemetry;
        drivetrain = new Drivetrain(hardware);
    }


    @Override
    public Mat process(Mat input) {
        boolean debugAlignment = true; // Show debug lines to show alignment settings


        // Copy the input mat to our working mats, then release it for memory
        input.copyTo(displayMat);
        input.copyTo(workingMat);
        input.release();


        //Preprocess the working Mat (blur it then apply a yellow filter)
        Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        yellowFilter.process(workingMat.clone(),Yellow);

        //Find contours of the yellow mask and draw them to the display mat for viewing

        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Imgproc.findContours(Yellow, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(230,70,70),2);

        // Current result
        Rect bestRect = null;
        double bestDifference = Double.MAX_VALUE; // MAX_VALUE since less diffrence = better

        // Loop through the contours and score them, searching for the best result
        for(MatOfPoint cont : contoursYellow){
            double score = calculateScore(cont); // Get the diffrence score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0,0,255),2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            if(score < bestDifference){
                bestDifference = score;
                bestRect = rect;
            }
        }

        // Vars to calculate the alignment logic.
        double alignX    = (getAdjustedSize().width / 2) + alignPosOffset; // Center point in X Pixels
        double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
        double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels
        double xPos; // Current Gold X Pos

        if(bestRect != null){
            // Show chosen result
            Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(255,0,0),4);
            Imgproc.putText(displayMat, "Chosen", bestRect.tl(),0,1,new Scalar(255,255,255));

            // Set align X pos
            xPos = bestRect.x + (bestRect.width / 2);
            goldXPos = xPos;

            // Draw center point
            Imgproc.circle(displayMat, new Point( xPos, bestRect.y + (bestRect.height / 2)), 5, new Scalar(0,255,0),2);

            // Check if the mineral is aligned
            aligned = xPos < alignXMax && xPos > alignXMin;

            // Draw Current X
            Imgproc.putText(displayMat,"Current X: " + bestRect.x,new Point(10,getAdjustedSize().height - 10),0,0.5, new Scalar(255,255,255),1);
            found = true;
        }else{
            found = false;
            aligned = false;
        }
        if(debugAlignment){

            //Draw debug alignment info
            if(isFound()){
                Imgproc.line(displayMat,new Point(goldXPos, getAdjustedSize().height), new Point(goldXPos, getAdjustedSize().height - 30),new Scalar(255,255,0), 2);
            }

            Imgproc.line(displayMat,new Point(alignXMin, getAdjustedSize().height), new Point(alignXMin, getAdjustedSize().height - 40),new Scalar(0,255,0), 2);
            Imgproc.line(displayMat,new Point(alignXMax, getAdjustedSize().height), new Point(alignXMax,getAdjustedSize().height - 40),new Scalar(0,255,0), 2);
        }

        //Print result
        Imgproc.putText(displayMat,"Result: " + aligned,new Point(10,getAdjustedSize().height - 30),0,1, new Scalar(255,255,0),1);


        return displayMat;

    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorer);

        // Add different scores depending on the selected mode
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }

    }

    /**
     * Set the alignment settings for GoldAlign
     * @param offset - How far from center frame (in pixels)
     * @param width - How wide the margin is (in pixels, on each side of offset)
     */
    public void setAlignSettings(int offset, int width){
        alignPosOffset = offset;
        alignSize = width;
    }

    public int getAlignOffset(){
        return (int)Math.round(alignPosOffset);
    }

    /**
     * Returns if the gold element is aligned
     * @return if the gold element is alined
     */
    public boolean getAligned(){
        return aligned;
    }

    /**
     * Returns gold element last x-position
     * @return last x-position in screen pixels of gold element
     */
    public double getXPosition(){
        return goldXPos;
    }

    /**
     * Returns if a gold mineral is being tracked/detected
     * @return if a gold mineral is being tracked/detected
     */
    public boolean isFound() {
        return found;
    }

    public void startOpenCV () {

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(),0,true);
        detector.useDefaults();
        detector.alignSize = 200;
        detector.alignPosOffset = 0;
        detector.downscale = .4;
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = LICENSE_KEY;
        parameters.fillCameraMonitorViewParent = true;
        parameters.cameraName = hardwareMap.get(WebcamName.class,"Webcam 1");
        dogeForia = new Dogeforia(parameters);
        dogeForia.enableConvertFrameToBitmap();
        dogeForia.setDogeCVDetector(detector);
        dogeForia.enableDogeCV();
        dogeForia.showDebug();
        dogeForia.start();

        /*init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        useDefaults();
        // Optional Tuning
        alignSize = 5; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        downscale = 0.4; // How much to downscale the input frames
        areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //falcon.goldAlignDetector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        maxAreaScorer.weight = 0.005;
        ratioScorer.weight = 5;
        ratioScorer.perfectRatio = 1.0;
        enable();*/
    }

    public void startOpenCVPhone(){
        init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        useDefaults();
        // Optional Tuning
        alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        downscale = 0.4; // How much to downscale the input frames
        areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //falcon.goldAlignDetector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        maxAreaScorer.weight = 0.005;
        ratioScorer.weight = 5;
        ratioScorer.perfectRatio = 1.0;
        enable();
    }




    public void alignGold(){
        PIDController getTheGold = new PIDController(.0025,0,0,1);
        long startTime = System.nanoTime();
        long beginTime = startTime;
        long stopState = 0;
        while(opModeIsActive() && (stopState <= 1000)){
            double power = getTheGold.power(TARGET_GOLD_X_POS,getXPosition());
            telemetry.addLine("PIDAlign");
            telemetry.addData("Aligned:",getAligned());
            telemetry.addData("Found:",isFound());
            telemetry.addData("Pos:",getXPosition());
            telemetry.addData("Heading:",hardware.imu.getYaw());
            telemetry.addLine(" ");
            telemetry.addData("KP*error: ",getTheGold.returnVal()[0]);
            telemetry.addData("KI*i: ",getTheGold.returnVal()[1]);
            telemetry.addData("KD*d: ",getTheGold.returnVal()[2]);
            telemetry.update();
            hardware.frontLeft.setPower(-power);
            hardware.backLeft.setPower(-power);
            hardware.frontRight.setPower(-power);
            hardware.backRight.setPower(-power);

            if (Math.abs(TARGET_GOLD_X_POS-getXPosition()) <= 25) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            if(System.nanoTime()/1000000-beginTime/1000000>10000){
                break;
            }
        }
        drivetrain.stop();
    }

    private boolean opModeIsActive() {
        return auto.getOpModeIsActive();
    }


}
