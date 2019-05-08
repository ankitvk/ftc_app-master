package ftc.library.MaelstromUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import ftc.library.MaelstromWrappers.MaelstromLinearOp;

/*utils class containing clipping and normalizing values*/
public class MaelstromUtils implements LibConstants {

    private static MaelstromLinearOp linearOpMode;
    public static double DEFAULT_SLEEP_TIME = 0;
    public static double DEFAULT_TIMEOUT = 10;
    public static double DEFAULT_STOPSTATE = 10;
    public static double DEFAULT_SPEED = 1;

    public static String LEFT_FRONT_KEY = "leftFront";
    public static String LEFT_BACK_KEY = "leftBack";
    public static String RIGHT_FRONT_KEY = "rightFront";
    public static String RIGHT_BACK_KEY = "rightBack";

    private static double deltaTime = 0;
    private static double deltaPos = 0;
    private static double currTime = 0;
    private static long prevTime = 0;


    public static void sleep (int sleep) {
        try {Thread.sleep(sleep);}
        catch (InterruptedException e) {e.printStackTrace();}
    }
    public static void sleep (double sleep) {
        try {Thread.sleep((long) sleep);}
        catch (InterruptedException e) {e.printStackTrace();}
    }

    public static double clipValue (double input, double clipRange) {

        double clippedValue = Math.min(clipRange, Math.max(-clipRange, input));

        return clippedValue;
    }

    public static void clipValue (double[] input, double clipRange) {

        for (int i = 0; i < input.length; i++) {
            input[i] = Math.min(clipRange, Math.max(-clipRange, input[i]));
        }

    }

    public static long getNanoSeconds(){return System.nanoTime();}

    public static double getDeltaTime(){
        currTime = getNanoSeconds();
        deltaTime = (currTime - prevTime) / NANOSECS_PER_SEC;
        prevTime = getNanoSeconds();
        return deltaTime;
    }

    public static double getDeltaTime(TimeUnits units){
        currTime = getNanoSeconds();
        deltaTime = (currTime - prevTime) / units.value;
        prevTime = getNanoSeconds();
        return deltaTime;
    }

    public static double clipValueToRange (double input, double lowerRange, double upperRange) {

        double clippedValue = Math.min(upperRange, Math.max(lowerRange, input));

        return clippedValue;
    }

    public static double clipValueToRange(double input, double upperRange){
        double clippedValue = Math.min(input, upperRange);
        return clippedValue;
    }

    public static void  clipValueToRange (double[] input, double lowerRange, double upperRange) {

        for (int i = 0; i < input.length; i++) {
            input[i] = Math.min(upperRange, Math.max(lowerRange, input[i]));
        }
    }

    public static void normalizeValues (double[] values) {

        double maxValue = 0;

        for (int i = 0; i < values.length; i++) {
            maxValue = Math.max(maxValue, Math.abs(values[i]));
        }

        for (int i = 0; i < values.length; i++) {
            values[i] /= maxValue;
        }
    }

    public static void normalizeSpeedsToMax (double[] speeds, double maxSpeed) {
        double maxValue = 0;

        for (int i = 0; i < speeds.length; i++) {
            maxValue = Math.max(maxValue, Math.abs(speeds[i]));
        }

        if (maxValue > maxSpeed) {
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] /= maxValue;
                speeds[i] *= maxSpeed;
            }
        }
    }

    public static void setLinearOpMode(MaelstromLinearOp pLinearOpMode) {
        linearOpMode = pLinearOpMode;
    }

    public static boolean opModeActive(){
        return linearOpMode.opModeIsActive();
    }

    public class KP {
        public static final double DT = +0.001;
        public static final double DISTANCE = +0.01;
        public static final double TURN = +0.02;
        public static final double RANGE = +0.02;
        public static final double SIDE = +0.02;
        public static final double POS = +0.02;
    }
    public class KI {
        public static final double DT = +0.003;
        public static final double DISTANCE = +0.0;
        public static final double TURN = +0.0;
        public static final double RANGE = +0.2;
        public static final double SIDE = +0.0;
        public static final double POS = +0.0;
    }
    public class KD {
        public static final double DT = +0.0;
        public static final double DISTANCE = +0.0;
        public static final double TURN = +0.0;
        public static final double RANGE = +0.0;
        public static final double SIDE = +0.0;
        public static final double POS = +0.0;
    }

    public enum AutoColor {
        RED, BLUE
    }

    public interface AutonomousOpMode {
        boolean getOpModeIsActive();

        Telemetry getTelemetry();
    }
}
