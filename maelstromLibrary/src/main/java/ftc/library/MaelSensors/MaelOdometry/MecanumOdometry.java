package ftc.library.MaelSensors.MaelOdometry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelMotions.MaelMotors.MaelMotor;
import ftc.library.MaelMotions.MaelMotors.Motor;
import ftc.library.MaelRobot;
import ftc.library.MaelSensors.MaelIMU;
import ftc.library.MaelSubsystems.MaelstromDrivetrain.MaelDrivetrain;
import ftc.library.MaelUtils.MaelUtils;
import ftc.library.MaelUtils.LibConstants;

/*Class for odometry wheel tracking*/
public class MecanumOdometry implements LibConstants {
    MaelMotor motor;
    MaelRobot robot;
    MaelUtils.AutonomousOpMode auto;
    public MaelIMU imu;
    public double distanceFromCenter = 0;
    public double previousYaw;

    private int previousPos = 0;
    private long previousTime = 0;
    private double rpm = 0;
    private double power;
    private double target;


    public MecanumOdometry(String name, Motor model, MaelIMU imu, HardwareMap hwMap){
        this.motor = new MaelMotor(name,model,hwMap);
        this.imu = imu;
    }

    public MecanumOdometry(MaelMotor motor, MaelIMU imu) {
        this.motor = motor;
        this.imu = imu;
        this.reset();
    }

    public double getPosition(){
        return motor.getCounts();
    }

    public double getTargetCounts(double pos){
        pos = ((pos/(Math.PI*2))*motor.getCPR());
        setTargetCounts(pos);
        target = pos;
        return pos;
    }

    public double trackPosition(){
        double pos = ((getPosition()*motor.getCPR())*(2*Math.PI));
        return pos;
    }

    public void reset(){
        motor.stopAndReset();
        motor.runWithoutEncoders();
    }

    public double getAcceleration(){return motor.getAcceleration();}

    public double getVelocity(){
        return motor.getVelocity();
    }

    public void setTargetCounts(double target){
        this.target = target;
    }

    public double getAngle(){
        double countsRemaining = target - getPosition();
        return countsRemaining;
    }
}
