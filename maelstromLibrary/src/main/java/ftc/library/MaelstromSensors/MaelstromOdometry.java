package ftc.library.MaelstromSensors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotor;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;
import ftc.library.MaelstromRobot;
import ftc.library.MaelstromUtils.MaelstromUtils;
import ftc.library.MaelstromUtils.TimeConstants;

/*Class for odometry wheel tracking*/
public class MaelstromOdometry implements TimeConstants {
    MaelstromMotor motor;
    MaelstromRobot robot;
    MaelstromUtils.AutonomousOpMode auto;
    public MaelstromIMU imu;
    public double distanceFromCenter = 0;
    public double previousYaw;

    private int previousPos = 0;
    private long previousTime = 0;
    private double rpm = 0;
    private double power;
    private double target;

    public MaelstromOdometry(String name, MotorModel model, MaelstromIMU imu, HardwareMap hwMap){
        this.motor = new MaelstromMotor(name,model,hwMap);
        this.imu = imu;
    }

    public MaelstromOdometry(MaelstromMotor motor, MaelstromIMU imu) {
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
