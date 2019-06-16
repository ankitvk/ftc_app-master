package ftc.library.MaelSubsystems.MaelstromDrivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelControl.PID.PIDController;
import ftc.library.MaelMotions.MaelMotors.MaelMotor;
import ftc.library.MaelMotions.MaelMotors.MaelMotorSystem;
import ftc.library.MaelMotions.MaelMotors.Motor;
import ftc.library.MaelRobot;
import ftc.library.MaelSensors.MaelstromEncoder;
import ftc.library.MaelUtils.MaelUtils;

public class MaelDrivetrain {

    public MaelMotorSystem leftDrive, rightDrive;
    public double driveGearReduction;
    public double drivenGearReduction;
    public DrivetrainModels model;
    public MaelstromEncoder encoderSensor;
    public MaelMotor fl,bl,fr,br;
    public double distanceKp, distanceKi, distanceKd, distanceMaxI;
    public double turnKp, turnKi, turnKd, turnMaxI;
    public double rangeKp, rangeKi, rangeKd, rangeMaxI;
    public double sideKp, sideKi, sideKd, sideMaxI;

    // public MaelUtils.AutonomousOpMode auto;
    //public PIDController distanceDrive = new PIDController(pidPackage().getDistanceKp(),pidPackage().getDistanceKi(),pidPackage().getDistanceKd(),1);
    public PIDController distancePid = new PIDController(distanceKp,distanceKi,distanceKd,distanceMaxI);
    public PIDController turnPid = new PIDController(turnKp,turnKi,turnKd,turnMaxI);
    public PIDController rangePid = new PIDController(rangeKp,rangeKi,rangeKd,rangeMaxI);
    public PIDController sidePid = new PIDController(sideKp,sideKi,sideKd,sideMaxI);


    public MaelDrivetrain(String name1, String name2, String name3, String name4, HardwareMap hwMap, Motor encoder) {
        leftDrive = new MaelMotorSystem(name1, name2, DcMotorSimple.Direction.REVERSE, hwMap, encoder);
        rightDrive = new MaelMotorSystem(name3, name4, DcMotorSimple.Direction.FORWARD, hwMap, encoder);
    }

    public MaelDrivetrain(DrivetrainModels model, double gearRatio, double Kp, double Ki, double Kd, HardwareMap hwMap, Motor type) {
        //leftDrive = new MotorSystem("leftFront", "leftBack", "Left Drive",DcMotor.Direction.REVERSE, "LEFTDRIVE", hwMap, type);
        leftDrive = new MaelMotorSystem(MaelUtils.LEFT_FRONT_KEY, MaelUtils.LEFT_BACK_KEY, Kp, Ki, Kd, DcMotor.Direction.REVERSE, hwMap, type);
        rightDrive = new MaelMotorSystem(MaelUtils.RIGHT_FRONT_KEY, MaelUtils.RIGHT_BACK_KEY, Kp, Ki, Kd, DcMotor.Direction.FORWARD, hwMap, type);
        leftDrive.setGearRatio(gearRatio);
        rightDrive.setGearRatio(gearRatio);
        driveGearReduction = (1 / gearRatio);
        drivenGearReduction = gearRatio;
        fl = leftDrive.motor1;
        bl = leftDrive.motor2;
        fr = rightDrive.motor1;
        br = rightDrive.motor2;
        this.model = model;
    }

    public MaelDrivetrain(DrivetrainModels model, Motor type, HardwareMap hwMap){
        leftDrive = new MaelMotorSystem("leftFront", "leftBack", DcMotor.Direction.REVERSE, hwMap, type);
        rightDrive = new MaelMotorSystem("rightFront", "rightBack", DcMotor.Direction.FORWARD, hwMap, type);
        this.model = model;
    }

    public MaelDrivetrain(DrivetrainModels model, double gearRatio, double Kp, double Ki, double Kd, HardwareMap hwMap, Motor type, MaelRobot robot) {
        //leftDrive = new MotorSystem("leftFront", "leftBack", "Left Drive",DcMotor.Direction.REVERSE, "LEFTDRIVE", hwMap, type);
        leftDrive = new MaelMotorSystem("frontLeft", "backLeft", Kp, Ki, Kd, DcMotor.Direction.REVERSE, hwMap, type);
        rightDrive = new MaelMotorSystem("frontRight", "backRight", Kp, Ki, Kd, DcMotor.Direction.FORWARD, hwMap, type);
        leftDrive.setGearRatio(gearRatio);
        rightDrive.setGearRatio(gearRatio);
        driveGearReduction = (1 / gearRatio);
        drivenGearReduction = gearRatio;
        this.model = model;
    }

    public void eReset() {
        leftDrive.stopAndReset();
        leftDrive.runWithoutEncoders();
        rightDrive.stopAndReset();
        rightDrive.runWithoutEncoders();
    }

    public void setPower(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    public void setPower(double left, double right) {
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    public void setVelocity(double velocity) {
        leftDrive.setVelocity(velocity);
        rightDrive.setVelocity(velocity);
    }

    public void setVelocity(double left, double right) {
        leftDrive.setVelocity(left);
        rightDrive.setVelocity(right);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        leftDrive.setZeroPowerBehavior(behavior);
        rightDrive.setZeroPowerBehavior(behavior);
    }

    public void setBreakeMode(){
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setFloatMode(){
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public double getPower() {
        return (leftDrive.getPower() + rightDrive.getPower()) / 2;
    }

    public double getCounts() {
        return leftDrive.getCounts();
    }

    public double getDriveGearReduction() {
        return driveGearReduction;
    }

    public double getDrivenGearReduction() {
        return drivenGearReduction;
    }

    public double getRawInches() {
        return leftDrive.getInches();
    }

    public double getTotalInches() {
        return (leftDrive.getInches() + rightDrive.getInches()) / 2;
    }

    public double getInches() {
        return (leftDrive.getInches() + rightDrive.getInches()) / 2;
    }

    public double getWheelDiameter() {
        return encoderSensor.getWheelDiameter();
    }

    public double setWheelDiameter() {
        return encoderSensor.getWheelDiameter();
    }

    public DrivetrainModels getModel() {
        return model;
    }

    public void setPID(double kp, double ki, double kd) {
        leftDrive.setPID(kp, ki, kd);
        rightDrive.setPID(kp, ki, kd);
    }

    public void setClosedLoop(boolean state){
        leftDrive.setClosedLoop(state);
        rightDrive.setClosedLoop(state);
    }
}
