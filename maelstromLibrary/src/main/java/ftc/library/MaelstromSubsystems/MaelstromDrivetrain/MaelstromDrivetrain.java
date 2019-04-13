package ftc.library.MaelstromSubsystems.MaelstromDrivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromControl.MaelstromPID.PIDController;
import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotorSystem;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;
import ftc.library.MaelstromRobot;
import ftc.library.MaelstromSensors.MaelstromEncoder;
import ftc.library.MaelstromUtils.MaelstromUtils;

public class MaelstromDrivetrain {

    public MaelstromMotorSystem leftDrive, rightDrive;
    public double driveGearReduction;
    public double drivenGearReduction;
    public DrivetrainModels model;
    public MaelstromEncoder encoderSensor;
    public double distanceKp, distanceKi, distanceKd, distanceMaxI;
    public double turnKp, turnKi, turnKd, turnMaxI;
    public double rangeKp, rangeKi, rangeKd, rangeMaxI;
    public double sideKp, sideKi, sideKd, sideMaxI;

    public MaelstromUtils.AutonomousOpMode auto;
    //public PIDController distanceDrive = new PIDController(pidPackage().getDistanceKp(),pidPackage().getDistanceKi(),pidPackage().getDistanceKd(),1);
    public PIDController distancePid = new PIDController(distanceKp,distanceKi,distanceKd,distanceMaxI);
    public PIDController turnPid = new PIDController(turnKp,turnKi,turnKd,turnMaxI);
    public PIDController rangePid = new PIDController(rangeKp,rangeKi,rangeKd,rangeMaxI);
    public PIDController sidePid = new PIDController(sideKp,sideKi,sideKd,sideMaxI);


    public MaelstromDrivetrain(String name1, String name2, String name3, String name4, HardwareMap hwMap, MotorModel encoder) {
        leftDrive = new MaelstromMotorSystem(name1, name2, DcMotorSimple.Direction.REVERSE, hwMap, encoder);
        rightDrive = new MaelstromMotorSystem(name3, name4, DcMotorSimple.Direction.FORWARD, hwMap, encoder);
    }

    public MaelstromDrivetrain(DrivetrainModels model, double gearRatio, double Kp, double Ki, double Kd, HardwareMap hwMap, MotorModel type) {
        //leftDrive = new MotorSystem("leftFront", "leftBack", "Left Drive",DcMotor.Direction.REVERSE, "LEFTDRIVE", hwMap, type);
        leftDrive = new MaelstromMotorSystem(MaelstromUtils.LEFT_FRONT_KEY, MaelstromUtils.LEFT_BACK_KEY, Kp, Ki, Kd, DcMotor.Direction.REVERSE, hwMap, type);
        rightDrive = new MaelstromMotorSystem(MaelstromUtils.RIGHT_FRONT_KEY, MaelstromUtils.RIGHT_BACK_KEY, Kp, Ki, Kd, DcMotor.Direction.FORWARD, hwMap, type);
        leftDrive.setGearRatio(gearRatio);
        rightDrive.setGearRatio(gearRatio);
        driveGearReduction = (1 / gearRatio);
        drivenGearReduction = gearRatio;
        this.model = model;
    }

    public MaelstromDrivetrain(DrivetrainModels model, MotorModel type, HardwareMap hwMap){
        leftDrive = new MaelstromMotorSystem("leftFront", "leftBack", DcMotor.Direction.REVERSE, hwMap, type);
        rightDrive = new MaelstromMotorSystem("rightFront", "rightBack", DcMotor.Direction.FORWARD, hwMap, type);
        this.model = model;
    }

    public MaelstromDrivetrain(DrivetrainModels model, double gearRatio, double Kp, double Ki, double Kd, HardwareMap hwMap, MotorModel type, MaelstromRobot robot) {
        //leftDrive = new MotorSystem("leftFront", "leftBack", "Left Drive",DcMotor.Direction.REVERSE, "LEFTDRIVE", hwMap, type);
        leftDrive = new MaelstromMotorSystem("frontLeft", "backLeft", Kp, Ki, Kd, DcMotor.Direction.REVERSE, hwMap, type);
        rightDrive = new MaelstromMotorSystem("frontRight", "backRight", Kp, Ki, Kd, DcMotor.Direction.FORWARD, hwMap, type);
        leftDrive.setGearRatio(gearRatio);
        rightDrive.setGearRatio(gearRatio);
        driveGearReduction = (1 / gearRatio);
        drivenGearReduction = gearRatio;
        this.model = model;
        this.auto  = robot.auto;
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

    public MaelstromUtils.AutonomousOpMode opModeActive(){
        return auto;
    }

    public void setClosedLoop(boolean state){
        leftDrive.setClosedLoop(state);
        rightDrive.setClosedLoop(state);
    }
}
