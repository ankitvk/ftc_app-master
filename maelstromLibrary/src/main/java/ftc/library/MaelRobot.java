package ftc.library;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import java.util.ArrayList;
import java.util.List;

import ftc.library.MaelControl.PID.PIDController;

import ftc.library.MaelControl.PurePursuit.MaelPose;
import ftc.library.MaelMotions.MaelMotors.Direction;
import ftc.library.MaelSensors.MaelIMU;
import ftc.library.MaelSensors.MaelOdometry.MecanumOdometry;
import ftc.library.MaelSensors.MaelOdometry.TankOdometry;
import ftc.library.MaelSensors.MaelTimer;
import ftc.library.MaelSubsystems.MaelstromDrivetrain.DrivetrainModels;
import ftc.library.MaelSubsystems.MaelstromDrivetrain.MaelDrivetrain;
import ftc.library.MaelSubsystems.Subsystem;
import ftc.library.MaelUtils.AxesSigns;
import ftc.library.MaelUtils.MaelUtils;
import ftc.library.MaelUtils.LibConstants;
import ftc.library.MaelUtils.TimeUnits;
import ftc.library.MaelWrappers.MaelController;
import ftc.library.MaelWrappers.MaelTellemetry;

public abstract class MaelRobot implements LibConstants {
    public abstract void initHardware(HardwareMap hwMap);
    public MaelTellemetry feed;
    public MaelDrivetrain dt;
    public MecanumOdometry xPos = null;
    public MecanumOdometry yPos = null;
    public TankOdometry tankTracker;
    public MaelIMU imu;
    public double gearRatio = 0;
    public MaelController controller;
    public boolean stabilization = false;
    private double yDirection = 0;
    private double xDirection = 0;
    private double speedMultiplier = 1;
    private double angle = 0;
    private double pitchTarget = 0;
    private double fieldCentric = 0;
    private double prevX = 0 , prevY = 0;
    private double desiredAngle = 0;
    private double pitchCorrection = 0;
    private double speeds[];
    private double mechAngle = 0;
    private double chosenMultiplier = 0;

    //public PIDController distanceDrive = new PIDController(pidPackage().getDistanceKp(),pidPackage().getDistanceKi(),pidPackage().getDistanceKd(),1);
    public PIDController distancePid = new PIDController(0.01,0,0,1);
    public PIDController pitchPid = new PIDController(0.01,0,0,1);
    public PIDController turnPid = new PIDController(0.01,0,0,1);
    public PIDController sideTurnPid = new PIDController(0.01,0,0,1);


    public static void reMapAxis(MaelIMU imu, AxesOrder order, AxesSigns signs){
        imu.remapAxes(imu,order,signs);
    }

    public void add(Subsystem... elements){
        for(Subsystem s : elements) MaelUtils.subsystems.add(s);
    }

/*    public List<Subsystem> getSubsystemList(){
        return subsystems;
    }*/

    public void driveDistance(double distance, double speed, Direction direction, double stopTime, double sleep, double timeout){
        dt.eReset();
        distance *= direction.value;
        double counts = distanceToCounts(distance);
        MaelTimer timer = new MaelTimer();
        MaelTimer timeoutTimer = new MaelTimer();
        long startTime = /*System.nanoTime()*/ timer.startTime();
        long stopState = 0;
        //double initialHeading = imu.getRelativeYaw();

        while(isStopRequested() && stopState <= stopTime /*!timer.elapsedTime(timeout, MaelTimer.Time.SECS)*/){
            //double position = dt.getCounts();
            double position = dt.getInches();
            //double power = (distanceDrive.power(counts,position))*speed;
            double power = (distancePid.power(distance,position))*speed;
            timeoutTimer.reset();
            //double power = distanceDrive.power(distance,dt.getInches());

            drive(power);

            feed.add("Power:",power);
            feed.add("Kp*error:",distancePid.getP());
            feed.add("Ki*i:",distancePid.getI());
            feed.add("Distance:",/*countsToDistance(dt.getCounts())*/dt.getInches());
            feed.add("Stop state:",stopState);
            feed.update();

            if(distancePid.getError() <= 0.5){
                //stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
                stopState = timer.stopState();
            }
            else startTime = timer.startTime();

            if(/*startTime/NANOSECS_PER_MILISEC >= 5000*/timeoutTimer.elapsedTime(timeout, TimeUnits.SECS)) break;
        }
        stop();
        MaelUtils.sleep(sleep);
    }
    public void driveDistance(double distance, double speed, Direction direction, double stopTime, double sleep){
        driveDistance(distance,speed,direction,stopTime,sleep, MaelUtils.DEFAULT_TIMEOUT);
    }
    public void driveDistance(double distance, double speed, Direction direction, double stopTime){
        driveDistance(distance,speed,direction,stopTime, MaelUtils.DEFAULT_SLEEP_TIME);
    }
    public void driveDistance(double distance, double speed, Direction direction){
        driveDistance(distance,speed,direction, MaelUtils.DEFAULT_STOPSTATE);
    }
    public void driveDistance(double distance, Direction direction){
        driveDistance(distance, MaelUtils.DEFAULT_SPEED,direction);
    }
    public void driveDistance(double distance){
        driveDistance(distance, Direction.FORWARD);
    }

    public void driveAngle(double distance, double maxSpeed, double angle, Direction direction, long stopTime){
        if(dt.getModel() == DrivetrainModels.MECH_FIELD || dt.getModel() == DrivetrainModels.MECH_ROBOT){
            double frontLeftPower;
            double backLeftPower;
            double frontRightPower;
            double backRightPower;

            dt.eReset();

            long startTime = System.nanoTime();
            long stopState = 0;
            //double initialHeading = imu.getRelativeYaw();
            angle = Math.toRadians(angle);
            double adjustedAngle = angle + Math.PI/4;
            distance *= direction.value;
            double counts = distanceToCounts(distance);

            frontLeftPower = Math.sin(adjustedAngle);
            backLeftPower = Math.cos(adjustedAngle);
            frontRightPower = Math.cos(adjustedAngle);
            backRightPower = Math.sin(adjustedAngle);

            while (isStopRequested() && (stopState <= stopTime)) {
                double position = dt.getCounts();
                double power = (distancePid.power(counts,position));

                speeds[0] = frontLeftPower * power;
                speeds[1] = backLeftPower * power;
                speeds[2] = frontRightPower * power;
                speeds[3] = backRightPower * power;

                MaelUtils.normalizeSpeedsToMax(speeds, maxSpeed);

                dt.leftDrive.motor1.setVelocity(speeds[0]);
                dt.leftDrive.motor2.setVelocity(speeds[1]);
                dt.rightDrive.motor1.setVelocity(speeds[2]);
                dt.rightDrive.motor2.setVelocity(speeds[3]);

                feed.add("frontLeft:",speeds[0]);
                feed.add("backLeft:",speeds[1]);
                feed.add("frontRight:",speeds[2]);
                feed.add("backRight:",speeds[3]);
                feed.add("IMU:",imu.getRelativeYaw());
                feed.add("Stop State:",stopState);

                if(distancePid.getError() <= 0.5){
                    stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
                }
                else startTime = System.nanoTime();

                if(startTime/NANOSECS_PER_MILISEC >= 5000) break;
            }
            stop();
        }
        else{
            feed.add("CANNOT RUN: INCORRECT DRIVETRAIN MODEL");
            feed.update();
        }

    }
    public void driveAngle(double distance, double maxSpeed, double angle, Direction direction){
        driveAngle(distance, maxSpeed, angle, direction, 0);
    }
    public void driveAngle(double distance, double angle, Direction direction){
        driveAngle(distance, 1, angle, direction);
    }

    public void rotate(double speed){
        dt.leftDrive.setVelocity(-speed);
        dt.rightDrive.setVelocity(speed);
    }

    public void turnAbsolute(double degrees, double speed, Direction direction, double stopTime){
        long startTime = System.nanoTime();
        long stopState = 0;
        degrees *= direction.value;
        //double target = imu.getRelativeYaw();

        while (isStopRequested() && (stopState <= stopTime)){
            double position = imu.getRelativeYaw();
            double power = (turnPid.power(degrees,position))*speed;

            rotate(power);

            feed.add("Power:",power);
            feed.add("Kp*error:",turnPid.getP());
            feed.add("Ki*i:",turnPid.getI());
            feed.add("Angle:",imu.getRelativeYaw());
            feed.add("Stop state:",stopState);
            feed.update();

            if(turnPid.getError() <= 0.5){
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            }
            else startTime = System.nanoTime();

            if(startTime/NANOSECS_PER_MILISEC >= 5000) break;
        }
        stop();
    }
    public void turnAbsolute(double degrees, double speed, Direction direction){
        turnAbsolute(degrees, speed,direction, MaelUtils.DEFAULT_STOPSTATE);
    }
    public void turnAbsolute(double degrees, Direction dIrection){
        turnAbsolute(degrees, MaelUtils.DEFAULT_SPEED, dIrection);
    }
    public void turn(double degrees, double speed, Direction direction, double stopTime){
        turnAbsolute(imu.getRelativeYaw() + degrees,speed,direction,stopTime);
    }
    public void turn(double degrees, double speed, Direction direction){
        turn(degrees,speed,direction, MaelUtils.DEFAULT_STOPSTATE);
    }
    public void turn(double degrees, Direction direction){
        turn(degrees, MaelUtils.DEFAULT_SPEED,direction);
    }
    public void sideTurn(double degrees, double ratio, String side, Direction direction){
        long startTime = System.nanoTime();
        long stopState = 0;

        while(isStopRequested() && (stopState <= 1000)){

            double position = imu.getRelativeYaw();
            degrees *= direction.value;
            double power = sideTurnPid.power(degrees,position);

            dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(side == "left") drive(power,0);
            else drive(0,power);

            feed.add("Angle:",imu.getRelativeYaw());
            feed.add("");
            feed.add("KP*error: ", sideTurnPid.returnVal()[0]);
            feed.add("KI*i: ", sideTurnPid.returnVal()[1]);
            feed.add("KD*d: ", sideTurnPid.returnVal()[2]);
            feed.add("Error: ", sideTurnPid.getError());
            feed.add("Power: ", power);
            feed.update();

            if (/*Math.abs(distance - currDistance) */ sideTurnPid.getError() <= 0.5) {
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            } else startTime = System.nanoTime();

            if(startTime/NANOSECS_PER_MILISEC >= 5000) break;
        }
        stop();
    }

/*    public void driveToPoint(MaelPose point, double maxSpeed, long stopTime){

        double xTarget = point.x;
        double yTarget = point.y;

        xTarget = xPos.getTargetCounts(xTarget);
        yTarget = yPos.getTargetCounts(yTarget);

        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        dt.eReset();
        xPos.reset();
        yPos.reset();

        long startTime = System.nanoTime();
        long stopState = 0;
        //double initialHeading = imu.getRelativeYaw();
        double distance = Math.hypot(xTarget, yTarget);

    if(xPos != null && yPos != null) {
        while (isStopRequested() && (stopState <= stopTime)) {
            angle = Math.atan2(xPos.getAngle(), yPos.getAngle());
            double adjustedAngle = angle + Math.PI / 4;

            frontLeftPower = Math.sin(adjustedAngle);
            backLeftPower = Math.cos(adjustedAngle);
            frontRightPower = Math.cos(adjustedAngle);
            backRightPower = Math.sin(adjustedAngle);

            double currDistance = Math.hypot(xPos.getPosition(), yPos.getPosition());
            double pidPower = distancePid.power(distance, currDistance);

            speeds[0] = frontLeftPower * pidPower;
            speeds[1] = backLeftPower * pidPower;
            speeds[2] = frontRightPower * pidPower;
            speeds[3] = backRightPower * pidPower;

            MaelUtils.normalizeSpeedsToMax(speeds, maxSpeed);

            dt.leftDrive.motor1.setVelocity(speeds[0]);
            dt.leftDrive.motor2.setVelocity(speeds[1]);
            dt.rightDrive.motor1.setVelocity(speeds[2]);
            dt.rightDrive.motor2.setVelocity(speeds[3]);

            feed.add("X Position:", xPos.trackPosition());
            feed.add("Y Position:", yPos.trackPosition());
            feed.add("Distance:", distance);
            feed.add("Stop state:", stopState);
            feed.add("Kp*error:", distancePid.getP());
            feed.add("Ki*i:", distancePid.getI());
            feed.add("Kd*d:", distancePid.getD());
            feed.update();

            if (*//*Math.abs(distance - currDistance) *//* distancePid.getError() <= 0.5) {
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
                if (stopState == stopTime - 10) {
                    prevX = xPos.trackPosition();
                    prevY = yPos.trackPosition();
                }
            } else startTime = System.nanoTime();
        }
    }
        stop();
    }
    public void driveToPoint(MaelPose point, double maxSpeed){
        driveToPoint(point, maxSpeed,0);
    }
    public void driveToPoint(MaelPose point){
        driveToPoint(point,1);
    }*/

    public void strafe(double speed, double angle, Direction strafe, long stopTime){
        DrivetrainModels model = dt.getModel();
        if(model == DrivetrainModels.MECH_FIELD || model == DrivetrainModels.MECH_ROBOT){
            double frontLeft;
            double backLeft;
            double frontRight;
            double backRight;

            dt.eReset();
            long startTime = System.nanoTime();
            long stopState = 0;

            angle = Math.toRadians(angle*-(strafe.value));
            double adjustedAngle = angle + Math.PI/4;

            frontLeft = speed * (Math.sin(adjustedAngle));
            backLeft = speed * (Math.cos(adjustedAngle));
            frontRight = speed * (Math.cos(adjustedAngle));
            backRight = speed * (Math.sin(adjustedAngle));

            while(isStopRequested() && stopState <= stopTime){
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
                dt.leftDrive.motor1.setVelocity(frontLeft);
                dt.leftDrive.motor2.setVelocity(backLeft);
                dt.rightDrive.motor1.setVelocity(frontRight);
                dt.rightDrive.motor2.setVelocity(backRight);

                feed.add("frontLeft:",frontLeft);
                feed.add("backLeft:",backLeft);
                feed.add("frontRight:",frontRight);
                feed.add("backRights:",backRight);
                feed.add("IMU:",imu.getRelativeYaw());
                feed.add("Stop State:",stopState);
            }
            stop();
        }
        else {
            feed.add("CANNOT RUN: INCORRECT DRIVETRAIN MODEL");
            feed.update();
        }
    }
    public void strafe(double speed, double angle, Direction strafe){
        strafe(speed, angle,strafe, 0);
    }

    public void stop(){
        dt.setVelocity(0);
    }

    public void drive(double power){
        dt.setVelocity(power);
    }
    public void drive(double left, double right){
        dt.setVelocity(left,right);
    }

    public void driveTeleop(MaelController controller){
        DrivetrainModels model = dt.getModel();
        this.controller = controller;
        if(model == DrivetrainModels.ARCADE){
            yDirection = speedMultiplier * controller.leftStickY();
            xDirection = speedMultiplier * controller.rightStickX();

            double left = yDirection - xDirection;
            double right = yDirection + xDirection;
            left += pitchCorrection;
            right += pitchCorrection;

            drive(left,right);
        }
        else if(model == DrivetrainModels.TANK){
            double left = controller.leftStickY();
            double right = controller.rightStickY();
            left += pitchCorrection;
            right += pitchCorrection;

            drive(left,right);
        }
        else if(model == DrivetrainModels.MECH_FIELD){

            double leftY = controller.leftStickY();
            double leftX = controller.leftStickX();
            double rightX = controller.rightStickX();

            double x = -leftY;
            double y = leftX;

            double angle = Math.atan2(y,x)/*controller.getTan(x,y)*/;
            double fieldCentric = angle - Math.toRadians(imu.getYaw());
            double adjustedAngle = fieldCentric + Math.PI / 4;

            this.angle = angle;
            this.fieldCentric = fieldCentric;

            double speedMagnitude = Math.hypot(x,y);

            if(Math.abs(rightX) > 0.00001) desiredAngle = imu.getRelativeYaw();

            //double speeds[] = {Math.sin(adjustedAngle), Math.cos(adjustedAngle), Math.cos(adjustedAngle), Math.sin(adjustedAngle)};
            double speeds[] = getSpeeds(adjustedAngle,speedMagnitude,rightX);
/*
            MaelUtils.normalizeValues(speeds);

            speeds[0] = (speeds[0] * speedMagnitude) - rightX;
            speeds[1] = (speeds[1] * speedMagnitude) - rightX;
            speeds[2] = (speeds[2] * speedMagnitude) + rightX;
            speeds[3] = (speeds[3] * speedMagnitude) + rightX;
            this.speeds = speeds;*/

            dt.fl.setVelocity(speeds[0] + pitchCorrection);
            dt.bl.setVelocity(speeds[1] + pitchCorrection);
            dt.fr.setVelocity(speeds[2] + pitchCorrection);
            dt.br.setVelocity(speeds[3] + pitchCorrection);
        }
        else if(model == DrivetrainModels.MECH_ROBOT){

            double leftY = -controller.leftStickY();
            double leftX = controller.leftStickX();
            double rightX = controller.rightStickX();

            double x = -leftY;
            double y = -leftX;

            double angle = Math.atan2(y,x)/*controller.getTan(x,y)*/;
            double adjustedAngle = angle + Math.PI / 4;
            this.angle = angle;

            double speedMagnitude = Math.hypot(x,y);

            double speeds[] =  {Math.sin(adjustedAngle), Math.cos(adjustedAngle), Math.cos(adjustedAngle), Math.sin(adjustedAngle)};
            //double speeds[] = getSpeeds(adjustedAngle,speedMagnitude,rightX);


            MaelUtils.normalizeValues(speeds);

            speeds[0] = (speeds[0] * speedMagnitude) - rightX;
            speeds[1] = (speeds[1] * speedMagnitude) - rightX;
            speeds[2] = (speeds[2] * speedMagnitude) + rightX;
            speeds[3] = (speeds[3] * speedMagnitude) + rightX;
            this.speeds = speeds;

/*            dt.leftDrive.motor1.setVelocity(speeds[0] + pitchCorrection);
            dt.leftDrive.motor2.setVelocity(speeds[1] + pitchCorrection);
            dt.rightDrive.motor1.setVelocity(speeds[2] + pitchCorrection);
            dt.rightDrive.motor2.setVelocity(speeds[3] + pitchCorrection);*/

            dt.fl.setVelocity(speeds[0] + pitchCorrection);
            dt.bl.setVelocity(speeds[1] + pitchCorrection);
            dt.fr.setVelocity(speeds[2] + pitchCorrection);
            dt.br.setVelocity(speeds[3] + pitchCorrection);

        }
/*        if(stabilization){
            pitchCorrection = pitchPid.power(pitchTarget,imu.getPitch());
        }
        else pitchCorrection = 0;*/
        if(controller.leftJoystickButtonToggle() && controller.rightJoystickButtonToggle()) setSpeedMultiplier(.5);
        else  setSpeedMultiplier(chosenMultiplier);
    }

    public double[] getSpeeds(double adjustedAngle, double speedMagnitude, double rightX){

        double speeds[] = {Math.sin(adjustedAngle), Math.cos(adjustedAngle), Math.cos(adjustedAngle), Math.sin(adjustedAngle)};

        MaelUtils.normalizeValues(speeds);

        speeds[0] = (speeds[0] * speedMagnitude) - rightX;
        speeds[1] = (speeds[1] * speedMagnitude) - rightX;
        speeds[2] = (speeds[2] * speedMagnitude) + rightX;
        speeds[3] = (speeds[3] * speedMagnitude) + rightX;
        this.speeds = speeds;

        return speeds;
    }

    public void setStabilization(boolean bool){this.stabilization = bool;}

    public void setPitchPid(double kp, double ki, double kd){
        pitchPid.setPID(kp,ki,kd);
    }

    public void setSpeedMultiplier(double speedMultiplier){
        this.speedMultiplier = speedMultiplier;
        chosenMultiplier = speedMultiplier;
    }

    public double distanceToCounts(double distance){
        return (distance/(dt.leftDrive.motor1.getEncoder().getWheelCircumference())*dt.getDriveGearReduction()*dt.leftDrive.motor1.getEncoder().getCPR());
        //return (distance/(dt.leftDrive.getEncoder())*dt.getDriveGearReduction()*dt.leftDrive.motor1.getEncoder().getCPR());
    }

    public double countsToDistance(double counts){
        return (counts*dt.leftDrive.getWheelCircumference()*dt.getDrivenGearReduction())/dt.leftDrive.motor1.getEncoder().getCPR();
    }

    public boolean isStopRequested(){return !MaelUtils.linearOpMode.isStopRequested();}

}

