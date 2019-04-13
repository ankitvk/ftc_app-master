package ftc.library;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import ftc.library.MaelstromControl.MaelstromPID.PIDController;
import ftc.library.MaelstromControl.MaelstromPID.PIDPackage;

import ftc.library.MaelstromControl.MaelstromPurePursuit.WayPoint;
import ftc.library.MaelstromMotions.MaelstromMotors.Direction;
import ftc.library.MaelstromSensors.MaelstromIMU;
import ftc.library.MaelstromSensors.MaelstromOdometry;
import ftc.library.MaelstromSensors.MaelstromTimer;
import ftc.library.MaelstromSubsystems.MaelstromDrivetrain.DrivetrainModels;
import ftc.library.MaelstromSubsystems.MaelstromDrivetrain.MaelstromDrivetrain;
import ftc.library.MaelstromUtils.AxesSigns;
import ftc.library.MaelstromUtils.MaelstromUtils;
import ftc.library.MaelstromUtils.TimeConstants;
import ftc.library.MaelstromWrappers.MaelstromController;
import ftc.library.MaelstromWrappers.MaelstromTelemetry;

public abstract class MaelstromRobot implements TimeConstants {
    public abstract void initHardware(HardwareMap hwMap);
    public abstract PIDPackage pidPackage();
    public MaelstromTelemetry feed;
    public MaelstromDrivetrain dt;
    public MaelstromOdometry xPos;
    public MaelstromOdometry yPos;
    public MaelstromIMU imu;
    public MaelstromUtils.AutonomousOpMode auto;
    public double gearRatio;
    public MaelstromController controller;
    public boolean stabilization = false;
    private double yDirection;
    private double xDirection;
    private double speedMultiplier;
    private double angle;
    private double pitchTarget = 0;
    private double fieldCentric = 0;
    private double prevX = 0 , prevY = 0;
    private double desiredAngle = 0;
    private double pitchCorrection = 0;
    private double speeds[];
    public PIDController distanceDrive = new PIDController(pidPackage().getDistanceKp(),pidPackage().getDistanceKi(),pidPackage().getDistanceKd(),1);
    public PIDController pitchPid = new PIDController(0,0,0,1);

    public static void reMapAxis(MaelstromIMU imu, AxesOrder order, AxesSigns signs){
        imu.remapAxes(imu,order,signs);
    }

    public void driveDistance(double distance, double speed, Direction direction, double stopTime, double sleep, double timeout){
        dt.eReset();
        distance *= direction.value;
        double counts = distanceToCounts(distance);
        MaelstromTimer timer = new MaelstromTimer();
        MaelstromTimer timeoutTimer = new MaelstromTimer();
        long startTime = /*System.nanoTime()*/ timer.startTime();
        long stopState = 0;
        //double initialHeading = imu.getRelativeYaw();

        while(opModeActive() && stopState <= stopTime /*!timer.elapsedTime(timeout, MaelstromTimer.Time.SECS)*/){
            //double position = dt.getCounts();
            double position = dt.getInches();
            //double power = (distanceDrive.power(counts,position))*speed;
            double power = (distanceDrive.power(distance,position))*speed;
            timeoutTimer.reset();
            //double power = distanceDrive.power(distance,dt.getInches());

            drive(power);

            feed.add("Power:",power);
            feed.add("Kp*error:",distanceDrive.getP());
            feed.add("Ki*i:",distanceDrive.getI());
            feed.add("Distance:",/*countsToDistance(dt.getCounts())*/dt.getInches());
            feed.add("Stop state:",stopState);
            feed.update();

            if(distanceDrive.getError() <= 0.5){
                //stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
                stopState = timer.stopState();
            }
            else startTime = timer.startTime();

            if(/*startTime/NANOSECS_PER_MILISEC >= 5000*/timeoutTimer.elapsedTime(timeout, MaelstromTimer.Time.SECS)) break;
        }
        stop();
        MaelstromUtils.sleep(sleep);
    }
    public void driveDistance(double distance, double speed, Direction direction, double stopTime, double sleep){
        driveDistance(distance,speed,direction,stopTime,sleep,MaelstromUtils.DEFAULT_TIMEOUT);
    }
    public void driveDistance(double distance, double speed, Direction direction, double stopTime){
        driveDistance(distance,speed,direction,stopTime,MaelstromUtils.DEFAULT_SLEEP_TIME);
    }
    public void driveDistance(double distance, double speed, Direction direction){
        driveDistance(distance,speed,direction,MaelstromUtils.DEFAULT_STOPSTATE);
    }
    public void driveDistance(double distance, Direction direction){
        driveDistance(distance,MaelstromUtils.DEFAULT_SPEED,direction);
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

            while (opModeActive() && (stopState <= stopTime)) {
                double position = dt.getCounts();
                double power = (distanceDrive.power(counts,position));

                speeds[0] = frontLeftPower * power;
                speeds[1] = backLeftPower * power;
                speeds[2] = frontRightPower * power;
                speeds[3] = backRightPower * power;

                MaelstromUtils.normalizeSpeedsToMax(speeds, maxSpeed);

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

                if(distanceDrive.getError() <= 0.5){
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
        PIDController turnAngle = new PIDController(pidPackage().getTurnKp(),pidPackage().getTurnKi(),pidPackage().getTurnKd(),1);
        long startTime = System.nanoTime();
        long stopState = 0;
        degrees *= direction.value;
        //double target = imu.getRelativeYaw();

        while (opModeActive() && (stopState <= stopTime)){
            double position = imu.getRelativeYaw();
            double power = (turnAngle.power(degrees,position))*speed;

            rotate(power);

            feed.add("Power:",power);
            feed.add("Kp*error:",turnAngle.getP());
            feed.add("Ki*i:",turnAngle.getI());
            feed.add("Angle:",imu.getRelativeYaw());
            feed.add("Stop state:",stopState);
            feed.update();

            if(turnAngle.getError() <= 0.5){
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            }
            else startTime = System.nanoTime();

            if(startTime/NANOSECS_PER_MILISEC >= 5000) break;
        }
        stop();
    }
    public void turnAbsolute(double degrees, double speed, Direction direction){
        turnAbsolute(degrees, speed,direction, MaelstromUtils.DEFAULT_STOPSTATE);
    }
    public void turnAbsolute(double degrees, Direction dIrection){
        turnAbsolute(degrees, MaelstromUtils.DEFAULT_SPEED, dIrection);
    }
    public void turn(double degrees, double speed, Direction direction, double stopTime){
        turnAbsolute(imu.getRelativeYaw() + degrees,speed,direction,stopTime);
    }
    public void turn(double degrees, double speed, Direction direction){
        turn(degrees,speed,direction,MaelstromUtils.DEFAULT_STOPSTATE);
    }
    public void turn(double degrees, Direction direction){
        turn(degrees,MaelstromUtils.DEFAULT_SPEED,direction);
    }
    public void sideTurn(double degrees, double ratio, String side, Direction direction){
        long startTime = System.nanoTime();
        long stopState = 0;
        PIDController turnSide = new PIDController(pidPackage().getSideKp(), pidPackage().getSideKi(), pidPackage().getSideKd(), 1);

        while(opModeActive() && (stopState <= 1000)){

            double position = imu.getRelativeYaw();
            degrees *= direction.value;
            double power = turnSide.power(degrees,position);

            dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(side == "left") drive(power,0);
            else drive(0,power);

            feed.add("Angle:",imu.getRelativeYaw());
            feed.add("");
            feed.add("KP*error: ", turnSide.returnVal()[0]);
            feed.add("KI*i: ", turnSide.returnVal()[1]);
            feed.add("KD*d: ", turnSide.returnVal()[2]);
            feed.add("Error: ", turnSide.getError());
            feed.add("Power: ", power);
            feed.update();

            if (/*Math.abs(distance - currDistance) */ turnSide.getError() <= 0.5) {
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            } else startTime = System.nanoTime();

            if(startTime/NANOSECS_PER_MILISEC >= 5000) break;
        }
        stop();
    }

    public void driveToPoint(WayPoint point, double maxSpeed, long stopTime){

        double xTarget = point.getX();
        double yTarget = point.getY();

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

        while(opModeActive() && (stopState <= stopTime)){
            angle = Math.atan2(xPos.getAngle(),yPos.getAngle());
            double adjustedAngle = angle + Math.PI/4;

            frontLeftPower = Math.sin(adjustedAngle);
            backLeftPower = Math.cos(adjustedAngle);
            frontRightPower = Math.cos(adjustedAngle);
            backRightPower = Math.sin(adjustedAngle);

            double currDistance = Math.hypot(xPos.getPosition(),yPos.getPosition());
            double pidPower = distanceDrive.power(distance,currDistance);

            speeds[0] = frontLeftPower*pidPower;
            speeds[1] = backLeftPower*pidPower;
            speeds[2] = frontRightPower*pidPower;
            speeds[3] = backRightPower*pidPower;

            MaelstromUtils.normalizeSpeedsToMax(speeds, maxSpeed);

            dt.leftDrive.motor1.setVelocity(speeds[0]);
            dt.leftDrive.motor2.setVelocity(speeds[1]);
            dt.rightDrive.motor1.setVelocity(speeds[2]);
            dt.rightDrive.motor2.setVelocity(speeds[3]);

            feed.add("X Position:",xPos.trackPosition());
            feed.add("Y Position:",yPos.trackPosition());
            feed.add("Distance:",distance);
            feed.add("Stop state:",stopState);
            feed.add("Kp*error:",distanceDrive.getP());
            feed.add("Ki*i:",distanceDrive.getI());
            feed.add("Kd*d:",distanceDrive.getD());
            feed.update();

            if (/*Math.abs(distance - currDistance) */ distanceDrive.getError() <= 0.5) {
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
                if(stopState == stopTime - 10) {
                    prevX = xPos.trackPosition();
                    prevY = yPos.trackPosition();
                }
            } else startTime = System.nanoTime();
        }
        stop();
    }
    public void driveToPoint(WayPoint point, double maxSpeed){
        driveToPoint(point, maxSpeed,0);
    }
    public void driveToPoint(WayPoint point){
        driveToPoint(point,1);
    }

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

            while(opModeActive() && stopState <= stopTime){
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

    public void driveTeleop(MaelstromController controller){
        DrivetrainModels model = dt.getModel();
        if(model == DrivetrainModels.ARCADE){
            this.controller = controller;
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

            double angle = /*Math.atan2(y,x)*/controller.getTan(x,y);
            double fieldCentric = angle - Math.toRadians(imu.getYaw());
            double adjustedAngle = fieldCentric + Math.PI / 4;

            this.angle = angle;
            this.fieldCentric = fieldCentric;

            double speedMagnitude = Math.hypot(x,y);

            if(Math.abs(rightX) > 0.00001) desiredAngle = imu.getRelativeYaw();

            double speeds[] = {Math.sin(adjustedAngle), Math.cos(adjustedAngle), Math.cos(adjustedAngle), Math.sin(adjustedAngle)};

            MaelstromUtils.normalizeValues(speeds);

            speeds[0] = (speeds[0] * speedMagnitude) + rightX;
            speeds[1] = (speeds[1] * speedMagnitude) + rightX;
            speeds[2] = (speeds[2] * speedMagnitude) + rightX;
            speeds[3] = (speeds[3] * speedMagnitude) + rightX;
            this.speeds = speeds;

            dt.leftDrive.motor1.setVelocity(speeds[0] + pitchCorrection);
            dt.leftDrive.motor2.setVelocity(speeds[1] + pitchCorrection);
            dt.rightDrive.motor1.setVelocity(speeds[2] + pitchCorrection);
            dt.rightDrive.motor2.setVelocity(speeds[3] + pitchCorrection);
        }
        else if(model == DrivetrainModels.MECH_ROBOT){

            double leftY = controller.leftStickY();
            double leftX = controller.leftStickX();
            double rightX = controller.rightStickX();

            double x = -leftY;
            double y = leftX;

            double angle = Math.atan2(y,x)/*controller.getTan(x,y)*/;
            double adjustedAngle = angle + Math.PI / 4;

            this.angle = angle;

            double speedMagnitude = Math.hypot(x,y);

            if(Math.abs(rightX) > 0.00001) desiredAngle = imu.getRelativeYaw();

            double speeds[] = {Math.sin(adjustedAngle), Math.cos(adjustedAngle), Math.cos(adjustedAngle), Math.sin(adjustedAngle)};

            MaelstromUtils.normalizeValues(speeds);

            speeds[0] = (speeds[0] * speedMagnitude) + rightX;
            speeds[1] = (speeds[1] * speedMagnitude) + rightX;
            speeds[2] = (speeds[2] * speedMagnitude) + rightX;
            speeds[3] = (speeds[3] * speedMagnitude) + rightX;
            this.speeds = speeds;

            dt.leftDrive.motor1.setVelocity(speeds[0] + pitchCorrection);
            dt.leftDrive.motor2.setVelocity(speeds[1] + pitchCorrection);
            dt.rightDrive.motor1.setVelocity(speeds[2] + pitchCorrection);
            dt.rightDrive.motor2.setVelocity(speeds[3] + pitchCorrection);
        }
        if(stabilization){
            pitchCorrection = pitchPid.power(pitchTarget,imu.getPitch());
        }
    }

    public void enableStabalization(boolean bool){this.stabilization = bool;}

    public void setPitchPid(double kp, double ki, double kd){
        pitchPid.setPID(kp,ki,kd);
    }

    public void setSpeedMultiplier(double speedMultiplier){
        this.speedMultiplier = speedMultiplier;
    }

    public double distanceToCounts(double distance){
        return (distance/(dt.leftDrive.motor1.getEncoder().getWheelCircumference())*dt.getDriveGearReduction()*dt.leftDrive.motor1.getEncoder().getCPR());
        //return (distance/(dt.leftDrive.getEncoder())*dt.getDriveGearReduction()*dt.leftDrive.motor1.getEncoder().getCPR());
    }

    public double countsToDistance(double counts){
        return (counts*dt.leftDrive.getWheelCircumference()*dt.getDrivenGearReduction())/dt.leftDrive.motor1.getEncoder().getCPR();
    }

    public boolean opModeActive(){return auto.getOpModeIsActive();}

}

