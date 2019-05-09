package ftc.library.MaelSensors;

import ftc.library.MaelMotions.MaelMotors.MaelMotor;
import ftc.library.MaelMotions.MaelMotors.Motor;

/*Custom class for motor encoder*/
public class MaelstromEncoder {
    private Motor model;
    private MaelMotor motor;
    private double wheelDiameter = 0, gearRatio;
    private double currCounts, zeroPos;
    private double countsPerInch;

    public MaelstromEncoder(MaelMotor motor, Motor model){
        this.model = model;
        this.motor = motor;
    }

    public double getCountsPerInch(){
        countsPerInch = (model.CPR() / (wheelDiameter * Math.PI)) * gearRatio;
        return countsPerInch;
    }

    public double getRelativePosition(){
        currCounts = (int) (getPosition() - zeroPos);
        return currCounts;
    }

    public double getPosition(){
        return motor.getCounts();
    }

    public double getInches(){
        return ((Math.PI * wheelDiameter * getRelativePosition()) / getCPR()) * gearRatio;
    }

    public double getCPR(){ return model.CPR(); }
    public double getRPM(){return model.RPM();}

    public void resetEncoder(){
        zeroPos = (int) getPosition();
        currCounts = 0;
    }

    public double getWheelCircumference(){
        double circumference = getWheelDiameter() * Math.PI;
        return circumference;
    }

    public void setWheelDiameter(double wheelDiameter){this.wheelDiameter = wheelDiameter;}
    public void setGearRatio(double gearRatio){this.gearRatio = gearRatio;}
    public void setType(Motor model){ this.model = model;}
    public double getWheelDiameter(){return wheelDiameter;}
    public double getGearRatio(){return gearRatio;}

}
