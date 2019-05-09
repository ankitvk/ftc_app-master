package ftc.library.MaelMotions.MaelServos.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import ftc.library.MaelSensors.MaelTimer;
import ftc.library.MaelUtils.MaelUtils;
import ftc.library.MaelUtils.TimeUnits;

/*custom class for set-position servos*/
public class MaelServo {
    private Servo servo;
    private String nameServo;
    MaelTimer timer = new MaelTimer();
    private double targetPos;
    private double max = 1, min = 0;
    private MaelUtils.AutonomousOpMode auto;

    public MaelServo(String name, HardwareMap hwMap){
        this.nameServo = name;
        servo = hwMap.servo.get(name);
    }
    public MaelServo(String name, Servo.Direction direction, HardwareMap hwMap){
        this.nameServo = name;
        servo = hwMap.servo.get(name);
        servo.setDirection(direction);
    }

    public void setPos(double position){
        targetPos = position;
        position = ((max - min)*position) + min;
        servo.setPosition(position);
    }
    public double getPos(){
        return servo.getPosition();
    }

    public void setPos(double position, int time){
        for (double i = servo.getPosition(); i < position; i += 0.01){
            if (!opModeActive())break;

            else setPos(i);
            try {Thread.sleep(time);}
            catch (InterruptedException e){}
        }
    }

    public boolean isStalled( int time){
        boolean isStalled = false;
        double prePos = getPos();
        if (getPos() == prePos
                && !timer.elapsedTime(time, TimeUnits.SECS)) isStalled = true;
        return isStalled;
    }

    public void sleep(int time) throws InterruptedException{servo.wait(time);}

    public boolean opModeActive(){
        return auto.getOpModeIsActive();
    }

}
