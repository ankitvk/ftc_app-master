package ftc.library.MaelMotions.MaelServos.CRServo;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

/*custom class for setting up  cr servo systems*/
public class MaelCRServoSystem {
    private MaelCRServo servo1, servo2, servo3, servo4;
    private List<MaelCRServo> servos;

    public MaelCRServoSystem(String name1, String name2, HardwareMap hwMap){
        this( new MaelCRServo(name1,hwMap), new MaelCRServo(name2, hwMap));
    }

    public MaelCRServoSystem(String name1, String name2, CRServo.Direction direction1, CRServo.Direction direction2, HardwareMap hwMap){
        this (new MaelCRServo(name1,direction1,hwMap),new MaelCRServo(name2,direction2,hwMap));
    }
    public MaelCRServoSystem(String name1, String name2, String name3, HardwareMap hwMap){
        this ( new MaelCRServo(name1,hwMap), new MaelCRServo(name2, hwMap), new MaelCRServo(name3, hwMap));
    }
    public MaelCRServoSystem(String name1, String name2, String name3, String name4, HardwareMap hwMap){
        this ( new MaelCRServo(name1,hwMap), new MaelCRServo(name2, hwMap), new MaelCRServo(name3, hwMap), new MaelCRServo(name4,hwMap));
    }

    public MaelCRServoSystem(MaelCRServo one, MaelCRServo two){
        servo1 = one; servo2 = two; servo3 = null; servo4 =null;
        servos = Arrays.asList(servo1, servo2);
    }
    public MaelCRServoSystem(MaelCRServo one, MaelCRServo two, MaelCRServo three){
        servo1 = one; servo2 = two; servo3 = three; servo4 =null;
        servos = Arrays.asList(servo1,servo2,servo3);
    }
    public MaelCRServoSystem(MaelCRServo one, MaelCRServo two, MaelCRServo three, MaelCRServo four){
        servo1 = one; servo2 = two; servo3 = three; servo4 = four;
        servos = Arrays.asList(servo1,servo2,servo3,servo4);
    }

    public void setPower(double power){for (MaelCRServo servo : servos) servo.setPower(power);}

    public double getPower(){return servo1.getPower();}

    public void setDirection(CRServo.Direction direction){
        for(MaelCRServo servo : servos){
            servo.setDirection(direction);
        }
    }

    public void setDirection(CRServo.Direction direction1,CRServo.Direction direction2, CRServo.Direction direction3, CRServo.Direction direction4 ){
        servo1.setDirection(direction1);
        servo2.setDirection(direction2);
        servo3.setDirection(direction3);
        servo4.setDirection(direction4);
    }

    public void sleep(int time) throws InterruptedException {for (MaelCRServo servo : servos) servo.wait(time);}

    public void setPower(double p1, double p2){servo1.setPower(p1); servo2.setPower(p2);}



}
