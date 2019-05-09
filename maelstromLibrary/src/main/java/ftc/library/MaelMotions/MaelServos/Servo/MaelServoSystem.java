package ftc.library.MaelMotions.MaelServos.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

/*custom class for creating set-position servo systems*/
public class MaelServoSystem {
    private MaelServo servo1;
    private MaelServo servo2;
    private MaelServo servo3;
    private MaelServo servo4;
    private int offset;
    private double position;
    private int time;
    private List<MaelServo> servos;

    public MaelServoSystem(String name1, String name2, HardwareMap hwMap) {
        this(new MaelServo(name1,hwMap),new MaelServo(name2, hwMap));
    }
    public MaelServoSystem(String name1, String name2, String name3, String name4, HardwareMap hwMap){
        this(new MaelServo(name1,hwMap),new MaelServo(name2, hwMap), new MaelServo(name3,hwMap), new MaelServo(name4,hwMap));
    }
    public MaelServoSystem(String name1, String name2, String name3, HardwareMap hwMap){
        this(new MaelServo(name1,hwMap),new MaelServo(name2, hwMap), new MaelServo(name3,hwMap));
    }
    public MaelServoSystem(String name1, String name2, String name3, String name4,
                           Servo.Direction direction1, Servo.Direction direction2, Servo.Direction direction3, Servo.Direction direction4,
                           HardwareMap hwMap){
        this(new MaelServo(name1,direction1, hwMap),new MaelServo(name2,direction2, hwMap),new MaelServo(name3,direction3, hwMap), new MaelServo(name4,direction4, hwMap));
    }
    public MaelServoSystem(MaelServo one, MaelServo two){
        servo1 = one; servo2 = two; servo3 = null; servo4 = null;
        servos = Arrays.asList(servo1,servo2);
    }
    public MaelServoSystem(MaelServo one, MaelServo two, MaelServo three){
        servo1 = one; servo2 = two; servo3 = three; servo4 = null;
        servos = Arrays.asList(servo1,servo2,servo3);
    }
    public MaelServoSystem(MaelServo one, MaelServo two, MaelServo three, MaelServo four){
        servo1 = one; servo2 = two; servo3 = three; servo4 = four;
        servos = Arrays.asList(servo1, servo2, servo3, servo4);
    }

    public double getPos(){return position;}

    public void setPos(double position){
        this.position = position;
        for(MaelServo servo : servos){
            servo.setPos(position);
        }
    }

    public void setPos(double position, int time){
        this.position = position;
        this.time = time;
        for(MaelServo servo : servos){
            servo.setPos(position,time);
        }
    }

    public void sleep(int time) throws InterruptedException{
        for (MaelServo servo : servos){
            servo.sleep(time);
        }
    }






}
