package ftc.library.MaelstromSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromMotions.MaelstromMotors.Direction;
import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotor;
import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotorSystem;
import ftc.library.MaelstromMotions.MaelstromMotors.Motor;
import ftc.library.MaelstromMotions.MaelstromServos.CRServo.MaelstromCRServo;
import ftc.library.MaelstromMotions.MaelstromServos.CRServo.MaelstromCRServoSystem;
import ftc.library.MaelstromUtils.SubsystemModels;
import ftc.library.MaelstromUtils.LibConstants;
import ftc.library.MaelstromWrappers.MaelstromController;

public class MaelCollector implements LibConstants, Subsystem {

    //public MaelstromMotorSystem collector;
    public MaelstromMotor oneMotorCollector;
    public MaelstromMotorSystem twoMotorCollectors;
    public MaelstromCRServo oneCrCollector;
    public MaelstromCRServoSystem twoCrCollector;
    public double intakePower, outtakePower;
    public double numOfMotors = 0, numOfCr = 0;
    public Motor motor;
    public SubsystemModels model;
    public State state = State.STOP;

    public enum State{
        INTAKE,
        OUTTAKE,
        STOP
    }

    public State getState(){return state;}

    public void setState(State state){
        this.state = state;
    }

    @Override
    public void update() throws InterruptedException {
        switch(state){
            case INTAKE:
                intake();
                break;
            case OUTTAKE:
                outtake();
                break;
            case STOP:
                stop();
                break;
        }
    }

    public MaelCollector(String name1, Motor motor, SubsystemModels model, HardwareMap hwMap){
        if(model == SubsystemModels.MOTOR){ oneMotorCollector = new MaelstromMotor(name1,motor,hwMap);
            this.motor = motor;
            this.model = model;
            numOfMotors = 1;}

        else if(model == SubsystemModels.CR){oneCrCollector = new MaelstromCRServo(name1,hwMap);
            this.motor = motor;
            this.model = model;
            numOfCr = 1;}
    }

    public MaelCollector(String name1, String name2,
                         Direction direction1, Direction direction2,
                         SubsystemModels model, Motor motor, HardwareMap hwMap){

        if(model == SubsystemModels.MOTOR){
            if(direction1 == Direction.REVERSE) twoMotorCollectors = new MaelstromMotorSystem(name1,name2, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,hwMap,motor);
            else if(direction2 == Direction.REVERSE) twoMotorCollectors = new MaelstromMotorSystem(name1, name2, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, hwMap, motor);
            this.motor = motor;
            this.model = model;
            numOfMotors = 2;
        }
        else if(model == SubsystemModels.CR){
            if(direction1 == Direction.REVERSE){ twoCrCollector = new MaelstromCRServoSystem(name1,name2, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, hwMap);}
            else if(direction2 == Direction.REVERSE) twoCrCollector = new MaelstromCRServoSystem(name1,name2, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,hwMap);
            this.motor = motor;
            this.model = model;
            numOfCr = 2;
        }

    }


    public void DriverControl(MaelstromController controller, MaelstromController.Toggler toggler) {

        if(toggler == MaelstromController.Toggler.OFF){
            if(controller.y()) intake();
            else if(controller.x()) outtake();
            else stop();
        }
        else if(toggler == MaelstromController.Toggler.ON){
            if(controller.yToggle()) intake();
            else if(controller.xToggle()) outtake();
            else stop();
        }
    }

    public void intake(){
        setPower(intakePower);
    }

    public void outtake(){
        setPower(outtakePower);
    }

    public void stop(){
        setPower(0);
    }

    public void setPower(double power){
        if(numOfMotors == 1) oneMotorCollector.setPower(power);
        else if(numOfMotors == 2) twoMotorCollectors.setPower(power);

        if(numOfCr == 1) oneCrCollector.setPower(power);
        else if(numOfCr == 2) twoCrCollector.setPower(power);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        if(numOfMotors == 1) oneMotorCollector.setZeroPowerBehavior(behavior);
        else if(numOfMotors == 2) twoMotorCollectors.setZeroPowerBehavior(behavior);
    }

    public double getPower(){
        double power = 0;
        if(numOfMotors == 1) power += oneMotorCollector.getPower();
        else if(numOfMotors == 2) power += twoMotorCollectors.getPower();
        return power;
    }

    public double getCounts(){
        double counts = 0;
        if(numOfMotors == 1) counts += oneMotorCollector.getCounts();
        else if(numOfMotors == 2) counts+= twoMotorCollectors.getCounts();
        return counts;
    }

/*    public void setDirections(DcMotorSimple.Direction direction1, DcMotorSimple.Direction direction2){
        if(model == SubsystemModels.ONE_MOTOR_COLLECTOR) motorCollector.setDirection(direction1);
        else if(model == SubsystemModels.TWO_MOTOR_COLLECTOR) ;
    }*/

    public void setCollectorPowers(double intakePower, double outtakePower){
        this.intakePower = intakePower;
        this.outtakePower = outtakePower;
    }


}


