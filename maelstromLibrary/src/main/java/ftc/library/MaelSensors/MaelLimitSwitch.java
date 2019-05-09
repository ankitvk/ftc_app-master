package ftc.library.MaelSensors;


import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*Custom class for limit switches*/
public class MaelLimitSwitch {

    private DigitalChannel limitSwitch;
    private String name;

    private boolean signalValue;
    private boolean state;

    public MaelLimitSwitch(String name, HardwareMap hwMap) {
        this.name = name;
        limitSwitch = hwMap.digitalChannel.get(name);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void updateState(){
        signalValue = limitSwitch.getState();
        state = !signalValue;
    }

    public boolean getSignalValue(){
        updateState();
        return signalValue;
    }

    public boolean getState(){
        updateState();
        return state;
    }

    public boolean pressed(){ return !getState(); }

    public boolean released(){
        boolean released = false;
        while (pressed()) released = false;
        while(!pressed()) released = true;
        return released;
    }
    public String getName(){return name;}


}