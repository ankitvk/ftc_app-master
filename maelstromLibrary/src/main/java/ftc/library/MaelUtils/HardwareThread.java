package ftc.library.MaelUtils;

import java.util.ArrayList;

import ftc.library.MaelSubsystems.Subsystem;
import ftc.library.MaelWrappers.MaelLinearOp;

public class HardwareThread extends Thread {
    private final ArrayList<Subsystem> subsystems;
    private final MaelLinearOp opMode;

    public HardwareThread(ArrayList<Subsystem> subsystems, MaelLinearOp opMode){
        this.subsystems = subsystems;
        this.opMode = opMode;
        run();
    }

    @Override
    public void run(){
        try {
            while(!opMode.isStopRequested()){
                for(Subsystem s : subsystems){
                    s.update();
                }
            }
        } catch(InterruptedException ex){
            Thread.currentThread().interrupt();
        }
    }
}
