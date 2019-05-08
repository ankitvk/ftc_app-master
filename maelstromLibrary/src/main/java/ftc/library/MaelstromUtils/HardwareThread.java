package ftc.library.MaelstromUtils;

import java.util.List;

import ftc.library.MaelstromSubsystems.Subsystem;
import ftc.library.MaelstromWrappers.MaelstromLinearOp;

public class HardwareThread extends Thread {
    private final List<Subsystem> subsystems;
    private final MaelstromLinearOp opMode;

    public HardwareThread(List<Subsystem> subsystems,MaelstromLinearOp opMode){
        this.subsystems = subsystems;
        this.opMode = opMode;
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
