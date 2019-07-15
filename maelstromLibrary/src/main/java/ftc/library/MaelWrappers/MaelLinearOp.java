package ftc.library.MaelWrappers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

import ftc.library.MaelMotions.MaelMotors.Direction;
import ftc.library.MaelSubsystems.Subsystem;
import ftc.library.MaelUtils.HardwareThread;
import ftc.library.MaelUtils.LibConstants;
import ftc.library.MaelUtils.MaelUtils;

/*custom class for linear op mode */
public abstract class MaelLinearOp extends LinearOpMode implements LibConstants {
    protected MaelTellemetry feed;
    protected MaelController controller1, controller2;
    public Direction forward = Direction.FORWARD;
    public Direction backward = Direction.BACKWARD;
    public Direction left = Direction.LEFT;
    public Direction right = Direction.RIGHT;
    public Direction def = Direction.DEFAULT;
    public Direction unknown = Direction.UNKNOWN;
    public HardwareThread hardwareThread;

    final public void runOpMode() throws InterruptedException {
        try{
            feed = new MaelTellemetry(super.telemetry);
            feed.setNewFirst();
            MaelUtils.feed = feed;
            MaelUtils.setLinearOpMode(this);
            controller1 = new MaelController(super.gamepad1,"controller1");
            controller2 = new MaelController(super.gamepad2,"controller2");
            initHardware();
            hardwareThread = new HardwareThread(MaelUtils.subsystems,this);
            run();
        }
        finally {
            stopLinearOpMode();
        }
    }



    public abstract void run() throws InterruptedException;

    public abstract void initHardware();
    public void stopLinearOpMode() {}
    public void runSimultaneously(Runnable r1, Runnable r2){
        Thread t1 = new Thread(r1);
        Thread t2 = new Thread(r2);
        t1.start();
        t2.start();
        while(opModeIsActive() && (t1.isAlive() || t2.isAlive())){
            idle();
        }
    }
    public void runSimultaneously(Runnable... runnables){
        List<Thread> threads = new ArrayList<>();
        int i = 0;
        for (Runnable runnable : runnables) {
            threads.add(new Thread(runnable));
            threads.get(i).start();
            i++;
        }
        int count = 0;
        while (opModeIsActive() && count < i) {
            count = 0;
            for(Thread t : threads) if (!t.isAlive()) count++;
        }
    }

    public void sleep(int timeSeconds){
        try {
            Thread.sleep((long) timeSeconds * 1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void sleep(double timeSeconds) {
        try {
            Thread.sleep((long) timeSeconds * 1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void sleep(float timeMilli) {
        try {
            Thread.sleep((long) timeMilli);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}
