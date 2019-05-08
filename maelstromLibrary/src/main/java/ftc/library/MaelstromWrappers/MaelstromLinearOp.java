package ftc.library.MaelstromWrappers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

import ftc.library.MaelstromMotions.MaelstromMotors.Direction;
import ftc.library.MaelstromUtils.MaelstromUtils;

/*custom class for linear op mode */
public abstract class MaelstromLinearOp extends LinearOpMode {
    protected MaelstromTellemetry feed;
    protected MaelstromController controller1, controller2;
    public Direction forward = Direction.FORWARD;
    public Direction backward = Direction.BACKWARD;
    public Direction left = Direction.LEFT;
    public Direction right = Direction.RIGHT;
    public Direction def = Direction.DEFAULT;
    public Direction unknown = Direction.UNKNOWN;

    public final void runOpMode() throws InterruptedException {
        try{
            feed = new MaelstromTellemetry(super.telemetry);
            feed.setNewFirst();
            MaelstromUtils.setLinearOpMode(this);
            controller1 = new MaelstromController(super.gamepad1,"controller1");
            controller2 = new MaelstromController(super.gamepad2,"controller2");
            runLinearOpMode();
        }
        finally {
            stopLinearOpMode();
        }
    }



    public abstract void runLinearOpMode() throws InterruptedException;
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
            Thread.sleep((long) timeMilli * 1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}
