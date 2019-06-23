package ftc.library.MaelWrappers;


public abstract class MaelIterative extends MaelLinearOp {


    @Override
    public void run() throws InterruptedException {
        initLoop();
        waitForStart();
        while (!isStopRequested()) teleop();
    }

    public abstract void initLoop();

    public abstract void teleop();

    public abstract void displayData();
}
