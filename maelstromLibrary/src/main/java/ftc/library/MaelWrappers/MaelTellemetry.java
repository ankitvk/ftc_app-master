package ftc.library.MaelWrappers;



import org.firstinspires.ftc.robotcore.external.Telemetry;

import ftc.library.MaelUtils.MaelUtils;

import static java.lang.Thread.sleep;

/*custom telemetry class*/
public class MaelTellemetry {
    private int length;
    private Telemetry telemetry;
    private MaelUtils.AutonomousOpMode auto;
    private boolean open;
    private static MaelTellemetry instance;

    public MaelTellemetry(Telemetry telemetry, MaelUtils.AutonomousOpMode auto){
        this.telemetry = telemetry;
        this.auto = auto;
        instance = this;
    }
    public MaelTellemetry(Telemetry telemetry){
        this.telemetry = telemetry;
        instance = this;
    }

    public static MaelTellemetry getFeed(){
        return instance;
    }

    public void add(String string){
        telemetry.addLine(string);
    }

    public void add(String string, Object value){
        telemetry.addData(string,value);
    }

    public void update(){
        telemetry.update();
    }
    public void startUpdate(){
        Runnable runnable = new Runnable() {
            @Override
            public void run() {
                while(auto.getOpModeIsActive() && open){
                    update();
                    try {
                        sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        };
        Thread thread = new Thread(runnable);
        thread.start();
    }
    public void setNewFirst() {
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
    }

    public void addBlankLine(){
        telemetry.addLine("");
    }


}

