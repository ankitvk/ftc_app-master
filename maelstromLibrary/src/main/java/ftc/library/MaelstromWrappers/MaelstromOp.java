package ftc.library.MaelstromWrappers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import ftc.library.MaelstromMotions.MaelstromMotors.Direction;
import ftc.library.MaelstromRobot;

/*custom class for regular op modes*/
public abstract class MaelstromOp extends OpMode {
    public MaelstromRobot robot;
    public Direction forward = Direction.FORWARD;
    public Direction backward = Direction.BACKWARD;
    public Direction left = Direction.LEFT;
    public Direction right = Direction.RIGHT;
    public Direction def = Direction.DEFAULT;
    public Direction in = Direction.IN;
    public Direction out = Direction.OUT;
    public Direction unknown  = Direction.UNKNOWN;
    public MaelstromTelemetry feed = new MaelstromTelemetry(super.telemetry);
    protected MaelstromController controller1 = new MaelstromController(super.gamepad1,"controller1");
    protected MaelstromController controller2 = new MaelstromController(super.gamepad2,"controller2");
}
