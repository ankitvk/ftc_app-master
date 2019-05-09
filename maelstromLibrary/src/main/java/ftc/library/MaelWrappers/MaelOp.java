package ftc.library.MaelWrappers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import ftc.library.MaelMotions.MaelMotors.Direction;
import ftc.library.MaelRobot;

/*custom class for regular op modes*/
public abstract class MaelOp extends OpMode {
    public MaelRobot robot;
    public Direction forward = Direction.FORWARD;
    public Direction backward = Direction.BACKWARD;
    public Direction left = Direction.LEFT;
    public Direction right = Direction.RIGHT;
    public Direction def = Direction.DEFAULT;
    public Direction unknown  = Direction.UNKNOWN;
    public MaelTellemetry feed = new MaelTellemetry(super.telemetry);
    protected MaelController controller1 = new MaelController(super.gamepad1,"controller1");
    protected MaelController controller2 = new MaelController(super.gamepad2,"controller2");
}
