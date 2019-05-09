package ftc.library.MaelControl;


import java.util.List;

public class StateMachineRunner {

    List states;
    private int currState;
    private int stateDelay;
    private long previousTime = 0;

    public StateMachineRunner(int stateDelay) {
        currState = 0;
        this.stateDelay = stateDelay;
    }

    public StateMachineRunner(List states, int stateDelay) {
        this.states = states;
        this.stateDelay = stateDelay;
        currState = 0;
    }

 /*   public void runStateMachine() {
        if (states != null && currState < states.size() && delayDone()) {
            switch (states.get(currState)) {
                case turn90Clockwise:
                    if (Robot.driveTrain.turnPOM(90, DriveTrain.Direction.CLOCKWISE)) {
                        nextState();
                    }
                    break;
                case drive10Inches:
                    if (Robot.driveTrain.moveGyroDistancePOM(10, DriveTrain.Direction.FORWARD, 1, 0)) {
                        nextState();
                    }
                    break;
            }
        } else {
            System.out.println("States finished or not initialized");
        }
    }*/

    public void setStates(List states) {
        this.states = states;
        currState = 0;
    }

    public void addStates(boolean... states) {
        for (boolean state : states) {
            this.states.add(state);
        }

    }


    public void nextState() {
        currState++;
        previousTime = System.currentTimeMillis();

    }

    public void setStateDelay(int stateDelay) {
        this.stateDelay = stateDelay;
    }

    public int getStateDelay() {
        return stateDelay;
    }

    public boolean delayDone() {
        return System.currentTimeMillis() - previousTime >= stateDelay;
    }

}
