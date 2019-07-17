package ftc.library.MaelWrappers;

import ftc.library.MaelUtils.LibConstants;

public abstract class MaelAutoBase extends MaelLinearOp implements LibConstants {
    public int programStage = 0;
    public boolean stageFinished = false;
    private int nextStage = 0;

    @Override
    public void run() throws InterruptedException {
        loopStateMachine();
    }

    @Override
    public void initHardware() {
        hardwareInit();
    }

    public void nextStage(int ordinal){
        nextStage = ordinal;
        incrementStage();
    }

    public void nextStage(){
        nextStage(programStage + 1);
        System.out.println("Stage " + programStage + " Complete");
        System.out.println();
    }

    private void incrementStage(){
        programStage = nextStage;
        stageFinished = true;
    }

    private void loopStateMachine(){
        while(!isStopRequested()){
            MainStateMachine();

            if(programStage == getOrdinalLength()) break;
        }
    }

    public abstract int getOrdinalLength();

    public void initializeStateVariables(){
        if(stageFinished) stageFinished = false;
    }

    public abstract void MainStateMachine();

    public abstract void hardwareInit();
}
