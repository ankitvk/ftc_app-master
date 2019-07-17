package ftc.library.MaelControl;

import ftc.library.MaelUtils.MaelUtils;

public abstract class StateMachineRunner {
    public int programStage = 0;
    public boolean stageFinished = false;
    private int nextStage = 0;

    public void nextStage(int ordinal){
        nextStage = ordinal;
        incrementStage();
    }

    public void nextStage(){
        nextStage(programStage + 1);
/*        System.out.println("Stage " + programStage + " Complete");
        System.out.println();*/
    }

    private void incrementStage(){
        programStage = nextStage;
        stageFinished = true;
    }

    public void loopStateMachine(){
        while(!MaelUtils.linearOpMode.isStopRequested()){
            MainStateMachine();

            if(programStage == getOrdinalLength()) break;
        }
    }

    public abstract int getOrdinalLength();

    public void initializeStateVariables(){
        if(stageFinished) stageFinished = false;
    }

    public abstract void MainStateMachine();

}
