package org.firstinspires.ftc.teamcode.Control;

public interface Constants {
    double NANOSECONDS_PER_MINUTE = 6e+10;
    double NEVEREST_20_MAX_RPM = 320;
    double NEVEREST20_COUNTS_PER_REV = 560;
    double NEVEREST40_COUNTS_PER_REV = 1120;
    double E4T_COUNTS_PER_REV = 537.6;
    int PATH_FOLLOWING_INTERVAL = 50;
    double SPEED_MULTIPLIER = 1;

    double LENGTH_BETWEEN_WHEELS = 15.789;
    double WHEEL_DIAMETER  =4;

    double DISTANCE_TOLERANCE = 2;
    double IMU_TOLERANCE = 10;
    double PIVOT_TOLERANCE = 15;

    double LOOKAHEAD = 18;

    int ALIGN_POSITION = 50;

    double ROTATIONS_FOR_EXTENSION = 5.5;

    double dtKP = .03;
    double dtKI = 0;
    double dtKD = 0;
    double dtMaxI = 1;

    double topPivot1KP = 20;
    double topPivot1KI = 0;
    double topPivot1KD = 0;
    double topPivot1MaxI = 1;

    double topPivot2KP = 20;
    double topPivot2KI = 0;
    double topPivot2KD = 0;
    double topPivot2MaxI = 1;

    double extensionKP = .05;
    double extensionKI = 0;
    double extensionKD = 0;
    double extensionMaxI = 1;

    double pivotKP = .1;
    double pivotKI = 0;
    double pivotKD = 0;
    double pivotMaxI = 1;



}
