package org.firstinspires.ftc.teamcode.Control;

public interface Constants {
    double NANOSECONDS_PER_MINUTE = 6e+10;
    double NEVEREST_20_MAX_RPM = 320;
    double NEVEREST20_COUNTS_PER_REV = 560;
    double NEVEREST40_COUNTS_PER_REV = 1120;
    double E4T_COUNTS_PER_REV = 537.6;
    int PATH_FOLLOWING_INTERVAL = 50;
    double SPEED_MULTIPLIER = .75;

    double LENGTH_BETWEEN_WHEELS = 15.789;
    double WHEEL_DIAMETER  =4;

    double DISTANCE_TOLERANCE = 1;
    double IMU_TOLERANCE = 5;

    double LOOKAHEAD = 18;

    int ALIGN_POSITION = -75;


    double dtKP = .001;
    double dtKI = 0;
    double dtKD = 0;
    double dtMaxI = 1;

    double extensionKP = .015;
    double extensionKI = 0;
    double extensionKD = 0;
    double extensionMaxI = 1;




}
