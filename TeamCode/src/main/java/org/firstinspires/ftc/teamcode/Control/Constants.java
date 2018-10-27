package org.firstinspires.ftc.teamcode.Control;

public interface Constants {
    double NANOSECONDS_PER_MINUTE = 6e+10;
    double NEVEREST_20_MAX_RPM = 320;
    double NEVEREST20_COUNTS_PER_REV = 560;
    double E4T_COUNTS_PER_REV = 537.6;
    int PATH_FOLLOWING_INTERVAL = 50;
    double SPEED_MULTIPLIER = 1;

    double LENGTH_BETWEEN_WHEELS = 15.789;
    double WHEEL_DIAMETER  =4;

    double LOOKAHEAD = 18;

    double ROTATIONS_FOR_EXTENSION = 5.5;

    double frontLeftKP = 20;
    double frontLeftKI = 0;
    double frontLeftKD = 0;
    double frontLeftMaxI = 1;

    double frontRightKP = 20;
    double frontRightKI = 0;
    double frontRightKD = 0;
    double frontRightMaxI = 1;

    double backLeftKP = 20;
    double backLeftKI = 0;
    double backLeftKD = 0;
    double backLeftMaxI = 1;

    double backRightKP = 20;
    double backRightKI = 0;
    double backRightKD = 0;
    double backRightMaxI = 1;

    double topPivot1KP = 20;
    double topPivot1KI = 0;
    double topPivot1KD = 0;
    double topPivot1MaxI = 1;

    double topPivot2KP = 20;
    double topPivot2KI = 0;
    double topPivot2KD = 0;
    double topPivot2MaxI = 1;

    double extensionKP = .1;
    double extensionKI = 0;
    double extensionKD = 0;
    double extensionMaxI = 1;


}
