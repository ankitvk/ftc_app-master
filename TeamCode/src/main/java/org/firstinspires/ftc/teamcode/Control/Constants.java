package org.firstinspires.ftc.teamcode.Control;

public interface Constants {
    double NANOSECONDS_PER_MINUTE = 6e+10;
    double NEVEREST_20_MAX_RPM = 320;
    double NEVEREST_COUNTS_PER_REV = 1440;
    double E4T_COUNTS_PER_REV = 537.6;
    int PATH_FOLLOWING_INTERVAL = 50;
    double SPEED_MULTIPLIER = 1;

    double LENGTH_BETWEEN_WHEELS = 15.789;
    double WHEEL_DIAMETER  =4;

    double LOOKAHEAD = 18;

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

    double topPivotKP = 20;
    double topPivotKI = 0;
    double topPivotKD = 0;
    double topPivotMaxI = 1;

    double extensionLeftKP = 20;
    double extensionLeftKI = 0;
    double extensionLeftKD = 0;
    double extensionLeftMaxI = 1;

    double extensionRightKP = 20;
    double extensionRightKI = 0;
    double extensionRightKD = 0;
    double extensionRightMaxI = 1;

    double intakeKP = 20;
    double intakeKI = 0;
    double intakeKD = 0;
    double intakeMaxI = 1;
}
