package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public interface Constants {
    double NANOSECONDS_PER_MINUTE = 6e+10;
    double NEVEREST_CLASSIC_MAX_RPM = 6600;
    double NEVEREST20_COUNTS_PER_REV = 560;
    double NEVEREST40_COUNTS_PER_REV = 1120;

    double NEVEREST_CLASSIC_TICKS_PER_ROTATION = 28;
    double DT_GEAR_RATIO = 55/3;
    double mecanum_ratio = 1;
    double DT_GEARBOX_TICKS_PER_ROTATION = DT_GEAR_RATIO*NEVEREST_CLASSIC_TICKS_PER_ROTATION;
    double DT_MAX_RPM = NEVEREST_CLASSIC_MAX_RPM/DT_GEAR_RATIO;
    double PIVOT_GEAR_RATIO = 235;
    double PIVOT_LIMIT_ANGLE = 48.1;

    double PIVOT_TICKS_PER_ROTATION = PIVOT_GEAR_RATIO*NEVEREST_CLASSIC_TICKS_PER_ROTATION;
    double PIVOT_TICKS_PER_INCH = 100;

    double E4T_COUNTS_PER_REV = 537.6;
    int PATH_FOLLOWING_INTERVAL = 50;
    double SPEED_MULTIPLIER = .85;
    double TURN_MULTIPLIER = .65;
    double PATH_FOLLOW_SPEED_MULTIPLIER = .25;

    double LENGTH_BETWEEN_WHEELS = 15.789;
    double WHEEL_DIAMETER  =4;

    double DISTANCE_TOLERANCE = 1.5;
    double IMU_TOLERANCE = .5;
    double ENCODER_TOLERANCE = 50;

    double LOOKAHEAD = 5;

    int ALIGN_POSITION = 55;

    int TARGET_GOLD_X_POS = 425;

    int ledCount = 150;

    double MIN_ROTATE_POWER = 0.39;
    double MIN_DRIVE_POWER = 0;

    double dtKP = .000175;
    double dtKI = /*.035*/0.01;
    double dtKD = 0;
    double dtMaxI = 1;

    double dtRotateKP = 0.012;
    double dtRotateKI = 1;
    double dtRotateKD = 0;
    double dtRotateMaxI = 1;

    double dtBigRotateKP = 0.01285;
    double dtBigRotateKI = 2;
    double dtBigRotateKD = 0;
    double dtBigRotateMaxI = 1;

    double extensionKP = .015;
    double extensionKI = 0;
    double extensionKD = 0;
    double extensionMaxI = 1;

    double pivotKP = 1e-6;
    double pivotKI = 0;
    double pivotKD = 0;
    double pivotMaxI = 1;

    //tarun's stuff
    double PIVOT_UP = 1;
    double PIVOT_DOWN = -1;

    double INTAKE_POWER = 1;
    double OUTTAKE_POWER = -1;

    double LIFT_EXTEND = 1;
    double LIFT_RETRACT = -1;

    double distanceKP = 0.00125;
    double distanceKI = 0.1;
    double distanceKD = 0;
    double distanceMaxI = 1;

    //angle correction PID
    double angleCorrectionKP = 0.001;
    double angleCorrectionKI = 0;
    double angleCorrectionKD = 0;
    double angleCorrectionMaxI = 1;

    //turn angle (< 50) PID
    double turnKP = 0.03;
    double turnKI = 0;
    double turnKD = 0;
    double turnMaxI = 1;

    //turn big angle (> 50) PID
    double turnBigKP = 0.039;
    double turnBigKI = 0;
    double turnBigKD = 0;
    double turnBigMaxI = 1;

    //test turn PID
    double testTurnKP = 0.002;
    double testTurnKI = 0;
    double testTurnKD = 0;
    double testTurnMaxI = 1;

    double bigTestTurnKP = 0.013;
    double bigTestTurnKI = 0;
    double bigTestTurnKD = 0;
    double bigTestTurnMaxI = 1;

    //turn one side PID
    double sideKP = 0.001;
    double sideKI = 0;
    double sideKD = 0;
    double sideMaxI = 1;

    double bigSideKP = 0.001;
    double bigSideKI = 0;
    double bigSideKD = 0;
    double bigSideMaxI = 1;

    double velocityKp = .0031;
    double velocityKi = 0;
    double velocityKd = 0;

    double distanceKp = .03;
    double distanceKi = 0;
    double distanceKd = .00001;
    //private double kp = .029, ki = 0, kd = 0.00003;

    double turnKp = .03;
    double turnKi = 0;
    double turnKd = 0.000011;

    double dtMaxRpm = 340;

    double AUTO_SPEED_MULTIPLIER = 1;


    String LICENSE_KEY = "AbxcZxf/////AAABmYehhkt9/k+lgWzWDZFG64eH2XR0TK1U2WbaJWSXrI" +
            "BuB4xBU9FpE1oHH3WHomRfTKwInQH9f18rQNXlH0LoYQmYHUR30N4" +
            "7eierR8KeOCKWOz/ew5qYXDq5W4hy20SDt6/GrORnhVttBPrp1lu+RllaRA" +
            "NDz4PKdioMn8uobSLb4MC9z77cXv+BCxSQGsswhgaVNYh/3GoBLrYYEYDpgGGFRL" +
            "EJtrmf8pYbd3s+KKatCS5xEaEoILRYWQZMt5kENV2CyoaSznfg+zmRNeNpVpFBLWMl" +
            "aVoT19fBzj2IZZx209ztGb+MLCRkHwy2b0cTDc1DfBm/RfzcJsAm8ym9sAPTmlbi6UlWB/dJr9s48kwM";

    RevBlinkinLedDriver.BlinkinPattern PATTERN = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE;


}