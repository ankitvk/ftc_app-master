package org.firstinspires.ftc.teamcode.Control;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public interface Constants {
    double NANOSECONDS_PER_MINUTE = 6e+10;
    double NEVEREST_CLASSIC_MAX_RPM = 6600;
    double NEVEREST20_COUNTS_PER_REV = 560;
    double NEVEREST40_COUNTS_PER_REV = 1120;

    double NEVEREST_CLASSIC_TICKS_PER_ROTATION = 28;
    double DT_GEAR_RATIO = 55/3;
    double DT_GEARBOX_TICKS_PER_ROTATION = DT_GEAR_RATIO*NEVEREST_CLASSIC_TICKS_PER_ROTATION;
    double DT_MAX_RPM = NEVEREST_CLASSIC_MAX_RPM/DT_GEAR_RATIO;

    double PIVOT_TICKS_PER_ROTATION = 100;
    double PIVOT_TICKS_PER_INCH = 100;

    double E4T_COUNTS_PER_REV = 537.6;
    int PATH_FOLLOWING_INTERVAL = 50;
    double SPEED_MULTIPLIER = .85;
    double PATH_FOLLOW_SPEED_MULTIPLIER = .25;

    double LENGTH_BETWEEN_WHEELS = 15.789;
    double WHEEL_DIAMETER  =4;

    double DISTANCE_TOLERANCE = 1.5;
    double IMU_TOLERANCE = .5;
    double ENCODER_TOLERANCE = 50;

    double LOOKAHEAD = 5;

    int ALIGN_POSITION = 55;

    int TARGET_GOLD_X_POS = 425;

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

    String LICENSE_KEY = "AbxcZxf/////AAABmYehhkt9/k+lgWzWDZFG64eH2XR0TK1U2WbaJWSXrI" +
            "BuB4xBU9FpE1oHH3WHomRfTKwInQH9f18rQNXlH0LoYQmYHUR30N4" +
            "7eierR8KeOCKWOz/ew5qYXDq5W4hy20SDt6/GrORnhVttBPrp1lu+RllaRA" +
            "NDz4PKdioMn8uobSLb4MC9z77cXv+BCxSQGsswhgaVNYh/3GoBLrYYEYDpgGGFRL" +
            "EJtrmf8pYbd3s+KKatCS5xEaEoILRYWQZMt5kENV2CyoaSznfg+zmRNeNpVpFBLWMl" +
            "aVoT19fBzj2IZZx209ztGb+MLCRkHwy2b0cTDc1DfBm/RfzcJsAm8ym9sAPTmlbi6UlWB/dJr9s48kwM";

    RevBlinkinLedDriver.BlinkinPattern PATTERN = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE;


}
