package ftc.library.MaelUtils;

public enum TimeUnits{
    NANOSECS(1),
    MILLISECS(1E6),
    SECS(1E9);
    public final double value;
    TimeUnits (double value){this.value = value;}
}
