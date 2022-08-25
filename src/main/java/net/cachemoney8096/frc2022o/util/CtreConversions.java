package net.cachemoney8096.frc2022o.util;

public class CtreConversions {
    public static double rpmToTicks(double rpm){
        var rps = rpm / 60d;
        var rp100ms = rps / 10;
        return rp100ms * 2048;
    }

    public static double ticksToRpm(double ticks){
        return ticks / 2048 * 10 * 60;
    }
}
