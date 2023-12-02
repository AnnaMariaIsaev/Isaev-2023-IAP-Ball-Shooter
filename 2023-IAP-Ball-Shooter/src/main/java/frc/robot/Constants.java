package frc.robot;

public class Constants {

    //there will only be one set of PID constants in the final robot
    public static final class FlywheelPIDConsts {
        public static double pidP = 0.065619;
        public static double pidI = 0;
        public static double pidD = 0;
    }

    public static final class FeedForwardConst {
        //position, velocity, acceleration
        public static double kS = 0.80824;
        public static double kV = 0.12407;
        public static double kA = 0.015099;
    }

    public static final class BeamBreak {
        public static final int beamID1 = 1;
        public static final int beamID2 = 9;
    }

    public static final int joystick = 0;
    public static final double flyCircumference = 31.42; //in cm

}
