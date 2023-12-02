package frc.robot;

public class Constants {

    //there will only be one set of PID constants in the final robot
    public static final class FlywheelPIDConsts {
        public static double pidP = 0.00002018;
        public static double pidI = 0;
        public static double pidD = 0;
    }

    public static final class FeedForwardConst {
        public static double kS = 0.74034;
        public static double kV = 0.000030589;
        public static double kA = 0.000003086;
    }

    public static final class BallHandlerPorts {
        public static final int beamBreakPort = 0;
        public static final int beamBreakPort2 = 9;
        public static final int leftFlywheelPort = 16;
        public static final int rightFlywheelPort = 17;
        public static final int pivotPort = 14;
        public static final int rollerPort = 15;
    }

    public static final class BeamBreak {
        public static final int beamID1 = 0;
        public static final int beamID2 = 9;
    }


    public static final int joystick = 0;

    // public static final int hoodID = 0; (for our final robot)

    public static final int flyCircumference = 0;
    public static final int feedCircumference = 0;

    public static final String BallHandler = null;

}
