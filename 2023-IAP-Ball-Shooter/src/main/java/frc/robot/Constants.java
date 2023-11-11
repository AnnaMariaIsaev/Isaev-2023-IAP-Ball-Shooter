package frc.robot;

public class Constants {

    //there will only be one set of PID constants in the final robot
    public static final class leftFlywheelPIDConsts {
        public static double pidP = 0.07;
        public static double pidI = 0;
        public static double pidD = 0;
    }

    public static final class rightFlywheelPIDConsts {
        public static double pidP = 0.07;
        public static double pidI = 0;
        public static double pidD = 0;
    }

    public static final class BallHandlerPorts {
        public static final int beamBreakPort = 1;
        public static final int beamBreakPort2 = 2;
        public static final int leftFlywheelPort = 16;
        public static final int rightFlywheelPort = 17;
        public static final int pivotPort = 14;
        public static final int rollerPort = 15;
    }

    public static final class pivotPIDConsts {
        public static final double pidP = 0.05;
        public static final double pidI = 0;
        public static final double pidD = 0;
    }

    public static final class BeamBreak {
        public static final int beamID1 = 0;
        public static final int beamID2 = 0;
    }


    public static final int joystick = 0;

    // public static final int hoodID = 0; (for our final robot)

    public static final int flyCircumference = 0;
    public static final int feedCircumference = 0;

    public static final String BallHandler = null;

}
