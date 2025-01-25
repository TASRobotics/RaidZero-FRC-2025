package raidzero.robot;

public class Constants {
    public class Climb {
        public class Joint {
            public static final int MOTOR_ID = 15;
            
            public static final double KP = 2.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;
            public static final double KF = 0.5;

            public static final double CLIMB_SETPOINT = 0.6;

            public static final int CURRENT_LIMIT = 20;
        }

        public class Winch {
            public static final int MOTOR_ID = 14;
        }
    } 
}
