package frc.robot;


import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import frc.lib.swerve.SwerveModule.SwerveModuleConstants;

public final class Constants {

    public static double kLooperDt = 0.02; 

    public static class Field {
        public static final double length = 17.55; 
        public static final double width = 8.05; 
    }

    public enum StatusAction {
        InProcess,
        Done,
        Undefined
    }

    

    public static TrajectoryConfig createTrajConfig (double maxVel, double maxAccel) {
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
        config.setStartVelocity(0); 
        config.setEndVelocity(0); 
        return new TrajectoryConfig(maxVel, maxAccel);
    }

    public static class Drive {
        public static final int id_pigeon = 13;
        public static final double track_width = Units.inchesToMeters(21.75); //19.5 in 28
        public static final double wheel_base = Units.inchesToMeters(23.75); //21.5in 31.5

        public enum DriveControlMode {
            Velocity,
            PercentOutput
        }

        public static final double maxVelocity = 4.5; //Empirical Max Velocity
        public static final double maxAngularVelocity = 8.9; //Theoretical Max Angular Velocity

        public static class KinematicLimits {
            public double kMaxDriveVelocity = maxVelocity; 
            public double kMaxAccel = Double.MAX_VALUE; 
            public double kMaxAngularVelocity = maxAngularVelocity; 
            public double kMaxAngularAccel = Double.MAX_VALUE; 
            KinematicLimits () {}
            KinematicLimits (double kMaxDriveVelocity, double kMaxAngularVelocity) { 
              this.kMaxDriveVelocity = kMaxDriveVelocity; 
              this.kMaxAngularVelocity = kMaxAngularVelocity; 
            }
            KinematicLimits (double kMaxDriveVelocity, double kMaxAccel, double kMaxAngularVelocity, double kMaxAngularAccel){
              this.kMaxDriveVelocity = kMaxDriveVelocity; 
              this.kMaxAccel = kMaxAccel;
              this.kMaxAngularVelocity = kMaxAngularVelocity; 
              this.kMaxAngularAccel = kMaxAngularAccel;
            }
        } 
        
        public static final KinematicLimits uncappedLimits = new KinematicLimits();
        public static final KinematicLimits autoLimits = new KinematicLimits(4, 4.2, Math.PI*2, Math.PI*2);  
        public static final KinematicLimits oneMPSLimits = new KinematicLimits(4.5, 3*Math.PI); 
    }

    public static class SwerveModules {
        public static final double steering_gear_ratio = 18.75; 
        public static final double drive_gear_ratio = 5.36;
        public static final double wheelCircumference = Math.PI * Units.inchesToMeters(4);

        //Front Left  por   back left
        public static final SwerveModuleConstants MOD2 = new SwerveModuleConstants(4, 0, 6, -0.47705078125); 
        //Front Right 
        public static final SwerveModuleConstants MOD1 = new SwerveModuleConstants(5, 6, 9, 0.48828125); 
        //Back Left  por                                       front left 
        public static final SwerveModuleConstants MOD0 = new SwerveModuleConstants(1, 2, 3, 0.450439453125); 
        //Back Right 
        public static final SwerveModuleConstants MOD3 = new SwerveModuleConstants(8, 7, 12, -0.499755859375); 

        public static double steer_kP = 2 ; //2
        public static double steer_kI = 0; 
        public static double steer_kD = 1;//0
        public static double steer_kS = 0; 
       
        
        public static double drive_kP = 0.1; 
        public static double drive_kI = 0; 
        public static double drive_kD = 0; 
        public static double drive_kS = 0;
       
       
        

    }
    public static final class MotorConstants {
        public static final int id_ms= 20; 
         public static final int id_mi= 27; 
         public static final int id_er = 0;
         public static final int id_mb =40;
      }
      
}
