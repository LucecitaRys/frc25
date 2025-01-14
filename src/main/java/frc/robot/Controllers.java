package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controllers {
      public static final CommandXboxController control0 = new CommandXboxController(0);
 

     public static double getLeftY_0 (){
        return MathUtil.applyDeadband(-control0.getLeftY(), 0.2); 
    }
    public static double getRightY_0 (){
        return MathUtil.applyDeadband(-control0.getRightY(), 0.2); 
    }
    public static double getLT(){
        return control0.getLeftTriggerAxis();
    }
    public static double getRT(){
        return control0.getRightTriggerAxis();
    }
}
