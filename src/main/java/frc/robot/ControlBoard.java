package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

public class ControlBoard {
    public static final XboxController driver = new XboxController(0); 
    public static final XboxController operator = new XboxController(1); 


    public static DoubleSupplier getLeftYC0 () {
        return () -> MathUtil.applyDeadband(-driver.getLeftY(), 0.2); 
       
    }

    public static DoubleSupplier getLeftXC0 () {
        return () -> MathUtil.applyDeadband(-driver.getLeftX(), 0.2); 
    }

    public static DoubleSupplier getRightXC0 () {
        return () -> MathUtil.applyDeadband(-driver.getRightX(), 0.2); 
    }

    public static double getLeftY_ope (){
        return MathUtil.applyDeadband(-operator.getLeftY(), 0.2); 
    }
    public static double getRightY_ope (){
        return MathUtil.applyDeadband(-operator.getRightY(), 0.2); 
    }
    public static boolean buttonA (){
        return operator.getAButtonPressed(); 
    }
    public static boolean buttonB(){
        return operator.getBButtonPressed(); 
    }
    public static boolean buttonx (){
        return operator.getXButtonPressed(); 
    }
    public static boolean buttony(){
        return operator.getYButtonPressed(); 
    }
    public static Boolean button5(){
        return operator.getRightBumperButtonPressed();
    }
    public static Boolean ButtonCOLLECT(){
        return operator.getLeftBumperButtonReleased();
    }
    public static Boolean ButtonCORAL(){
        return operator.getBackButtonPressed();
    }
    public static Boolean ButtonALGAE(){
        return operator.getStartButtonPressed();
    }
    public static Double ButtonMuL(){
        return operator.getLeftTriggerAxis();
    }
    public static Double ButtonMuR(){
        return -operator.getRightTriggerAxis();
    }
    
}
   
