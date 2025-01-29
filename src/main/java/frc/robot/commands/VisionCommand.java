 package frc.robot.commands;


import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;


public class VisionCommand extends Command {
     private final Vision mVision = Vision.getInstance(); 
     public VisionCommand() {
        addRequirements(mVision);
        
        }
    }
    
    
