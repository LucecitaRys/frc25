package frc.robot.commands;

import frc.robot.Controllers;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class ShootCommand extends Command {
    private final ShooterSubsystem mShooterSubsystem = ShooterSubsystem.getInstance(); 
  public ShootCommand() {
    addRequirements(mShooterSubsystem);
    
    }

    @Override
    public void initialize() {
       
    }

    @Override
    public void execute() {
     
      // mShooterSubsystem.setShooterSpeed(-Controllers.getRT(), -Controllers.getLT());
       mShooterSubsystem.setShooterSpeed(Controllers.getLeftY_0(), Controllers.getRightY_0());
       

    }
    @Override
    public void end(boolean interrupted) {
        mShooterSubsystem.setShooterSpeed(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false; // Este comando es continuo, hasta que se interrumpa
    }
}