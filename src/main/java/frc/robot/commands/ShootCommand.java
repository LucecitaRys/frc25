package frc.robot.commands;



import frc.robot.ControlBoard;
import frc.robot.subsystems.ShooterSubsystem;

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
     //  mShooterSubsystem.setShooterSpeed(ControlBoard.getLeftY_0(), ControlBoard.getRightY_0());
     //mShooterSubsystem.setDutyCycleOutput(ControlBoard.getLeftY_0());
     


        if (ControlBoard.buttonA()) {// Assuming button 1 is used to set forward direction
        mShooterSubsystem.setDutyCycleOutput(1.0); // Full forward
    } else if (ControlBoard.buttonB()) { // Assuming button 2 is used to set reverse direction
        mShooterSubsystem.setDutyCycleOutput(-1.0); // Full reverse
    } else {
        mShooterSubsystem.setDutyCycleOutput(0); // Stop motor if no buttons are pressed
    }

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