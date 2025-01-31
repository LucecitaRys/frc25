package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooAuto extends Command {
    private final ShooterSubsystem mShooterSubsystem = ShooterSubsystem.getInstance(); 
    private double startTime;

    public ShooAuto() {
        addRequirements(mShooterSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double elapsedTime = Timer.getFPGATimestamp() - startTime;

        if (elapsedTime >= 0.1 && elapsedTime <= 0.3) {
            mShooterSubsystem.setDutyCycleOutput(1);
        } else if (elapsedTime > 0.3) {
            mShooterSubsystem.setDutyCycleOutput(1); 
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