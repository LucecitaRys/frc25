package frc.robot;

import java.util.Optional;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.IAuto;
import frc.robot.commands.ModeAlgae;
import frc.robot.commands.ModeCorall;
import frc.robot.subsystems.Brazo;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevatorSub;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Brazo.brazoposes;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.subsystems.ElevatorSub.ElePoses;
import frc.robot.subsystems.Shooter.intake_states;


public class Robot extends TimedRobot {
  private Telemetry mTelemetry;
  private Drive mDrive;
private Shooter mShooter;
private Brazo mBrazo;
private ElevatorSub mElevatorSub;
  private Vision mVision;
  private Optional<IAuto> mAutoMode = Optional.empty();
  private Command mAutonomousCommand;
 
  

  @Override
  public void robotInit() {
    mTelemetry = new Telemetry();
mVision = Vision.getInstance();
mDrive = Drive.getInstance();
mShooter = Shooter.getInstance();
mBrazo = Brazo.getInstance();
mElevatorSub = ElevatorSub.getInstance();

   
   Command groupCommand = new SequentialCommandGroup(
        new ModeCorall(), // CoralCommand
        new ModeAlgae()   // AlgaeCommand
    );

    
    mShooter.setDefaultCommand(groupCommand);
    mElevatorSub.setDefaultCommand(groupCommand);
    mBrazo.setDefaultCommand(groupCommand);




    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
   mVision.salida();
    // mShooter.setShooterControlState(ShooterControlState.None);
    // mClimber.setControlState(ClimberControlState.None);
  }

  @Override
  public void disabledPeriodic() {
    mTelemetry.updateAutoModeCreator();
    mAutoMode = mTelemetry.getAutoModeSelected();
  }

  @Override
  public void autonomousInit() {
    if (mAutoMode.isPresent()) {
      mDrive.setKinematicsLimits(Constants.Drive.uncappedLimits);
      mDrive.resetOdometry(mAutoMode.get().getStartingPose());
      mAutonomousCommand = mAutoMode.get().getAutoCommand();
      mAutonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (mAutonomousCommand != null) {
      mAutonomousCommand.cancel();
    }
    mDrive.setKinematicsLimits(Constants.Drive.oneMPSLimits);
    mDrive.setDriveControlState(DriveControlState.TeleopControl);
    // mIntake.setControlState(IntakeControlState.VariableVelocity);
    // mShooter.setShooterControlState(ShooterControlState.VariableVelocity);
    // mClimber.setControlState(ClimberControlState.PercentOutput);
  }

  @Override
  public void teleopPeriodic() {
    if (ControlBoard.driver.getXButtonPressed()) {
      mDrive.setYawAngle(0); 
    }
    if (ControlBoard.driver.getPOV() != -1) { 
      mDrive.setHeadingControl(Rotation2d.fromDegrees(ControlBoard.driver.getPOV())); 
    } 

   

    
  }
  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
  }
