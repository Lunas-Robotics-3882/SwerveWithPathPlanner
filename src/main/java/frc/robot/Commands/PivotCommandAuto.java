package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class PivotCommandAuto extends Command{

    private Timer timer = new Timer();

    private PivotSubsystem pivot = new PivotSubsystem();
    
    private boolean firstcheck = true;

    public PivotCommandAuto(PivotSubsystem pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
      }

      @Override
  public void initialize() {
   
    pivot.setDownPosition();
  }

  @Override
  public void execute()
  {}

      @Override
      public boolean isFinished() {
        return pivot.CheckDownPost();
      }

     @Override
     public void end(boolean interrupted) {
    }
}