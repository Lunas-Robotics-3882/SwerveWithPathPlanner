package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootCommandLewikka extends Command{

    private Timer timer = new Timer();

     private IndexerSubsystem indexer = new IndexerSubsystem();
     private ShooterSubsystem shooter = new ShooterSubsystem();
     private FeederSubsystem feeder = new FeederSubsystem();
    
    private boolean firstcheck = true;

    public ShootCommandLewikka(ShooterSubsystem shooter,IndexerSubsystem indexer, FeederSubsystem feeder) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.feeder = feeder;
        addRequirements(shooter,feeder,indexer);

      }

      @Override
  public void initialize() {
    timer.restart();
    timer.start();
    shooter.NeutralShot();
  }

  @Override
  public void execute()
  {
    if (timer.get() > 0.2)
    {
      indexer.index();
      feeder.feeder();
    }
  }

      @Override
      public boolean isFinished() {
        return false;
      }

     @Override
     public void end(boolean interrupted) {
      shooter.disable();
      indexer.disable();
      feeder.disable();
    }
}