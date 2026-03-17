package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootCommandAuto extends Command{

    private Timer timer = new Timer();

     private IndexerSubsystem indexer = new IndexerSubsystem();
     private ShooterSubsystem shooter = new ShooterSubsystem();
     private FeederSubsystem feeder = new FeederSubsystem();
    
    private boolean firstcheck = true;

    public ShootCommandAuto(ShooterSubsystem shooter,IndexerSubsystem indexer, FeederSubsystem feeder) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.feeder = feeder;
        addRequirements(shooter,feeder,indexer);

      }

      @Override
  public void initialize() {
    timer.restart();
    timer.start();
    shooter.shooter();
  }

  @Override
  public void execute()
  {
    if (timer.get() > 0.33)
    {
      indexer.index();
      feeder.feeder();
    }
  }

      @Override
      public boolean isFinished() {

        if (timer.get() > 2)
        {
          return true;
        }

        return false;
      }

     @Override
     public void end(boolean interrupted) {
      shooter.disable();
      indexer.disable();
      feeder.disable();
    }
}