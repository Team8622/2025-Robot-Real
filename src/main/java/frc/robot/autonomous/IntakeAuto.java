package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral_Intake;

public class IntakeAuto extends Command{
    Coral_Intake m_intake;
    double setSpeed;
    double setTime;
    private final Timer timer = new Timer();

    public IntakeAuto(Coral_Intake intake, double speed, double time){
        m_intake = intake;
        setSpeed = speed;
        setTime = time;
        addRequirements(m_intake);
    }   
    
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        m_intake.start(setSpeed);
    }
    @Override
    public boolean isFinished() {
        return timer.get() >= setTime; // Stop after 1 second
    }
    @Override
    public void end(boolean interrupted){
        m_intake.stop();
    }
}
