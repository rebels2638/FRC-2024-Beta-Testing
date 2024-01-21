package frc.robot.commands.audio;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.audio.AudioPlayer;

public class playMusic extends Command {
    private AudioPlayer audioplayerSubsystem;
    private int toggle;
    private int maxBound;

    public playMusic(Audio audioplayerSubsystem) {
        this.audioSubsystem = audioplayerSubsystem;
        this.toggle = 0;
        this.maxBound = 2; // exclusive, also maxBound is silent
    }
    
    @Override
    public void execute() {
        if (this.toggle == this.maxBound) {
            this.audioplayerSubsystem.stop()
            this.toggle = 0;
        else {
            this.audioplayerSubsystem.play(this.toggle);
            this.toggle++;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
