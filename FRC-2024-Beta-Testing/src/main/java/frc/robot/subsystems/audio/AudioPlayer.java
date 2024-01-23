package frc.robot.subsystems.audio;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AudioPlayer extends SubsystemBase {
    
    private Orchestra orchestra;
    List<TalonFX> motors;

    public AudioPlayer() {
        List<ParentDevice> motors = new ArrayList<>(
            Arrays.asList(new TalonFX(0, "rio"), new TalonFX(1, "rio"), new TalonFX(2, "rio"), new TalonFX(3, "rio"), 
                new TalonFX(4, "rio"), new TalonFX(5, "rio"), new TalonFX(6, "rio"), new TalonFX(7, "rio")));

        orchestra = new Orchestra(motors);
    }

    public void play(int songID) {
        orchestra.loadMusic("src/main/java/frc/robot/subsystems/audio/tracks/"+Integer.toString(songID)+".chrp");
    }

    public void stop() {
        orchestra.stop();
    }

}
