package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

// class to represent a button on a gampad
// tracks how long the button has been held down
public class Button {
    private final ElapsedTime time; // timer
    private boolean down;           // private field keeping track of whether the button is down or not
    public Button() {               // constructor: initializes timer and "down" field
        time = new ElapsedTime();
        down = false;               // defaults to false (not pressed)
    }
    public void down() {    // button is pressed
        if(!down) {         // if the button was just pressed, start the timer
            down = true;
            time.reset();
        }
    }
    public double getTimeDown() {   // returns the time in the milliseconds since the button was pressed
        return time.milliseconds();
    }
    public void up() {  // button is released
        down = false;
    }
}
