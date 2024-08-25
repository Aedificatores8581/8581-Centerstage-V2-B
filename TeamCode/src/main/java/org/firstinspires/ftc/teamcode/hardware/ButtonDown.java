package org.firstinspires.ftc.teamcode.hardware;

public class ButtonDown {
    boolean pressed = false;
    ButtonListener listener;
    public ButtonDown(ButtonListener listener) {
        this.listener = listener;
    }
    public void runOnPressed() {listener.onButtonPressed();}
    public void update(boolean button) {
        if (button) {
            if (!pressed) {
                runOnPressed();
            }
            pressed = true;
        } else {
            pressed = false;
        }
    }
    public interface ButtonListener {void onButtonPressed();}
}
