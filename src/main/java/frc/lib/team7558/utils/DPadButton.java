package frc.lib.team7558.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

/** DPadButton class to make use of the four cardinal directions on the DPad */
public class DPadButton extends Button {

  XboxController m_joystick;
  Direction m_direction;

  /**
   * New DPadButton
   *
   * @param joystick - The joystick to watch
   * @param direction - The {@link DPadButton.Direction} to watch for
   */
  public DPadButton(XboxController joystick, Direction direction) {
    this.m_joystick = joystick;
    this.m_direction = direction;
  }

  /**
   * The Direction on the DPad
   *
   * @param UP
   * @param RIGHT
   * @param DOWN
   * @param LEFT
   */
  public static enum Direction {
    UP(0),
    RIGHT(90),
    DOWN(180),
    LEFT(270);

    int direction;

    private Direction(int direction) {
      this.direction = direction;
    }
  }

  /**
   * @return If the DPadButton is being activated
   */
  public boolean get() {
    int dPadValue = m_joystick.getPOV();
    return (dPadValue == m_direction.direction)
        || (dPadValue == (m_direction.direction + 45) % 360)
        || (dPadValue == (m_direction.direction + 315) % 360);
  }
}
