package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@SuppressWarnings("MethodName")
public class CommandKeypad extends CommandGenericHID {
  private final GenericHID m_hid;

  public CommandKeypad(int port) {
    super(port);
    m_hid = new GenericHID(port);
  }

  @Override
  public GenericHID getHID() {
    return m_hid;
  }

  // Knob
  public Trigger knob_press() {
    return knob_press(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger knob_press(EventLoop loop) {
    return button(1, loop);
  }

  public Trigger knob_left() {
    return knob_left(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger knob_left(EventLoop loop) {
    return button(28, loop);
  }

  public Trigger knob_right() {
    return knob_right(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger knob_right(EventLoop loop) {
    return button(29, loop);
  }

  // Symbol row
  public Trigger circle() {
    return circle(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger circle(EventLoop loop) {
    return button(2, loop);
  }

  public Trigger triangle() {
    return triangle(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger triangle(EventLoop loop) {
    return button(3, loop);
  }

  public Trigger square() {
    return square(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger square(EventLoop loop) {
    return button(4, loop);
  }

  public Trigger x() {
    return x(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger x(EventLoop loop) {
    return button(5, loop);
  }

  // Macro keys
  public Trigger m1() {
    return m1(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger m1(EventLoop loop) {
    return button(6, loop);
  }

  public Trigger m2() {
    return m2(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger m2(EventLoop loop) {
    return button(11, loop);
  }

  public Trigger m3() {
    return m3(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger m3(EventLoop loop) {
    return button(16, loop);
  }

  public Trigger m4() {
    return m4(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger m4(EventLoop loop) {
    return button(20, loop);
  }

  public Trigger m5() {
    return m5(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger m5(EventLoop loop) {
    return button(24, loop);
  }

  // Numpad function keys
  public Trigger numClr() {
    return numClr(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger numClr(EventLoop loop) {
    return button(7, loop);
  }

  public Trigger forwardSlash() {
    return forwardSlash(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger forwardSlash(EventLoop loop) {
    return button(8, loop);
  }

  public Trigger asterisk() {
    return asterisk(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger asterisk(EventLoop loop) {
    return button(9, loop);
  }

  public Trigger minus() {
    return minus(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger minus(EventLoop loop) {
    return button(10, loop);
  }

  public Trigger plus() {
    return plus(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger plus(EventLoop loop) {
    return button(15, loop);
  }

  public Trigger enter() {
    return enter(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger enter(EventLoop loop) {
    return button(27, loop);
  }

  // Numpad digits
  public Trigger num0() {
    return num0(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger num0(EventLoop loop) {
    return button(25, loop);
  }

  public Trigger num1() {
    return num1(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger num1(EventLoop loop) {
    return button(21, loop);
  }

  public Trigger num2() {
    return num2(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger num2(EventLoop loop) {
    return button(22, loop);
  }

  public Trigger num3() {
    return num3(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger num3(EventLoop loop) {
    return button(23, loop);
  }

  public Trigger num4() {
    return num4(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger num4(EventLoop loop) {
    return button(17, loop);
  }

  public Trigger num5() {
    return num5(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger num5(EventLoop loop) {
    return button(18, loop);
  }

  public Trigger num6() {
    return num6(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger num6(EventLoop loop) {
    return button(19, loop);
  }

  public Trigger num7() {
    return num7(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger num7(EventLoop loop) {
    return button(12, loop);
  }

  public Trigger num8() {
    return num8(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger num8(EventLoop loop) {
    return button(13, loop);
  }

  public Trigger num9() {
    return num9(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger num9(EventLoop loop) {
    return button(14, loop);
  }

  public Trigger decimal() {
    return decimal(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  public Trigger decimal(EventLoop loop) {
    return button(26, loop);
  }
}
