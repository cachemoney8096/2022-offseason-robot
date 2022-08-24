package net.cachemoney8096.frc2022o.util;

import java.util.function.Predicate;

public class LatchedConditional<T> {
  private boolean internalState;
  private Predicate<T> activeCondition;
  private Predicate<T> inactiveCondition;

  public LatchedConditional(Predicate<T> activeCondition, Predicate<T> inactiveCondition) {
    this.activeCondition = activeCondition;
    this.inactiveCondition = inactiveCondition;
  }

  public boolean calculate(T input) {
    if (internalState && inactiveCondition.test(input)) {
      internalState = false;
    } else if (!internalState && activeCondition.test(input)) {
      internalState = true;
    }

    return internalState;
  }

  public void reset(boolean state) {
    internalState = state;
  }

  public boolean getLast() {
    return internalState;
  }
}
