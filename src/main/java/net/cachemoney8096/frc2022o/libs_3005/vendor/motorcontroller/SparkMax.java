package net.cachemoney8096.frc2022o.libs_3005.vendor.motorcontroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import net.cachemoney8096.frc2022o.libs_3005.controller.Controller;
import net.cachemoney8096.frc2022o.libs_3005.controller.PIDGains;
import net.cachemoney8096.frc2022o.libs_3005.electromechanical.Encoder;
import net.cachemoney8096.frc2022o.libs_3005.electromechanical.EncoderSupplier;
import net.cachemoney8096.frc2022o.libs_3005.monitor.HealthMonitor;
import net.cachemoney8096.frc2022o.libs_3005.util.AccessOrderHashSet;
import net.cachemoney8096.frc2022o.libs_3005.util.Constraints;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import org.tinylog.Logger;

public class SparkMax extends CANSparkMax {

  private AccessOrderHashSet<BiFunction<CANSparkMax, Boolean, Boolean>> m_mutatorChain;
  private List<SparkMax> m_followers = new ArrayList<>();
  private final int kParameterSetAttemptCount = 5;

  /**
   * Store a reference to every spark max.
   *
   * <p>TODO: Release the reference if it is closed.
   */
  private static List<SparkMax> m_sparkMaxes = new ArrayList<>();

  private static int m_burnFlashCnt = 0;

  private static SparkMaxMonitor s_monitor = new SparkMaxMonitor();

  private static final int VELOCITY_GAIN_SLOT = 0;
  private static final int POSITION_GAIN_SLOT = 1;
  private static final int SMART_MOTION_GAIN_SLOT = 2;

  /**
   * Monitor the Spark Max to check for reset. This is used by the health monitor to automatically
   * re-initialize the spark max in case of reboot.
   *
   * @param sparkMax Spark Max object to monitor
   * @return True if the device has reset
   */
  private static boolean sparkmaxMonitorFunction(CANSparkMax sparkMax) {
    return sparkMax.getStickyFault(FaultID.kHasReset);
  }

  /**
   * Reinitialize the SparkMax by running through all mutations on the object in order.
   *
   * @return true if reinitialized correctly
   */
  private boolean reinitFunction() {
    for (var fcn : m_mutatorChain) {
      if (!fcn.apply(this, false)) {
        return false;
      }
    }
    return clearFaults() == REVLibError.kOk;
  }

  /**
   * Create a motor controller from a CANSparkMax object.
   *
   * @param sparkMax A CANSparkMax object. Run any initialization that you want excluded from the
   *     built in health monitor functions
   * @param initFunction A function which takes a CANSparkMax and returns a Boolean. This function
   *     is used to initialize the CANSparkMax device, and is called in one of two places. 1) It is
   *     called in this constructor, and 2) it is called in the case of a health monitor timeout
   *     (i.e. the controller has reset)
   */
  public SparkMax(int canId, MotorType motorType) {
    super(canId, motorType);

    // Always start fresh and apply settings in code for each device
    // Add delay to avoid any possible timing issues.
    restoreFactoryDefaults();
    Timer.delay(0.050);

    // If a parameter set fails, this will add more time to alleviate any bus traffic
    // default is 20ms
    setCANTimeout(50);

    m_mutatorChain = new AccessOrderHashSet<>();
    HealthMonitor.monitor(() -> sparkmaxMonitorFunction(this), () -> reinitFunction());
    m_sparkMaxes.add(this);
    Logger.tag("SparkMax").debug("Initializing SparkMax with Id {}", canId);
    if (m_burnFlashCnt > 0) {
      Logger.tag("SparkMax")
          .warn(
              "SparkMax with ID {} initialized after burning flash, flash count {}",
              canId,
              m_burnFlashCnt);
    }
    s_monitor.add(this);
  }

  /**
   * Create a SPARK MAX motor controller with a brushless motor
   *
   * @param canId Spark Max CAN Id
   */
  public SparkMax(int canId) {
    this(canId, MotorType.kBrushless);
  }

  /**
   * Create spark max object with an initializer method. This method is called on initialization as
   * well as if the spark max resets.
   *
   * @param initialize function that returns true on success, that takes a CANSparkMax and a Boolean
   *     to indicate if the call is from initialization or after a reset.
   * @return this
   */
  public SparkMax withInitializer(BiFunction<CANSparkMax, Boolean, Boolean> initialize) {
    m_mutatorChain.add(initialize);

    int setAttemptNumber = 0;
    while (initialize.apply(this, true) != true) {
      Logger.tag("Spark Max")
          .warn(
              "Spark Max ID {}: Failed to initialize, attempt {} of {}",
              getDeviceId(),
              setAttemptNumber,
              kParameterSetAttemptCount);
      setAttemptNumber++;

      if (setAttemptNumber >= kParameterSetAttemptCount) {
        Logger.tag("Spark Max").error("Spark Max ID {}: Failed to initialize!!", getDeviceId());
        break;
      }
    }

    return this;
  }

  /**
   * Create a Controller based on the internal spark max velocity controller
   *
   * @param gains PID gains, note that units will depend on how the device is configured so be sure
   *     to check this!!
   * @return
   */
  public SparkMaxController velocityController(PIDGains gains) {
    return new SparkMaxController(
        this, getPIDController(), ControlType.kVelocity, gains, -1.0, 1.0);
  }

  /**
   * Create a Controller based on the internal spark max velocity controller
   *
   * @param gains PID gains, note that units will depend on how the device is configured so be sure
   *     to check this!!
   * @param minOutput the minimum output for the controller in percent [-1, 1) must be lower than
   *     max
   * @param maxOutput the max output of the controller in percent (-1, 1] must be higher than min
   * @return
   */
  public SparkMaxController velocityController(PIDGains gains, double minOutput, double maxOutput) {
    return new SparkMaxController(
        this, getPIDController(), ControlType.kVelocity, gains, minOutput, maxOutput);
  }

  /**
   * Create a Controller based on the internal spark max position controller
   *
   * @param gains PID gains, note that units will depend on how the device is configured so be sure
   *     to check this!!
   * @return
   */
  public Controller positionController(PIDGains gains) {
    return new SparkMaxController(
        this, getPIDController(), ControlType.kPosition, gains, -1.0, 1.0);
  }

  /**
   * Create an encoder object based on the Spark Max internal encoder
   *
   * @return Encoder object
   */
  public Encoder builtinEncoder() {
    return new EncoderSupplier(
        () -> getEncoder().getVelocity(),
        () -> getEncoder().getPosition(),
        pos -> getEncoder().setPosition(pos));
  }

  /**
   * Create a profiled PID controller based on the spark max smart motion
   *
   * @param gains
   * @param constraints
   * @return
   */
  public Controller profiledController(PIDGains gains, Constraints constraints) {
    mutate(
        (sparkMax, firstCall) -> {
          int errors = 0;
          errors +=
              SparkMaxUtils.check(
                  sparkMax
                      .getPIDController()
                      .setSmartMotionMaxAccel(constraints.maxAcceleration, SMART_MOTION_GAIN_SLOT));
          errors +=
              SparkMaxUtils.check(
                  sparkMax
                      .getPIDController()
                      .setSmartMotionMaxVelocity(constraints.maxVelocity, SMART_MOTION_GAIN_SLOT));
          return errors == 0;
        });
    return new SparkMaxController(
        this, getPIDController(), ControlType.kPosition, gains, -1.0, 1.0);
  }

  /**
   * Modify CANSparkMax object. Mutations using this method are re-run in order in the case of a
   * device failure that is later recovered. The same function can be called multiple times, and
   * will simply be moved to the end of the list each call.
   *
   * <p>Only adds the function to the list if it succeeds
   *
   * @param fcn a function on the underlying CANSparkMax object returning true on success. Typically
   *     used to change parameter values. Function should run quickly and return.
   * @return result of mutate function
   */
  public boolean mutate(BiFunction<CANSparkMax, Boolean, Boolean> fcn) {
    Boolean result = fcn.apply(this, true);

    int setAttemptNumber = 0;
    while (result == null || result != true) {
      Logger.tag("Spark Max")
          .warn(
              "Spark Max ID {}: Failed to run mutator, attempt {} of {}",
              getDeviceId(),
              setAttemptNumber,
              kParameterSetAttemptCount);
      setAttemptNumber++;

      if (setAttemptNumber >= kParameterSetAttemptCount) {
        Logger.tag("Spark Max")
            .error("Spark Max ID {}: Failed to run mutator function!!", getDeviceId());
        break;
      }
      result = fcn.apply(this, true);
    }

    if (result != null && result) {
      m_mutatorChain.add(fcn);
    }
    return result == null ? false : result;
  }

  public enum FrameStrategy {
    kDefault,
    kVelocity,
    kPosition,
    kNoFeedback,
    kCurrent,
    kVoltage,
    kVelocityAndPosition,
  }

  /**
   * Set a frame strategy for the feedback frames. This changes the periodic frame rates to be
   * either slow (500ms) or fast (15ms) depending on the signal needed.
   *
   * @param sparkMax CANSparkMax object to run this on
   * @param strategy Signal strategy
   * @param withFollower does this device have a follower motor
   */
  public static void setFrameStrategy(
      CANSparkMax sparkMax, FrameStrategy strategy, boolean withFollower) {
    final int slowFrame = 500;
    final int fastFrame = 15;
    int status0 = 10;
    int status1 = 20;
    int status2 = 20;
    int status3 = 50;
    switch (strategy) {
      case kVelocity:
      case kCurrent:
      case kVoltage:
        status1 = fastFrame;
        status2 = slowFrame;
        status3 = slowFrame;
        break;
      case kPosition:
        status1 = slowFrame;
        status2 = fastFrame;
        status3 = slowFrame;
        break;
      case kNoFeedback:
        status1 = slowFrame;
        status2 = slowFrame;
        status3 = slowFrame;
        break;
      case kVelocityAndPosition:
        status1 = fastFrame;
        status2 = fastFrame;
        status3 = slowFrame;
        break;
      case kDefault:
      default:
        // already set
        break;
    }
    if (!withFollower) {
      status0 = slowFrame;
    }
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, status0);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, status1);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, status2);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, status3);
  }

  /**
   * Set a frame strategy for the feedback frames. This changes the periodic frame rates to be
   * either slow (500ms) or fast (15ms) depending on the signal needed.
   *
   * @param sparkMax CANSparkMax object to run this on
   * @param strategy Signal strategy
   */
  public static void setFrameStrategy(CANSparkMax sparkMax, FrameStrategy strategy) {
    setFrameStrategy(sparkMax, strategy, false);
  }

  public class SparkMaxController implements Controller {
    // Cache gains so 'get' commands don't go out on CAN
    // 4 gain slots in spark max
    private final SparkMax m_sparkMax;
    private PIDGains m_gainsCached = new PIDGains();
    private SparkMaxPIDController m_sparkMaxController;
    private Constraints m_constraintsCached = new Constraints(0.0, 0.0);
    private int m_slot;
    private ControlType m_controlType;

    private double m_minimumInput = 0.0;
    private double m_maximumInput = 0.0;
    private boolean m_continuous = false;

    protected SparkMaxController(
        SparkMax sparkMax,
        SparkMaxPIDController pid,
        ControlType type,
        PIDGains gains,
        double minOutput,
        double maxOutput) {
      m_sparkMax = sparkMax;
      m_sparkMaxController = pid;
      m_controlType = type;
      if (type == ControlType.kPosition) {
        m_slot = POSITION_GAIN_SLOT;
      } else if (type == ControlType.kVelocity) {
        m_slot = VELOCITY_GAIN_SLOT;
      } else if (type == ControlType.kSmartMotion) {
        m_slot = SMART_MOTION_GAIN_SLOT;
      }
      m_gainsCached = gains;

      // Add this to mutate list so it is reset if controller resets
      sparkMax.mutate(
          (cansparkmax, firstCall) -> {
            int errors = 0;
            errors += SparkMaxUtils.check(m_sparkMaxController.setP(gains.P, m_slot));
            errors += SparkMaxUtils.check(m_sparkMaxController.setI(gains.I, m_slot));
            errors += SparkMaxUtils.check(m_sparkMaxController.setD(gains.D, m_slot));
            errors +=
                SparkMaxUtils.check(
                    m_sparkMaxController.setOutputRange(minOutput, maxOutput, m_slot));
            return errors == 0;
          });

      if (m_burnFlashCnt > 0) {
        Logger.tag("SparkMax Controller")
            .warn(
                "Creating a SparkMax Controller after burning flash, settings will not be burned. CanID {}, FlashCnt {}",
                sparkMax.getDeviceId(),
                m_burnFlashCnt);
      }
    }

    /**
     * Initialize a sendable for tuning the velocity controller PID constants at run time
     *
     * @param builder Builder passed in during initSendable
     */
    private void controllerInitSendable(SendableBuilder builder, int slot) {
      builder.setSmartDashboardType("PIDController");
      builder.setActuator(true);
      builder.addDoubleProperty(
          "p",
          () -> m_gainsCached.P,
          (val) -> {
            m_gainsCached.P = val;
            m_sparkMaxController.setP(val, slot);
          });
      builder.addDoubleProperty(
          "i",
          () -> m_gainsCached.I,
          (val) -> {
            m_gainsCached.I = val;
            m_sparkMaxController.setI(val, slot);
          });
      builder.addDoubleProperty(
          "d",
          () -> m_gainsCached.D,
          (val) -> {
            m_gainsCached.D = val;
            m_sparkMaxController.setD(val, slot);
          });

      if (slot != SMART_MOTION_GAIN_SLOT) {
        return;
      }

      builder.addDoubleProperty(
          "max accel",
          () -> m_constraintsCached.maxAcceleration,
          (val) -> {
            m_constraintsCached.maxAcceleration = val;
            m_sparkMaxController.setSmartMotionMaxAccel(val, SMART_MOTION_GAIN_SLOT);
          });

      builder.addDoubleProperty(
          "max velocity",
          () -> m_constraintsCached.maxVelocity,
          (val) -> {
            m_constraintsCached.maxVelocity = val;
            m_sparkMaxController.setSmartMotionMaxVelocity(val, SMART_MOTION_GAIN_SLOT);
          });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      controllerInitSendable(builder, m_slot);
    }

    @Override
    public void setReference(double reference, double processVariable, double feedforward) {
      if (m_continuous) {
        /*
         * Spark Max can't automatically wrap smartly, so we must 'unwrap'
         * the values manually to set the correct target.
         */
        double angleMod = MathUtil.inputModulus(processVariable, m_minimumInput, m_maximumInput);
        reference = reference + processVariable - angleMod;
      }

      m_sparkMaxController.setReference(reference, m_controlType, m_slot, feedforward);
    }

    @Override
    public void enableContinuousInput(double minimumInput, double maximumInput) {
      // Implementation requires this currently
      assert minimumInput <= 0.0;
      assert maximumInput >= 0.0;

      m_continuous = true;
      m_minimumInput = minimumInput;
      m_maximumInput = maximumInput;
    }

    @Override
    public void disableContinuousInput() {
      m_continuous = false;
    }

    @Override
    public boolean isContinuousInputEnabled() {
      return m_continuous;
    }
  }

  public SparkMax withFollower(SparkMax follower) {
    return withFollower(follower, false);
  }

  public SparkMax withFollower(SparkMax follower, boolean invert) {
    follower.mutate(
        (sparkMax, firstCall) -> {
          return sparkMax.follow(this, invert) == REVLibError.kOk;
        });
    m_followers.add(follower);
    return this;
  }

  /**
   * Run burnFlash() for all controllers initialized. The ideal use case for this call is to call it
   * once everything has been initialized. The burnFlash() call has the side effect of preventing
   * all communication *to* the device for up to 200ms or more, potentially including some messages
   * called before the burnFlash() call, and receiveing messages *from* the device.
   *
   * <p>WARNING: This call will sleep the thread before and after burning flash. This is for your
   * safety.
   */
  public static void burnFlashInSync() {
    Logger.tag("SparkMax").debug("Burning Flash Count: {}", ++m_burnFlashCnt);
    Timer.delay(0.25);
    for (SparkMax max : m_sparkMaxes) {
      Logger.tag("SparkMax").trace("Burning flash for Can ID {}", max.getDeviceId());
      max.burnFlash();
      // Enough time to not spam the bus too bad
      Timer.delay(0.005);
    }
    Timer.delay(0.25);
    Logger.tag("SparkMax").debug("Burn Flash Complete.");
  }
}
