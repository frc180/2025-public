package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

    public final LEDColor RED = new LEDColor(50, 0, 0, 255);
    public final LEDColor BLUE = new LEDColor(21, 46, 150, 255); // navy blue
    public final LEDColor GREEN = new LEDColor(0, 255, 0, 255);
    public final LEDColor YELLOW = new LEDColor(255, 255, 0, 255);
    public final LEDColor WHITE = new LEDColor(255, 255, 255, 255);
    public final LEDColor ALGAE = new LEDColor(0, 255, 30, 255);
    public final LEDColor PURPLE = new LEDColor(163, 49, 196, 255);

    public final RainbowAnimation rainbow;
    public final SingleFadeAnimation blueFade, redFade, yellowFade, whiteFade, yellowFadeFast, purpleFade;
    public final TwinkleAnimation blueTwinkle, redTwinkle;
    public final ColorFlowAnimation blueFlow, redFlow, yellowFlow;
    public final ColorFlowAnimation blueLeftFlow, blueRightFlow;
    public final LarsonAnimation yellowLarson;
    public final StrobeAnimation greenStrobe;

    private final int STRIP_LENGTH = 49;
    private final int STRIP_2_LENGTH = 47;
    private final int NUM_LEDS = 8 + STRIP_LENGTH + STRIP_2_LENGTH;
    private final int STRIP_OFFSET = 0;
    private final int NO_CANDLE_OFFSET = 8;

    private final CANdle candle;

    private Animation currentAnimation = null;
    private Animation currentAnimation2 = null;
    private LEDColor currentColor = null;
    private LEDColor currentSplitColor = null;

    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.disableWhenLOS = false;
        config.statusLedOffWhenActive = true;
        config.brightnessScalar = 0.2;
        config.v5Enabled = true;
        config.stripType = LEDStripType.GRB;
        config.vBatOutputMode = VBatOutputMode.Off;
        candle = new CANdle(Constants.CANDLE, Constants.CANIVORE);
        candle.configAllSettings(config);

        rainbow = new RainbowAnimation(1, 1, NUM_LEDS, false, STRIP_OFFSET);

        blueFade = fade(BLUE, 0.5);
        redFade = fade(RED, 0.5);
        yellowFade = fade(YELLOW, 0.5);
        purpleFade = fade(PURPLE, 1);
        whiteFade = fade(WHITE, 1);
        yellowFadeFast = fade(YELLOW, 1);

        blueTwinkle = twinkle(BLUE, 0.25, TwinklePercent.Percent64);
        redTwinkle = twinkle(RED, 0.25, TwinklePercent.Percent64);

        blueFlow = colorFlow(BLUE, 0.7, Direction.Forward);
        redFlow = colorFlow(RED, 0.7, Direction.Forward);
        yellowFlow = colorFlow(YELLOW, 0.5, Direction.Forward);
        yellowLarson = larson(YELLOW, 0.5, 5, BounceMode.Front);

        blueLeftFlow = colorFlow(BLUE, 0.7, Direction.Forward, STRIP_LENGTH, NO_CANDLE_OFFSET);
        blueRightFlow = colorFlow(BLUE, 0.7, Direction.Backward, STRIP_2_LENGTH, NO_CANDLE_OFFSET + STRIP_LENGTH);

        greenStrobe = strobe(GREEN, 0.25);
    }

    public Command animate(Animation animation) {
        return run(() -> setAnimation(animation));
    }

    public Command color(LEDColor color) {
        return run(() -> setColor(color));
    }

    public void setAnimation(Animation animation) {
        setAnimation(animation, 0);
    }

    public void setAnimation(Animation animation, int slot) {
        if (animation != currentAnimation) {
            // clearAnimations();
            ErrorCode code = candle.animate(animation, slot);
            if (code == ErrorCode.OK) {
                clearState();
                currentAnimation = animation;
            }
        }
    }

    public void setDualAnimation(Animation animation1, Animation animation2) {
        if (currentAnimation != animation1 || currentAnimation2 != animation2) {
            clearAnimations();
            ErrorCode code1 = candle.animate(animation1, 0);
            ErrorCode code2 = candle.animate(animation2, 1);

            if (code1 == ErrorCode.OK && code2 == ErrorCode.OK) {
                clearState();
                currentAnimation = animation1;
                currentAnimation2 = animation2;
            }
        }
    }

    public void setColor(LEDColor color) {
        if (color != currentColor) {
            clearAnimations();
            ErrorCode code = candle.setLEDs(color.r, color.g, color.b, color.w, STRIP_OFFSET, NUM_LEDS);
            if (code == ErrorCode.OK) {
                clearState();
                currentColor = color;
            }
        }
    }

    public void setSplitColor(LEDColor top, LEDColor bottom) {
        if (top != currentColor || bottom != currentSplitColor) {
            clearAnimations();
            ErrorCode code1 = candle.setLEDs(bottom.r, bottom.g, bottom.b, bottom.w, 0, 24);
            ErrorCode code2 = candle.setLEDs(top.r, top.g, top.b, top.w, 24, 25);

            ErrorCode code3 =  candle.setLEDs(top.r, top.g, top.b, top.w, 49, 23);
            ErrorCode code4 =  candle.setLEDs(bottom.r, bottom.g, bottom.b, bottom.w, 72, 24);

            if (code1 == ErrorCode.OK && code2 == ErrorCode.OK && code3 == ErrorCode.OK && code4 == ErrorCode.OK) {
                clearState();
                currentColor = top;
                currentSplitColor = bottom;
            }
        }
    }

    private void clearAnimations() {
        candle.clearAnimation(0);
    }

    private void clearState() {
        currentAnimation = null;
        currentAnimation2 = null;
        currentColor = null;
        currentSplitColor = null;
    }

    public SingleFadeAnimation fade(LEDColor color, double speed) {
        return new SingleFadeAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS, STRIP_OFFSET);
    }

    public TwinkleAnimation twinkle(LEDColor color, double speed, TwinklePercent percent) {
        return new TwinkleAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS, percent, STRIP_OFFSET);
    }

    public TwinkleOffAnimation twinkleOff(LEDColor color, double speed, TwinkleOffPercent percent) {
        return new TwinkleOffAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS, percent, STRIP_OFFSET);
    }

    public ColorFlowAnimation colorFlow(LEDColor color, double speed, Direction direction) {
        return colorFlow(color, speed, direction, NUM_LEDS, STRIP_OFFSET);
    }

    public ColorFlowAnimation colorFlow(LEDColor color, double speed, Direction direction, int numLeds, int ledOffset) {
        return new ColorFlowAnimation(color.r, color.g, color.b, color.w, speed, numLeds, direction, ledOffset);
    }

    public LarsonAnimation larson(LEDColor color, double speed, int size, BounceMode mode) {
        return new LarsonAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS - NO_CANDLE_OFFSET, mode, size, NO_CANDLE_OFFSET);
    }

    public StrobeAnimation strobe(LEDColor color, double speed) {
        return new StrobeAnimation(color.r, color.g, color.b, color.w, speed, NUM_LEDS, STRIP_OFFSET);
    }

    private record LEDColor(int r, int g, int b, int w) {}
}
