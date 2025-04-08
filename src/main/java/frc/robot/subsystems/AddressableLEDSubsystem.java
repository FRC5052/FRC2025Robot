// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleFunction;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AddressableLEDSubsystem extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private final AddressableLEDSlice fullStrip;
  private final Color colorCorrection = new Color(1.0, 1.0, 1.0);

  public static class AddressableLEDSlice {
    private final AddressableLEDSubsystem parent;
    private int offset;
    private int length;
    private boolean reversed;

    private AddressableLEDSlice(AddressableLEDSubsystem parent, int offset, int length) {
      this.parent = parent;
      this.offset = offset;
      this.length = length;
      this.reversed = false;
    }

    private AddressableLEDSlice(AddressableLEDSubsystem parent, int offset, int length, boolean reversed) {
      this.parent = parent;
      this.offset = offset;
      this.length = length;
      this.reversed = reversed;
    }

    public int getOffset() {
      return this.offset;
    }

    public void setOffset(int offset) {
      // if (length+offset > parent.length()) {
      //   throw new IndexOutOfBoundsException();
      // }
      this.offset = offset;
    }

    public int getLength() {
      return this.length;
    }

    public void setLength(int length) {
      // if (length+offset > parent.length()) {
      //   throw new IndexOutOfBoundsException();
      // }
      this.length = length;
    }

    public void setReversed(boolean b) {
      this.reversed = b;
    }

    public boolean isReversed() {
      return this.reversed;
    }

    public void fill(Color color) {
      for (int i = offset; i < length+offset; i++) {
        parent.set(i, color);
      }
    }

    public void fillWithOffset(Color color, int startOffset, int endOffset) {
      this.setFunc((double value) -> {
        return color;
      },
      startOffset, endOffset);
    }
    
    public void setFunc(DoubleFunction<Color> func, int startOffset, int endOffset) {
      if (startOffset>length || endOffset > length) { return; }
      for (int i = offset+startOffset; i < length+offset-endOffset; i++) {
        double value = (((double)(i-offset))/(double)length);
        if (reversed) value = 1.0 - value;
        Color color = func.apply(value);
        if (color != null) {
          parent.set(i, color);
        }
      }
    }

    public void setFunc(DoubleFunction<Color> func, int startOffset) {
      setFunc(func, startOffset, 0);
    }

    public void setFunc(DoubleFunction<Color> func) {
      setFunc(func, 0, 0);
    }

    public void setHueSine(Timer timer, int hue) {
      this.setFunc((double value) -> {
        return Color.fromHSV(hue, 255, (int)((Math.sin((value + (timer.get() / 5))*Math.PI*2*5)+1)*127));
      });
    }

    public void setDoubleRGBSine(Timer timer, double speed, double frequency, Color color1, Color color2) {
      this.setFunc((double value) -> {
        double mult = (Math.sin((value + (timer.get() / speed))*Math.PI*2*frequency)+1)*0.5;
        return new Color(
          (color1.red*mult)+(color2.red*(1-mult)), 
          (color1.green*mult)+(color2.green*(1-mult)), 
          (color1.blue*mult)+(color2.blue*(1-mult))
        );
      });
    }

    public void setMeter(double threshold, Color onColor, Color offColor) {
      this.setFunc((double value) -> {
        if (value >= threshold) {
          return offColor;
        } else {
          return onColor;
        }
      });
    }

    public void setMeter(double total, double position, Color onColor, Color offColor) {
      this.setMeter(position/total, onColor, offColor);
    }

    public void setMeter(int total, int position, Color onColor, Color offColor) {
      this.setMeter((double)total, (double)position, onColor, offColor);
    }

    public void setSpooky(Timer timer) {
      this.setHueSine(timer, 4);
    }

    public void setRainbow(Timer timer) {
      this.setFunc((double value) -> {
        return Color.fromHSV((int)(((value + (timer.get() / 5)) % 1) * 180), 255, (int)((Math.sin((value + (timer.get() / 3))*Math.PI*2*5)+1)*127));
      });
    }
    public void setColor(double r, double g, double b) {
      this.setFunc((double value) -> {
        return new Color(r,g,b);      
      });
    }
  }

  public AddressableLEDSubsystem(int port, int length) {
    // PWM Port of LED
    led = new AddressableLED(port);

    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(length);
    led.setLength(ledBuffer.getLength());

    led.start();

    fullStrip = createSlice();
  }

  private void set(int index, Color color) {
    this.ledBuffer.setRGB(
      index, 
      (int)(Math.min(color.red*colorCorrection.red, 1.0) * 255), 
      (int)(Math.min(color.green*colorCorrection.green, 1.0) * 255), 
      (int)(Math.min(color.blue*colorCorrection.blue, 1.0) * 255)
    );
  }

  public AddressableLEDSlice createSlice(int offset, int length, boolean reversed) {
    return new AddressableLEDSlice(this, offset, length, reversed);
  }

  public AddressableLEDSlice createSlice(int offset, int length) {
    return this.createSlice(offset, length, false);
  }

  public AddressableLEDSlice createSlice() {
    return this.createSlice(0, this.length());
  }

  public void display() {
    this.led.setData(this.ledBuffer);
  }

  public int length() {
    return this.ledBuffer.getLength();
  }

  public AddressableLEDSlice getFullStrip() {
    return fullStrip;
  }

  public Command getFillCommand(Color c) {
    return new Command() {
      private AddressableLEDSlice strip;

      @Override
      public void initialize() {
        strip = getFullStrip();
      }

      @Override
      public void execute() {
          strip.fill(c);
      }
    };
  }

  public Command getFillCommand(AddressableLEDSlice targetSlice, Color c) {
    return new Command() {
      private AddressableLEDSlice slice;

      @Override
      public void initialize() {
        slice = targetSlice;
      }

      @Override
      public void execute() {
          slice.fill(c);
      }
    };
  }

  public Command getFillCommand(int r, int g, int b) {
    return new Command() {
      private AddressableLEDSlice strip;

      @Override
      public void initialize() {
        strip = getFullStrip();
      }

      @Override
      public void execute() {
          strip.setColor(r, g, b);
      }
    };
  }

  public Command getFillCommand(AddressableLEDSlice targetSlice, int r, int g, int b) {
    return new Command() {
      private AddressableLEDSlice slice;

      @Override
      public void initialize() {
        slice = targetSlice;
      }

      @Override
      public void execute() {
          slice.setColor(r, g, b);
      }
    };
  }
  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
