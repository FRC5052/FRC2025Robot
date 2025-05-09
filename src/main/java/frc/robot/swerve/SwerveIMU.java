package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;

import java.util.Objects;
import java.util.Optional;

import com.fasterxml.jackson.databind.JsonNode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.units.*;

public abstract class SwerveIMU {
    public enum SwerveIMUType {
        kNavX;

        public static SwerveIMUType fromString(String str) {
            switch (str) {
                case "kNavX":
                    return kNavX;
                default:
                    return null;
            }
        }
    }

    public static final SwerveIMUBuilder getBuilder(SwerveIMUType type) {
        switch (type) {
            case kNavX:
                return new NavXSwerveIMU.Builder();
            default:
                return null;
        }
    }

    public static final SwerveIMUBuilder builderFromJSON(JsonNode json) {
        var builder = getBuilder(SwerveIMUType.fromString(json.get("type").asText()));
        builder.fromJSON(json);
        return builder;
    }

    /** 
     * Returns the measured 3D rotation of this IMU.
     * @return The Rotation3d object representing the 3D rotation this IMU is reporting.
     */
    public abstract Rotation3d getRotation();

    /** 
     * Returns the heading of this IMU in the given unit, with heading correction applied. 
     * @param unit The unit of angle to convert the measurement into.
     * @return The measured heading, in the given unit.
     */
    public abstract double getHeading(AngleUnit unit);

    /** 
     * Returns the heading of this IMU in the given unit, without heading correction. 
     * @param unit The unit of angle to convert the measurement into.
     * @return The measured heading, in the given unit.
     */
    public abstract double getRawHeading(AngleUnit unit);

    /** 
     * Returns the heading of this IMU relative to the planet's magnetic field (if present), in the given unit. 
     * @param unit The unit of angle to convert the measurement into.
     * @return The measured heading, in the given unit.
     * @apiNote On some IMU implementations, the IMU has to be calibrated manually for this function to work correctly.
     */
    public abstract double getCompassHeading(AngleUnit unit);

    /** 
     * Returns the measured 3D acceleration of this IMU.
     * @param unit The unit of acceleration to convert the measurement into.
     * @return The Translation3d object representing the 3D acceleration this IMU is reporting.
     */
    public abstract Translation3d getWorldAccel(LinearAccelerationUnit unit);

    /** 
     * Calibrates this IMU.
     * @return Whether the IMU was actually calibrated. 
     */
    public abstract boolean calibrate();

    /**
     * Sets the heading offset to the raw heading reported by this IMU. 
     * The heading returned by {@link #getHeading(AngleUnit)} is guaranteed to be zero after this call.
     */
    public abstract void resetHeading();

    /**
     * Gets this IMU's configured heading offset.
     * @param unit The unit of angle to convert the value into.
     * @return The configured heading offset, in the given units.
     */
    public abstract double getHeadingOffset(AngleUnit unit);
    
    /**
     * Sets this IMU's configured heading offset.
     * @param offset The new heading offset value, in the given units
     * @param unit The unit of angle to convert the value into.
     */
    public abstract void setHeadingOffset(double offset, AngleUnit unit);

    public static interface SwerveIMUBuilder {
        public SwerveIMUBuilder clone();
        public void fromJSON(JsonNode json);
        public SwerveIMU build();
    }
    
    public static class NavXSwerveIMU extends SwerveIMU {
        private AHRS navX;

        private NavXSwerveIMU(Measure<AngleUnit> offset) {
            this.navX = new AHRS(NavXComType.kMXP_SPI);
            this.setHeadingOffset(offset.magnitude(), offset.unit());
        }

        private NavXSwerveIMU(SPI.Port port, Measure<AngleUnit> offset) {
            this.navX = new AHRS(NavXComType.kMXP_SPI);
            this.setHeadingOffset(offset.magnitude(), offset.unit());
        }

        private NavXSwerveIMU(I2C.Port port, Measure<AngleUnit> offset) {
            this.navX = new AHRS(NavXComType.kI2C);
            this.setHeadingOffset(offset.magnitude(), offset.unit());
        }

        private NavXSwerveIMU(SerialPort.Port port, Measure<AngleUnit> offset) {
            if(port.equals(SerialPort.Port.kUSB1)) {
                this.navX = new AHRS(NavXComType.kUSB1);
            } else if(port.equals(SerialPort.Port.kUSB2)) {
                this.navX = new AHRS(NavXComType.kUSB2);
            } else {
                this.navX = new AHRS(NavXComType.kUSB1);
            }
            this.setHeadingOffset(offset.magnitude(), offset.unit());
        }

        @Override
        public Rotation3d getRotation() {
            return new Rotation3d(new Quaternion(
                (double)this.navX.getQuaternionW(), 
                (double)this.navX.getQuaternionX(), 
                (double)this.navX.getQuaternionY(), 
                (double)this.navX.getQuaternionZ()
            ));
        }

        @Override
        public double getHeading(AngleUnit unit) {
            return this.getRawHeading(unit) - this.getHeadingOffset(unit);
        }

        @Override
        public double getRawHeading(AngleUnit unit) {
            return unit.convertFrom(-(double)this.navX.getFusedHeading(), Degrees);
        }

        @Override
        public double getCompassHeading(AngleUnit unit) {
            return unit.convertFrom(-(double)this.navX.getCompassHeading(), Degrees);
        }

        @Override
        public Translation3d getWorldAccel(LinearAccelerationUnit unit) {
            return new Translation3d(
                unit.convertFrom(this.navX.getWorldLinearAccelX(), Gs), 
                unit.convertFrom(this.navX.getWorldLinearAccelY(), Gs), 
                unit.convertFrom(this.navX.getWorldLinearAccelZ(), Gs)
            );
        }

        @Override
        public boolean calibrate() {
            return false;
        }

        @Override
        public void resetHeading() {
            this.setHeadingOffset(this.getRawHeading(Radians), Radians);
        }

        @Override
        public double getHeadingOffset(AngleUnit unit) {
            return unit.convertFrom(-this.navX.getAngleAdjustment(), Degrees);
        }

        @Override
        public void setHeadingOffset(double offset, AngleUnit unit) {
            this.navX.setAngleAdjustment(-Degrees.convertFrom(offset, unit));
        }

        public static class Builder implements SwerveIMUBuilder {
            private Optional<Object> port = Optional.empty();
            private PortType portType = PortType.SPI;
            private MutAngle offset = Radians.mutable(0);

            @Override
            public Builder clone() {
                Builder tmp = new Builder();
                tmp.port = this.port.isPresent() ? Optional.of(this.port.get()) : Optional.empty();
                tmp.offset = this.offset.mutableCopy();
                return tmp;
            }

            public Builder withPort() {
                this.port = Optional.of(SPI.Port.kMXP);
                this.portType = PortType.SPI;
                return this;
            }

            public Builder withPort(SPI.Port port) {
                this.port = Optional.of(port);
                this.portType = PortType.SPI;
                return this;
            }

            public Builder withPort(I2C.Port port) {
                this.port = Optional.of(port);
                this.portType = PortType.I2C;
                return this;
            }

            public Builder withPort(SerialPort.Port port) {
                this.port = Optional.of(port);
                this.portType = PortType.SerialPort;
                return this;
            }

            public Builder withOffset(double offset, AngleUnit unit) {
                this.offset.mut_replace(offset, unit);
                return this;
            }

            public Builder withOffset(Measure<AngleUnit> offset) {
                this.offset.mut_replace(offset.magnitude(), offset.unit());
                return this;
            }

            @Override
            public void fromJSON(JsonNode json) {
                if (json.has("offset") && (json.get("offset").isObject() || json.get("offset").isDouble())) {
                    JsonNode json_inner = json.get("offset");
                    AngleUnit unit = Degrees;
                    if (json_inner.has("unit") && json_inner.get("unit").isTextual() && json_inner.has("value") && json_inner.get("value").isDouble()) {
                        unit = Objects.requireNonNullElse(SwerveUtil.angleFromName(json_inner.get("unit").textValue()), unit);
                        this.withOffset(json_inner.get("value").doubleValue(), unit);
                    } else {
                        this.withOffset(json_inner.doubleValue(), unit);
                    }
                }
                if (json.has("spiPort") && json.get("spiPort").isTextual()) {
                    String port = json.get("spiPort").textValue();
                    switch (port) {
                        case "kOnboardCS0":
                            this.withPort(SPI.Port.kOnboardCS0);
                            break;
                        case "kOnboardCS1":
                            this.withPort(SPI.Port.kOnboardCS1);
                            break;
                        case "kOnboardCS2":
                            this.withPort(SPI.Port.kOnboardCS2);
                            break;
                        case "kOnboardCS3":
                            this.withPort(SPI.Port.kOnboardCS3);
                            break;
                        case "kMXP":
                            this.withPort(SPI.Port.kMXP);
                            break;
                    }
                } else if (json.has("i2cPort") && json.get("i2cPort").isTextual()) {
                    String port = json.get("i2cPort").textValue();
                    switch (port) {
                        case "kOnboard":
                            this.withPort(I2C.Port.kOnboard);
                            break;
                        case "kMXP":
                            this.withPort(I2C.Port.kMXP);
                            break;
                    }
                } else if (json.has("serialPort") && json.get("serialPort").isTextual()) {
                    String port = json.get("serialPort").textValue();
                    switch (port) {
                        case "kOnboard":
                            this.withPort(SerialPort.Port.kOnboard);
                            break;
                        case "kMXP":
                            this.withPort(SerialPort.Port.kMXP);
                            break;
                        case "kUSB":
                            this.withPort(SerialPort.Port.kUSB);
                            break;
                        case "kUSB1":
                            this.withPort(SerialPort.Port.kUSB1);
                            break;
                        case "kUSB2":
                            this.withPort(SerialPort.Port.kUSB2);
                            break;
                    }
                }

            }

            @Override
            public SwerveIMU build() {
                if (this.port.isEmpty()) {
                    throw new IllegalStateException("Port field was empty");
                }
                switch (this.portType) {
                    case I2C:
                        return new NavXSwerveIMU((I2C.Port)this.port.get(), this.offset);
                    case SPI:
                        return new NavXSwerveIMU((SPI.Port)this.port.get(), this.offset);
                    case SerialPort:
                        return new NavXSwerveIMU((SerialPort.Port)this.port.get(), this.offset);
                    default:
                        throw new IllegalStateException("Impossible");
                }
            }

            enum PortType {
                SPI,
                I2C,
                SerialPort,
            }
        }
    }
}
