package frc.spectrumLib.sim;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public class Length implements StructSerializable {

    public static final Length kZero = new Length();

    private final double inches;

    public Length() {
        this(0);
    }

    public Length(double inches) {
        this.inches = inches;
    }

    public static Length fromInches(double inches) {
        return new Length(inches);
    }

    public static Length fromFeet(double feet) {
        return from(feet, 12.0);
    }

    public static Length fromCentimeters(double centimeters) {
        return from(centimeters, 1.0 / 2.54);
    }

    public static Length fromMeters(double meters) {
        return from(meters, 39.37);
    }

    public static Length fromRotations(double rotations, Length circumference) {
        return new Length(rotations * circumference.inches);
    }

    public static Length from(double value, double inchConversionFactor) {
        return new Length(value * inchConversionFactor);
    }

    public Length plus(Length other) {
        return new Length(this.inches + other.getInches());
    }

    public Length minus(Length other) {
        return new Length(this.inches - other.getInches());
    }

    public Length times(double scalar) {
        return new Length(this.inches * scalar);
    }

    public Length div(double scalar) {
        return new Length(this.inches / scalar);
    }

    public Length times(Length scalar) {
        return new Length(this.inches * scalar.inches);
    }

    public Length div(Length scalar) {
        return new Length(this.inches / scalar.inches);
    }

    public Length squared() {
        return new Length(this.inches * this.inches);
    }

    public double getInches() {
        return inches;
    }

    public double getFeet() {
        return inches / 12.0;
    }

    public double getCentimeters() {
        return inches * 2.54;
    }

    public double getMeters() {
        return inches / 39.37;
    }

    @Override
    public boolean equals(Object obj) {
        return this.inches == ((Length) obj).inches;
    }

    public static final LengthStruct struct = new LengthStruct();

    private static class LengthStruct implements Struct<Length> {

        @Override
        public Class<Length> getTypeClass() {
            return Length.class;
        }

        @Override
        public String getTypeName() {
            return "Length";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 4;
        }

        @Override
        public String getSchema() {
            return "double inches;double feet;double centimeters;double meters";
        }

        @Override
        public Length unpack(ByteBuffer bb) {
            double inches = bb.getDouble();
            return new Length(inches);
        }

        @Override
        public void pack(ByteBuffer bb, Length value) {
            bb.putDouble(value.getInches());
            bb.putDouble(value.getFeet());
            bb.putDouble(value.getCentimeters());
            bb.putDouble(value.getMeters());
        }

        @Override
        public boolean isImmutable() {
            return true;
        }
    }
}
