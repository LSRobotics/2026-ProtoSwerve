package frc.robot.util;
import java.util.function.Supplier;
import java.util.jar.Attributes.Name;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class SendableSupplier<T> implements Supplier<T>, Sendable {
    private final Supplier<T> supplier;
    private final String name;

    public SendableSupplier(String name, Supplier<T> supplier) {
        this.supplier = supplier;
        this.name = name;
    }

    @Override
    public T get() {
        return supplier.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        T sample = supplier.get();

        System.out.println("=== initSendable called for: " + name + " ===");
        
        if (sample instanceof Boolean) {
            builder.addBooleanProperty(name, 
                () -> (Boolean) supplier.get(), 
                null);
        } else if (sample instanceof Double || sample instanceof Float) {
            builder.addDoubleProperty(name, 
                () -> ((Number) supplier.get()).doubleValue(), 
                null);
        } else if (sample instanceof Integer || sample instanceof Long) {
            builder.addIntegerProperty(name, 
                () -> ((Number) supplier.get()).intValue(), 
                null);
        } 
        else if (sample instanceof String) {
            builder.addStringProperty(name, 
                () -> (String) supplier.get(), 
                null);
        }
        else if (sample instanceof Character) {
            builder.addStringProperty(name, 
                () -> String.valueOf((Character) supplier.get()), 
                null);
        }

        else if (sample == null) {
            builder.addStringProperty(name, 
                () -> "null", 
                null);
        }
        else {
            // default to str 
            builder.addStringProperty(name, 
                () -> String.valueOf(supplier.get()), 
                null);
        }
    }
}