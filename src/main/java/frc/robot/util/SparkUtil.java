package frc.robot.util;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SparkUtil {
  // allows us to see if the sparkmax has had an error in the past
  public static boolean stickyFault = false;

  //only takes the sparks value if the spark isn't erroring
  public static void isOk(SparkBase spark, DoubleSupplier doubleSupplier, DoubleConsumer doubleConsumer){
    double num = doubleSupplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk){
        doubleConsumer.accept(num);
    }
    else {
        stickyFault = true;
        }
    }

    // same thing but can take in values as a array
    public static void isOK (SparkBase spark, DoubleSupplier[] doubleSuppliers, Consumer<double[]> doubleConsumers ){
        double[] nums = new double[doubleSuppliers.length];
        for (int i = 0; i < doubleSuppliers.length; i++) {
            nums[i] = doubleSuppliers[i].getAsDouble();
            if (spark.getLastError() != REVLibError.kOk) {
                stickyFault = true;
                return;
            }
        }
        doubleConsumers.accept(nums);
    }
    public static void makeItWork(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
        for (int i =0; i < maxAttempts; i++){
            var error = command.get();
            if (error == REVLibError.kOk) {
                break;
            }
            else {
                stickyFault = true;
            }
        }
    }
}
