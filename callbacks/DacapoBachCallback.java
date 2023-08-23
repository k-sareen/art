import org.dacapo.harness.Callback;
import org.dacapo.harness.CommandLineArgs;

public class DacapoBachCallback extends Callback {
  private AndroidCallback cb;

  public DacapoBachCallback(CommandLineArgs cla) {
    super(cla);
    cb = new AndroidCallback();
  }

  public void start(String benchmark) {
    if (!isWarmup()) {
      cb.start();
    }
    super.start(benchmark);
  }

  public void stop(long duration) {
    super.stop(duration);
    if (!isWarmup()) {
      cb.stop();
    }
  }
}
