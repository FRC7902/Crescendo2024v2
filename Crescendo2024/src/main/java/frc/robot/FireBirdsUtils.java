package frc.robot;

public class FireBirdsUtils {

  public int radsToCTRESensorUnits(double angleInRads, int encoderCPR) {
    return (int) ((angleInRads) / (2 * Math.PI) * encoderCPR);
  }

  public double CTRESensorUnitsToRads(double angleInSensorUnits, int encoderCPR) {
    return (double) ((angleInSensorUnits / encoderCPR) * 2 * Math.PI);
  }

  public int degToCTRESensorUnits(double angleInDeg, int encoderCPR) {
    return (int) (((angleInDeg) / 360) * encoderCPR);
  }

  public double CTRESensorUnitsToDegs(double angleInSensorUnits, int encoderCPR) {
    return (double) ((angleInSensorUnits / encoderCPR) * 360);
  }

  public double[] setZeiglerNicholsConstants(double ku, double tu){
    double constants[] = new double[3];
    constants[0] = 0.6 * ku;
    constants[1] = 1.2 * ku / tu;
    constants[2] = 0.075 * ku * tu;
    return constants;
  }
}