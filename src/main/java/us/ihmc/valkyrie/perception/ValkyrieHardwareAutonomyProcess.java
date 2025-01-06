package us.ihmc.valkyrie.perception;

import us.ihmc.sensors.ZEDImageSensor;
import us.ihmc.sensors.ZEDModelData;
import us.ihmc.zed.global.zed;

public class ValkyrieHardwareAutonomyProcess
{
   private final ZEDImageSensor zedSensor;

   private final ValkyrieAutonomyProcess perceptionAutonomyProcess;

   private ValkyrieHardwareAutonomyProcess()
   {
      zedSensor = new ZEDImageSensor(0, ZEDModelData.ZED_2I, zed.SL_INPUT_TYPE_USB);

      Runtime.getRuntime().addShutdownHook(new Thread(this::close, getClass().getSimpleName() + "Closer"));

      perceptionAutonomyProcess = new ValkyrieAutonomyProcess(zedSensor);
   }

   private void close()
   {
      perceptionAutonomyProcess.close();
   }

   public static void main(String[] args)
   {
      new ValkyrieHardwareAutonomyProcess();
   }
}
