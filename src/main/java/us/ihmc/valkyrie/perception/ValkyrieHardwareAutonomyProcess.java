package us.ihmc.valkyrie.perception;

import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;

public class ValkyrieHardwareAutonomyProcess
{
   private final ZEDImageSensor zedSensor;
   private final ValkyrieAutonomyProcess perceptionAutonomyProcess;

   private ValkyrieHardwareAutonomyProcess()
   {
      zedSensor = new ZEDImageSensor(0, ZEDModelData.ZED_MINI, zed.SL_INPUT_TYPE_USB);

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
