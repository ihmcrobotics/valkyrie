package us.ihmc.valkyrie.perception;

import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;

public class ValkyrieHardwareAutonomyProcess
{
   private final ValkyrieAutonomyProcess perceptionAutonomyProcess;

   private ValkyrieHardwareAutonomyProcess()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(this::close, getClass().getSimpleName() + "Closer"));

      ZEDImageSensor zedImageSensor = new ZEDImageSensor(0, ZEDModelData.ZED_MINI, zed.SL_INPUT_TYPE_STREAM, zed.SL_DEPTH_MODE_PERFORMANCE, "192.168.100.21", ValkyrieZEDStreamer.PORT);
      perceptionAutonomyProcess = new ValkyrieAutonomyProcess(zedImageSensor);
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
