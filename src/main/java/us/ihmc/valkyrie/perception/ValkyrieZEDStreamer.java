package us.ihmc.valkyrie.perception;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;

public class ValkyrieZEDStreamer
{
   private final ZEDImageSensor zedImageSensor;

   public ValkyrieZEDStreamer()
   {
      zedImageSensor = new ZEDImageSensor(0, ZEDModelData.ZED_MINI, zed.SL_INPUT_TYPE_USB, zed.SL_DEPTH_MODE_NEURAL);
   }

   public void start()
   {
      zedImageSensor.run(true);

      LogTools.info("Running");
   }

   public void stop()
   {
      zedImageSensor.run(false);
      zedImageSensor.close();
   }

   public static void main(String[] args)
   {
      ValkyrieZEDStreamer valkyrieZEDStreamer = new ValkyrieZEDStreamer();

      valkyrieZEDStreamer.start();

      Runtime.getRuntime().addShutdownHook(new Thread(valkyrieZEDStreamer::stop, "Shutdown"));

      ThreadTools.sleepForever();
   }
}
