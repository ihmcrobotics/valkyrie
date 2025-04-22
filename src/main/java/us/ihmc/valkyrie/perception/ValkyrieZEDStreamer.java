package us.ihmc.valkyrie.perception;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.zed.SL_InitParameters;
import us.ihmc.zed.SL_RuntimeParameters;
import us.ihmc.zed.library.ZEDJavaAPINativeLibrary;

import static us.ihmc.zed.global.zed.*;

public class ValkyrieZEDStreamer
{
   public static final int PORT = 31332;

   private static volatile boolean running = true;

   static
   {
      /*
        Load the zed-java-api library
        https://github.com/ihmcrobotics/zed-java-api
       */
      ZEDJavaAPINativeLibrary.load();

      /*
         Shutdown hook to close the local ZED sensor
       */
      Runtime.getRuntime().addShutdownHook(new Thread(ValkyrieZEDStreamer::destroy));
   }

   public static void main(String[] args)
   {
      /*
         Start the local USB ZED sensor
       */
      startLocalUSBSensor();

      /*
         Do nothing forever, everything else runs in other threads
       */
      ThreadTools.sleepForever();
   }

   public static void destroy()
   {
      running = false;

      stopLocalUSBSensor();

      System.out.println("Stopped");
   }

   private static void stopLocalUSBSensor()
   {
      /*
         All we need to do to stop the USB ZED sensor is call sl_close_camera
       */
      sl_close_camera(0);
   }

   private static void startLocalUSBSensor()
   {
      /*
         Set up the USB ZED
       */
      sl_create_camera(0);
      SL_InitParameters initParameters = new SL_InitParameters();
      initParameters.camera_fps(15);
      initParameters.resolution(SL_RESOLUTION_HD720);
      initParameters.input_type(SL_INPUT_TYPE_USB);
      initParameters.camera_device_id(0);
      int state = sl_open_camera(0, initParameters, 0, "", "", 0, "", "", "");
      sl_enable_streaming(0, SL_STREAMING_CODEC_H264, 6000, (short) PORT, -1, 0, 16084, 60);
      if (state != 0)
         throw new RuntimeException("Could not initialize ZED");

      /*
         Start grabbing images in a thread
       */
      SL_RuntimeParameters runtimeParameters = new SL_RuntimeParameters();
      runtimeParameters.enable_depth(true);
      Thread imageGrabThread = new Thread(() ->
      {
         while (running)
         {
            // Grab the image and do nothing with it
            sl_grab(0, runtimeParameters);
         }
      }, "ImageGrabThread");
      imageGrabThread.start();
   }
}
