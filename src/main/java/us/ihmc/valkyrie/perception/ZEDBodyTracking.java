package us.ihmc.valkyrie.perception;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.*;
import us.ihmc.zed.global.zed;
import us.ihmc.zed.library.ZEDJavaAPINativeLibrary;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.zed.global.zed.*;

public class ZEDBodyTracking
{
   private static final boolean ZED_SDK_LOADED = ZEDJavaAPINativeLibrary.load();
   private static final int CAMERA_ID = 0;
   private ZEDImageSensor zedSensor;
   private final SL_Bodies trackedBodies = new SL_Bodies();
   private final List<Point3D> bodyPartLocations = new ArrayList<>();
   private final SL_BodyTrackingRuntimeParameters bodyTrackingRuntimeParameters = new SL_BodyTrackingRuntimeParameters();
   private final SL_RuntimeParameters slRuntimeParameters = new SL_RuntimeParameters();
   private boolean enabled = false;

   public ZEDBodyTracking()
   {
      if (ZED_SDK_LOADED)
      {
         zedSensor = new ZEDImageSensor(CAMERA_ID, ZEDModelData.ZED_2, zed.SL_INPUT_TYPE_USB, zed.SL_DEPTH_MODE_PERFORMANCE);
      }
      else
      {
         LogTools.error("ZED SDK not found. Not using ZED sensor.");
      }
   }


   public void initialize()
   {
      SL_InitParameters initParams = new SL_InitParameters();
      initParams.camera_fps(30);
      initParams.resolution(SL_RESOLUTION_HD1080);
      initParams.input_type(SL_INPUT_TYPE_USB);
      initParams.camera_device_id(CAMERA_ID);
      initParams.camera_image_flip(zed.SL_FLIP_MODE_AUTO);
      initParams.camera_disable_self_calib(false);
      initParams.enable_image_enhancement(true);
      initParams.svo_real_time_mode(true);
      initParams.depth_mode(SL_DEPTH_MODE_PERFORMANCE);
      initParams.depth_stabilization(1);
      initParams.depth_maximum_distance(40);
      initParams.depth_minimum_distance(-1);
      initParams.coordinate_unit(SL_UNIT_METER);
      initParams.coordinate_system(SL_COORDINATE_SYSTEM_LEFT_HANDED_Y_UP);
      initParams.sdk_gpu_id(-1);
      initParams.sdk_verbose(0);
      initParams.sensors_required(false);
      initParams.enable_right_side_measure(false);
      initParams.async_grab_camera_recovery(false);
      initParams.grab_compute_capping_fps(0);
      initParams.enable_image_validity_check(0);


      if (sl_open_camera(CAMERA_ID, initParams, 0, "", "", 0, "", "", "") != 0)
      {
         throw new RuntimeException("Failed to open ZED camera.");
      }

      SL_PositionalTrackingParameters trackingParams = new SL_PositionalTrackingParameters();
      trackingParams.enable_area_memory(true);
      trackingParams.enable_imu_fusion(true);
      trackingParams.enable_pose_smothing(false);
      trackingParams.depth_min_range(-1);
      //            position.x(0).y(0).z(0);
      //            rotation.x(0).y(0).z(0).w(1);
      //            slTrackingParameters.initial_world_position(position);
      //            slTrackingParameters.initial_world_rotation(rotation);
      trackingParams.set_as_static(false);
      trackingParams.set_floor_as_origin(false);
      trackingParams.set_gravity_as_origin(true);
      trackingParams.mode(zed.SL_POSITIONAL_TRACKING_MODE_GEN_1);

      if (sl_enable_positional_tracking(CAMERA_ID, trackingParams, "") != 0)
      {
         throw new RuntimeException("Failed to enable positional tracking.");
      }

      SL_BodyTrackingParameters bodyTrackingParams = new SL_BodyTrackingParameters();
      bodyTrackingParams.enable_segmentation(false);
      bodyTrackingParams.enable_tracking(true);
      bodyTrackingParams.enable_body_fitting(true);
      bodyTrackingParams.max_range(40);
      bodyTrackingParams.detection_model(SL_BODY_TRACKING_MODEL_HUMAN_BODY_MEDIUM);
      bodyTrackingParams.allow_reduced_precision_inference(false);
      bodyTrackingParams.body_format(SL_BODY_FORMAT_BODY_18);
      bodyTrackingParams.body_selection(SL_BODY_KEYPOINTS_SELECTION_FULL);
      bodyTrackingParams.instance_module_id(0);

      if (sl_enable_body_tracking(CAMERA_ID, bodyTrackingParams) != 0)
      {
         throw new RuntimeException("Failed to enable body tracking.");
      }

      bodyTrackingRuntimeParameters.detection_confidence_threshold(40);
      bodyTrackingRuntimeParameters.minimum_keypoints_threshold(1);
      bodyTrackingRuntimeParameters.skeleton_smoothing(0.0f);

      slRuntimeParameters.enable_depth(true);
      slRuntimeParameters.confidence_threshold(95);
      slRuntimeParameters.reference_frame(zed.SL_REFERENCE_FRAME_CAMERA);
      slRuntimeParameters.texture_confidence_threshold(100);
      slRuntimeParameters.confidence_threshold(95);
      slRuntimeParameters.remove_saturated_areas(true);
   }

   public void enable()
   {
      enabled = true;
      runTrackingLoop();
   }

   public void disable()
   {
      enabled = false;
   }

   private void runTrackingLoop()
   {
      new Thread(() ->
                 {
                    while (enabled)
                    {
                       if (sl_grab(CAMERA_ID, slRuntimeParameters) == 0)
                       {
                          sl_retrieve_bodies(CAMERA_ID, bodyTrackingRuntimeParameters, trackedBodies, 0);
                          if (trackedBodies.is_new() > 0 && trackedBodies.nb_bodies() > 0)
                          {
                             bodyPartLocations.clear();
                             SL_BodyData bodyData = trackedBodies.body_list(0);
                             int[] keypointIndices = {1,2,5,4,7}; // e.g., shoulder, elbow, etc.

                             for (int i : keypointIndices)
                             {
                                SL_Vector3 kp = bodyData.keypoint(i);
                                Point3D point = new Point3D(kp.x(), kp.y(), kp.z());
                                bodyPartLocations.add(point);
                                System.out.printf("Keypoint %d: (%.2f, %.2f, %.2f)%n", i, kp.x(), kp.y(), kp.z());
                             }

                             LogTools.info("Tracked Body ID: " + bodyData.id());
                          }
                       }

                       try
                       {
                          Thread.sleep(30); // 30 fps
                       }
                       catch (InterruptedException e)
                       {
                          Thread.currentThread().interrupt();
                       }
                    }

                    sl_close_camera(CAMERA_ID);
                 }).start();
   }

   public List<Point3D> getBodyPartLocations()
   {
      return new ArrayList<>(bodyPartLocations);
   }

   public static void main(String[] args)
   {
      ZEDBodyTracking tracker = new ZEDBodyTracking();
      tracker.initialize();
      tracker.enable();
   }
}
