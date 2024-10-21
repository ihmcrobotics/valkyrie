package us.ihmc.valkyrieRosControl;

import ihmc_common_msgs.msg.dds.StampedPosePacket;
import org.apache.commons.math3.util.Precision;
import us.ihmc.affinity.Affinity;
import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.avatar.AvatarEstimatorThreadFactory;
import us.ihmc.avatar.BarrierSchedulerTools;
import us.ihmc.avatar.ControllerTask;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.factory.BarrierScheduledRobotController;
import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.avatar.factory.SingleThreadedRobotController;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler.TaskOverrunBehavior;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.log.LogTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.util.JVMStatisticsGenerator;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.ForceTorqueSensorHandle;
import us.ihmc.rosControl.wholeRobot.IHMCWholeRobotControlJavaBridge;
import us.ihmc.rosControl.wholeRobot.IMUHandle;
import us.ihmc.rosControl.wholeRobot.JointStateHandle;
import us.ihmc.rosControl.wholeRobot.PositionJointHandle;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.fingers.ValkyrieHandStateCommunicator;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.wholeBodyController.DRCOutputProcessorWithStateChangeSmoother;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.*;

public class ValkyrieRosControlController extends IHMCWholeRobotControlJavaBridge
{
   // Note: keep committed as DEFAULT, only change locally if needed
   public static final ValkyrieRobotVersion VERSION = ValkyrieRobotVersion.fromEnvironment();
   public static final String CUSTOM_ROBOT_PATH_ARG = "customRobotPath";

   public static final boolean ENABLE_FINGER_JOINTS = VERSION.hasFingers();
   public static final boolean LOG_SECONDARY_HIGH_LEVEL_STATES = true;

   private static final String[] torqueControlledJoints;

   static
   {
      List<String> jointList = new ArrayList<>();
      jointList.addAll(Arrays.asList("leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll"));
      jointList.addAll(Arrays.asList("rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll"));
      jointList.addAll(Arrays.asList("torsoYaw", "torsoPitch", "torsoRoll"));

      switch (VERSION)
      {
         case DEFAULT:
            jointList.addAll(Arrays.asList("leftIndexFingerMotorPitch1",
                                           "leftMiddleFingerMotorPitch1",
                                           "leftPinkyMotorPitch1",
                                           "leftThumbMotorRoll",
                                           "leftThumbMotorPitch1",
                                           "leftThumbMotorPitch2"));
            jointList.addAll(Arrays.asList("rightIndexFingerMotorPitch1",
                                           "rightMiddleFingerMotorPitch1",
                                           "rightPinkyMotorPitch1",
                                           "rightThumbMotorRoll",
                                           "rightThumbMotorPitch1",
                                           "rightThumbMotorPitch2"));
         case FINGERLESS:
            jointList.addAll(Arrays.asList("leftForearmYaw", "leftWristRoll", "leftWristPitch"));
            jointList.addAll(Arrays.asList("rightForearmYaw", "rightWristRoll", "rightWristPitch"));
         case ARM_MASS_SIM:
            jointList.addAll(Arrays.asList("leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch"));
            jointList.addAll(Arrays.asList("rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch"));
      }

      torqueControlledJoints = jointList.toArray(new String[0]);
   }

   private static final String[] positionControlledJoints = {"lowerNeckPitch", "neckYaw", "upperNeckPitch",};

   private static final String[] allValkyrieJoints;

   static
   {
      List<String> allJointsList = new ArrayList<>();
      Arrays.stream(torqueControlledJoints).forEach(allJointsList::add);
      Arrays.stream(positionControlledJoints).forEach(allJointsList::add);

      if (ENABLE_FINGER_JOINTS)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            String prefix = robotSide.getCamelCaseName();
            allJointsList.addAll(Arrays.asList(prefix + "IndexFingerPitch1", prefix + "IndexFingerPitch2", prefix + "IndexFingerPitch3"));
            allJointsList.addAll(Arrays.asList(prefix + "MiddleFingerPitch1", prefix + "MiddleFingerPitch2", prefix + "MiddleFingerPitch3"));
            allJointsList.addAll(Arrays.asList(prefix + "PinkyPitch1", prefix + "PinkyPitch2", prefix + "PinkyPitch3"));
            allJointsList.addAll(Arrays.asList(prefix + "ThumbRoll", prefix + "ThumbPitch1", prefix + "ThumbPitch2", prefix + "ThumbPitch3"));
         }
      }
      allValkyrieJoints = allJointsList.toArray(new String[0]);
   }

   public static final boolean USE_STATE_CHANGE_TORQUE_SMOOTHER_PROCESSOR = true;
   public static final boolean USE_YOVARIABLE_DESIREDS = false;
   public static final boolean USE_USB_MICROSTRAIN_IMUS = false;
   public static final boolean USE_SWITCHABLE_FILTER_HOLDER_FOR_NON_USB_IMUS = false;
   public static final String[] readIMUs = USE_USB_MICROSTRAIN_IMUS ? new String[0] : new String[ValkyrieSensorInformation.imuSensorsToUse.length];

   static
   {
      if (!USE_USB_MICROSTRAIN_IMUS)
      {
         for (int i = 0; i < ValkyrieSensorInformation.imuSensorsToUse.length; i++)
         {
            readIMUs[i] = ValkyrieSensorInformation.imuSensorsToUse[i].replace("pelvis_", "").replace("torso_", "");
         }
      }
   }

   public static final double gravity = 9.80665;

   public static final String VALKYRIE_IHMC_ROS_ESTIMATOR_NODE_NAME = "valkyrie_ihmc_state_estimator";
   public static final String VALKYRIE_IHMC_ROS_CONTROLLER_NODE_NAME = "valkyrie_" + HumanoidControllerAPI.HUMANOID_CONTROLLER_NODE_NAME;

   private static final WalkingProvider walkingProvider = WalkingProvider.DATA_PRODUCER;

   private YoVariableServer yoVariableServer;
   private AvatarEstimatorThread estimatorThread;
   // Save the reference to the sensor reader, we're going to use this to write controller commands right after the controller is done. 
   private ValkyrieRosControlSensorReader sensorReader;
   private RobotController robotController;

   private final SettableTimestampProvider wallTimeProvider = new SettableTimestampProvider();
   private final TimestampProvider monotonicTimeProvider = () -> RealtimeThread.getCurrentMonotonicClockTime();

   private boolean firstTick = true;

   private final ValkyrieAffinity valkyrieAffinity;
   private boolean isGazebo;

   public ValkyrieRosControlController()
   {
      processEnvironmentVariables();
      valkyrieAffinity = new ValkyrieAffinity(!isGazebo);
   }

   private ValkyrieCalibrationControllerStateFactory calibrationStateFactory = null;

   private HighLevelHumanoidControllerFactory createHighLevelControllerFactory(ValkyrieRobotModel robotModel,
                                                                               RealtimeROS2Node realtimeROS2Node,
                                                                               HumanoidRobotSensorInformation sensorInformation)
   {
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionalContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i),
                                                            additionalContactNames.get(i),
                                                            additionalContactTransforms.get(i));

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      HighLevelControllerParameters highLevelControllerParameters = robotModel.getHighLevelControllerParameters();
      CoPTrajectoryParameters copTrajectoryParameters = robotModel.getCoPTrajectoryParameters();

      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory,
                                                                                                    feetForceSensorNames,
                                                                                                    wristForceSensorNames,
                                                                                                    highLevelControllerParameters,
                                                                                                    walkingControllerParameters,
                                                                                                    copTrajectoryParameters,
                                                                                                    robotModel.getSplitFractionCalculatorParameters());

      controllerFactory.createControllerNetworkSubscriber(robotModel.getSimpleRobotName(), realtimeROS2Node);

      // setup states
      controllerFactory.setInitialState(highLevelControllerParameters.getDefaultInitialControllerState());
      controllerFactory.useDefaultStandPrepControlState();
      controllerFactory.useDefaultStandReadyControlState();
      controllerFactory.useDefaultStandTransitionControlState();
      controllerFactory.useDefaultWalkingControlState();
      controllerFactory.useDefaultExitWalkingTransitionControlState(STAND_PREP_STATE);

      ValkyrieTorqueOffsetPrinter valkyrieTorqueOffsetPrinter = new ValkyrieTorqueOffsetPrinter();
      calibrationStateFactory = new ValkyrieCalibrationControllerStateFactory(valkyrieTorqueOffsetPrinter, robotModel.getCalibrationParameters());
      controllerFactory.addCustomControlState(calibrationStateFactory);

      // setup transitions
      HighLevelControllerName fallbackControllerState = highLevelControllerParameters.getFallbackControllerState();

      HighLevelControllerName calibrationState = calibrationStateFactory.getStateEnum();
      controllerFactory.addRequestableTransition(calibrationState, STAND_PREP_STATE);
      controllerFactory.addFinishedTransition(calibrationState, STAND_PREP_STATE);

      controllerFactory.addFinishedTransition(STAND_PREP_STATE, STAND_READY);
      controllerFactory.addRequestableTransition(STAND_PREP_STATE, calibrationState);

      controllerFactory.addRequestableTransition(STAND_READY, STAND_TRANSITION_STATE);
      if (fallbackControllerState != STAND_READY)
         controllerFactory.addControllerFailureTransition(STAND_READY, fallbackControllerState);
      controllerFactory.addRequestableTransition(STAND_READY, calibrationState);
      controllerFactory.addRequestableTransition(STAND_READY, STAND_PREP_STATE);

      controllerFactory.addFinishedTransition(STAND_TRANSITION_STATE, WALKING, false);
      controllerFactory.addControllerFailureTransition(STAND_TRANSITION_STATE, fallbackControllerState);

      controllerFactory.addRequestableTransition(WALKING, EXIT_WALKING);
      controllerFactory.addFinishedTransition(EXIT_WALKING, STAND_PREP_STATE);
      controllerFactory.addControllerFailureTransition(WALKING, fallbackControllerState);

      if (walkingProvider == WalkingProvider.VELOCITY_HEADING_COMPONENT)
         controllerFactory.createComponentBasedFootstepDataMessageGenerator();
      if (USE_YOVARIABLE_DESIREDS)
         controllerFactory.createUserDesiredControllerCommandGenerator();

      return controllerFactory;
   }

   @Override
   protected void init()
   {
      LogTools.info("Valkyrie robot version: " + VERSION);
      /*
       * Create Robot model
       */

      ValkyrieRobotModel robotModel;
      if (isGazebo)
      {
         robotModel = new ValkyrieRobotModel(RobotTarget.GAZEBO, VERSION);
      }
      else
      {
         robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, VERSION);
      }

      { // Custom robot loading management.
         // First attempt to load a custom robot from a given VM argument indicating path to the custom robot.
         String vmArgumentValue = System.getProperty(CUSTOM_ROBOT_PATH_ARG);

         if (vmArgumentValue != null)
         {
            LogTools.info("Loading custom robot from properties: {}", vmArgumentValue);
            robotModel.setCustomModel(vmArgumentValue);
         }
         else
         { // Otherwise, attempt to load custom robot via environment variable
            robotModel.setCustomModelFromEnvironment();
         }
      }

      String robotName = robotModel.getSimpleRobotName();
      ValkyrieSensorInformation sensorInformation = robotModel.getSensorInformation();

      /*
       * Create joints
       */

      HashSet<String> torqueControlledJointsSet = new HashSet<>(Arrays.asList(torqueControlledJoints));
      HashSet<String> positionControlledJointsSet = new HashSet<>(Arrays.asList(positionControlledJoints));

      HashMap<String, EffortJointHandle> effortJointHandles = new HashMap<>();
      HashMap<String, PositionJointHandle> positionJointHandles = new HashMap<>();
      HashMap<String, JointStateHandle> jointStateHandles = new HashMap<>();

      for (String joint : allValkyrieJoints)
      {
         if (torqueControlledJointsSet.contains(joint) && positionControlledJointsSet.contains(joint))
         {
            throw new RuntimeException("Joint cannot be both position controlled and torque controlled via ROS Control! Joint name: " + joint);
         }

         if (torqueControlledJointsSet.contains(joint))
         {
            effortJointHandles.put(joint, createEffortJointHandle(joint));
         }

         if (positionControlledJointsSet.contains(joint))
         {
            positionJointHandles.put(joint, createPositionJointHandle(joint));
         }

         if (!(torqueControlledJointsSet.contains(joint) || positionControlledJointsSet.contains(joint)))
         {
            jointStateHandles.put(joint, createJointStateHandle(joint));
         }
      }

      HashMap<String, IMUHandle> imuHandles = new HashMap<>();
      for (String imu : readIMUs)
      {
         if (USE_SWITCHABLE_FILTER_HOLDER_FOR_NON_USB_IMUS)
         {
            String complimentaryFilterHandleName = "CF" + imu;
            String kalmanFilterHandleName = "EF" + imu;
            imuHandles.put(complimentaryFilterHandleName, createIMUHandle(complimentaryFilterHandleName));
            imuHandles.put(kalmanFilterHandleName, createIMUHandle(kalmanFilterHandleName));
         }
         else
         {
            imuHandles.put(imu, createIMUHandle(imu));
         }
      }

      HashMap<String, ForceTorqueSensorHandle> forceTorqueSensorHandles = new HashMap<>();
      for (String ftSensorName : sensorInformation.getForceSensorNames())
      {
         ForceTorqueSensorHandle ftSensorHandle = createForceTorqueSensorHandle(ftSensorName);
         LogTools.info("Creating FT sensor handle for {}, exists: {}", ftSensorName, ftSensorHandle != null);
         forceTorqueSensorHandles.put(ftSensorName, ftSensorHandle);
      }

      /*
       * Create registries
       */

      /*
       * Create network servers/clients
       */
      PeriodicRealtimeThreadSchedulerFactory realtimeThreadFactory = new PeriodicRealtimeThreadSchedulerFactory(ValkyriePriorityParameters.POSECOMMUNICATOR_PRIORITY);
      RealtimeROS2Node estimatorRealtimeROS2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.FAST_RTPS,
                                                                                    realtimeThreadFactory,
                                                                                    VALKYRIE_IHMC_ROS_ESTIMATOR_NODE_NAME);
      RealtimeROS2Node controllerRealtimeROS2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.FAST_RTPS,
                                                                                     realtimeThreadFactory,
                                                                                     VALKYRIE_IHMC_ROS_CONTROLLER_NODE_NAME);
      LogModelProvider logModelProvider = robotModel.getLogModelProvider();
      DataServerSettings logSettings = robotModel.getLogSettings();
      double estimatorDT = robotModel.getEstimatorDT();
      yoVariableServer = new YoVariableServer(getClass(), logModelProvider, logSettings, estimatorDT);

      /*
       * Create sensors
       */

      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

      ValkyrieJointMap jointMap = robotModel.getJointMap();
      ValkyrieRosControlSensorReaderFactory sensorReaderFactory = new ValkyrieRosControlSensorReaderFactory(wallTimeProvider,
                                                                                                            monotonicTimeProvider,
                                                                                                            stateEstimatorParameters,
                                                                                                            effortJointHandles,
                                                                                                            positionJointHandles,
                                                                                                            jointStateHandles,
                                                                                                            imuHandles,
                                                                                                            forceTorqueSensorHandles,
                                                                                                            jointMap,
                                                                                                            sensorInformation);

      /*
       * Create controllers
       */
      HighLevelHumanoidControllerFactory controllerFactory = createHighLevelControllerFactory(robotModel, controllerRealtimeROS2Node, sensorInformation);
      DRCOutputProcessor drcOutputProcessor = null;

      if (USE_STATE_CHANGE_TORQUE_SMOOTHER_PROCESSOR)
      {
         DRCOutputProcessorWithStateChangeSmoother outputSmoother = new DRCOutputProcessorWithStateChangeSmoother(drcOutputProcessor);
         controllerFactory.attachControllerStateChangedListener(outputSmoother.createControllerStateChangedListener());
         drcOutputProcessor = outputSmoother;
      }

      PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber = null;
      externalPelvisPoseSubscriber = new PelvisPoseCorrectionCommunicator(null, null);
      ROS2Topic<?> controllerInputTopic = HumanoidControllerAPI.getInputTopic(robotName);
      estimatorRealtimeROS2Node.createSubscription(ControllerAPI.getTopic(controllerInputTopic, StampedPosePacket.class), externalPelvisPoseSubscriber);

      /*
       * Build controller
       */
      HumanoidRobotContextDataFactory estimatorContextDataFactory = new HumanoidRobotContextDataFactory();
      AvatarEstimatorThreadFactory avatarEstimatorThreadFactory = new AvatarEstimatorThreadFactory();
      avatarEstimatorThreadFactory.setROS2Info(estimatorRealtimeROS2Node, robotName);
      avatarEstimatorThreadFactory.configureWithDRCRobotModel(robotModel);
      avatarEstimatorThreadFactory.setSensorReaderFactory(sensorReaderFactory);
      avatarEstimatorThreadFactory.setHumanoidRobotContextDataFactory(estimatorContextDataFactory);
      avatarEstimatorThreadFactory.setGravity(gravity);
      estimatorThread = avatarEstimatorThreadFactory.createAvatarEstimatorThread();
      sensorReader = sensorReaderFactory.getSensorReader();
      sensorReader.skipWritingCommandsInRead(); // Indicate that we'll handle the write separately
      yoVariableServer.setMainRegistry(estimatorThread.getYoRegistry(),
                                       estimatorThread.getFullRobotModel().getElevator(),
                                       estimatorThread.getSCS1YoGraphicsListRegistry());

      // The estimator runs synchronous with the scheduler so its context is the master context.
      HumanoidRobotContextData masterContext = estimatorThread.getHumanoidRobotContextData();
      FullHumanoidRobotModel masterFullRobotModel = estimatorThread.getFullRobotModel();

      if (ENABLE_FINGER_JOINTS)
      {
         ValkyrieHandStateCommunicator handStateCommunicator = new ValkyrieHandStateCommunicator(robotName,
                                                                                                 estimatorThread.getFullRobotModel(),
                                                                                                 robotModel.getHandModel(),
                                                                                                 estimatorRealtimeROS2Node);
         estimatorThread.addRobotController(handStateCommunicator);
      }

      HumanoidRobotContextDataFactory controllerContextFactory = new HumanoidRobotContextDataFactory();
      AvatarControllerThread controllerThread = new AvatarControllerThread(robotModel.getSimpleRobotName(),
                                                                           robotModel,
                                                                           null,
                                                                           sensorInformation,
                                                                           controllerFactory,
                                                                           controllerContextFactory,
                                                                           drcOutputProcessor,
                                                                           controllerRealtimeROS2Node,
                                                                           gravity,
                                                                           false);
      if (!LOG_SECONDARY_HIGH_LEVEL_STATES)
         detachSecondaryRegistries(controllerThread.getYoVariableRegistry());

      int controllerDivisor = (int) Math.round(robotModel.getControllerDT() / robotModel.getEstimatorDT());
      if (!Precision.equals(robotModel.getControllerDT() / robotModel.getEstimatorDT(), controllerDivisor))
         throw new RuntimeException("Controller DT must be multiple of estimator DT.");
      ControllerTask controllerTask = new ControllerTask("Controller", controllerThread, controllerDivisor, robotModel.getEstimatorDT(), masterFullRobotModel);
      controllerTask.addCallbackPostTask(BarrierSchedulerTools.createProcessorUpdater(drcOutputProcessor, controllerThread));
      yoVariableServer.addRegistry(controllerThread.getYoVariableRegistry(), controllerThread.getSCS1YoGraphicsListRegistry());
      controllerTask.addCallbackPostTask(() -> yoVariableServer.update(controllerThread.getHumanoidRobotContextData().getTimestamp(),
                                                                       controllerThread.getYoVariableRegistry()));

      ValkyrieCalibrationControllerState calibrationControllerState = calibrationStateFactory.getCalibrationControllerState();
      calibrationControllerState.attachForceSensorCalibrationModule(estimatorThread.getForceSensorCalibrationModule());

      // Attaching listeners. Need to double check any possible threading issues here. Might be fine because we're using the StatusMessageOutputManager.
      sensorReaderFactory.attachControllerAPI(controllerFactory.getStatusOutputManager());
      sensorReaderFactory.attachJointTorqueOffsetEstimator(calibrationControllerState.getJointTorqueOffsetEstimatorController());
      sensorReaderFactory.setupLowLevelControlCommunication(robotName, estimatorRealtimeROS2Node);

      /*
       * Setup and start the JVM memory statistics
       */

      PeriodicRealtimeThreadSchedulerFactory schedulerFactory = new PeriodicRealtimeThreadSchedulerFactory(ValkyriePriorityParameters.JVM_STATISTICS_PRIORITY);
      JVMStatisticsGenerator jvmStatisticsGenerator = new JVMStatisticsGenerator(yoVariableServer, schedulerFactory);
      jvmStatisticsGenerator.addVariablesToStatisticsGenerator(yoVariableServer);

      List<HumanoidRobotControlTask> tasks = Arrays.asList(controllerTask);

      if (isGazebo)
      {
         LogTools.info("Running with blocking synchronous execution between estimator and controller");
         robotController = new SingleThreadedRobotController<>(robotName, tasks, masterContext);
      }
      else
      {
         LogTools.info("Running multi-threaded.");
         PriorityParameters controllerPriority = ValkyriePriorityParameters.CONTROLLER_PRIORITY;
         RealtimeThread controllerRealtimeThread = new RealtimeThread(controllerPriority, controllerTask, controllerTask.getClass().getSimpleName() + "Thread");
         robotController = new BarrierScheduledRobotController(robotName,
                                                               tasks,
                                                               masterContext,
                                                               TaskOverrunBehavior.SKIP_SCHEDULER_TICK,
                                                               robotModel.getEstimatorDT());
         if (valkyrieAffinity.setAffinity())
         {
            controllerRealtimeThread.setAffinity(valkyrieAffinity.getControlThreadProcessor());
         }
         controllerRealtimeThread.start();
      }
      estimatorThread.getYoRegistry().addChild(robotController.getYoRegistry());

      /*
       * Connect all servers
       */
      robotController.initialize();
      jvmStatisticsGenerator.start();
      estimatorRealtimeROS2Node.spin();
      controllerRealtimeROS2Node.spin();
      yoVariableServer.start();
   }

   private void detachSecondaryRegistries(YoRegistry drcControllerThreadRegistry)
   {
      YoRegistry drcMomentumBasedControllerRegistry = findChild(drcControllerThreadRegistry, "DRCMomentumBasedController");
      YoRegistry humanoidHighLevelControllerManagerRegistry = findChild(drcMomentumBasedControllerRegistry, "HumanoidHighLevelControllerManager");

      removeChild(humanoidHighLevelControllerManagerRegistry, "ValkyrieCalibrationControllerState");
      removeChild(humanoidHighLevelControllerManagerRegistry, "StandPrepControllerState");
      removeChild(humanoidHighLevelControllerManagerRegistry, "StandReadyControllerState");
      removeChild(humanoidHighLevelControllerManagerRegistry, "toWalkingSmoothTransitionControllerState");
      removeChild(humanoidHighLevelControllerManagerRegistry, "exitWalkingSmoothTransitionControllerState");
      removeChild(humanoidHighLevelControllerManagerRegistry, "LowLevelOneDoFJointDesiredDataHumanoidHighLevelControllerManager");
   }

   private static YoRegistry findChild(YoRegistry parent, String childName)
   {
      return parent.getChildren().stream().filter(child -> child.getName().equals(childName)).findFirst().get();
   }

   private static void removeChild(YoRegistry parent, String nameOfChildToRemove)
   {
      parent.getChildren().remove(findChild(parent, nameOfChildToRemove));
   }

   private void processEnvironmentVariables()
   {
      String isGazeboEnvironmentVariable = System.getenv("IS_GAZEBO");
      isGazebo = false;
      if (isGazeboEnvironmentVariable != null)
      {
         switch (isGazeboEnvironmentVariable)
         {
            case "true":
               isGazebo = true;
               break;
            default:
               isGazebo = false;
               break;
         }
      }
   }

   @Override
   protected void doControl(long rosTime, long duration)
   {
      if (firstTick)
      {
         if (valkyrieAffinity.setAffinity())
         {
            System.out.println("Setting estimator thread affinity to processor " + valkyrieAffinity.getEstimatorThreadProcessor().getId());
            Affinity.setAffinity(valkyrieAffinity.getEstimatorThreadProcessor());
         }
         firstTick = false;
      }

      wallTimeProvider.setTimestamp(rosTime);
      // Read sensor data from robot before running the controller so it gets the newest data.
      HumanoidRobotContextData masterContext = estimatorThread.getHumanoidRobotContextData();
      long newTimestamp = sensorReader.read(masterContext.getSensorDataContext());
      masterContext.setTimestamp(newTimestamp);
      // Run the estimator
      estimatorThread.run();

      // Run barrier scheduler:
      // Doing this after the estimator allows the controller to get newest data
      // Doing this before writing command to the robot allows to get the robot with the newest commands
      robotController.doControl();

      // Finally write the commands to the robot.
      sensorReader.writeCommandsToRobot();
      yoVariableServer.update(masterContext.getTimestamp(), estimatorThread.getYoRegistry());
   }
}
