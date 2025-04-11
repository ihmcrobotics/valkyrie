package us.ihmc.valkyrie.visualizer;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionChangeListener;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.valkyrie.ValkyriePoseSender;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.concurrent.atomic.AtomicBoolean;

public class ValkyrieSessionVisualizer
{
   public ValkyrieSessionVisualizer()
   {
      ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRobotVersion.ARM_MASS_SIM);
      ROS2Node ros2Node = new ROS2NodeBuilder().build("dynamic_bracing_simulation");
      ValkyriePoseSender sender = new ValkyriePoseSender(ros2Node, valkyrieRobotModel.getSimpleRobotName(), valkyrieRobotModel);

      AtomicBoolean loopPosesAtomic = new AtomicBoolean(false);
      AtomicBoolean goHomeAtomic = new AtomicBoolean(false);
      AtomicBoolean sendStepsAtomic = new AtomicBoolean(false);

      SessionVisualizer sessionVisualizer = SessionVisualizer.startSessionVisualizerExpert(null, true);
      sessionVisualizer.getSessionVisualizerControls().addSessionChangedListener(new SessionChangeListener()
      {
         @Override
         public void sessionChanged(Session previousSession, Session newSession)
         {
            if (newSession == null)
               return;

            YoRegistry registry = newSession.getRootRegistry();

            YoBoolean loopPoses = new YoBoolean("loopPoses", registry);
            loopPoses.addListener(v ->
                                                {
                                                   loopPosesAtomic.set(loopPoses.getValue());
                                                });
            YoBoolean goHome = new YoBoolean("goHome", registry);
            goHome.addListener(v ->
                                  {
                                     goHomeAtomic.set(goHome.getValue());
                                     goHome.set(false, false);
                                  });

            YoBoolean sendSteps = new YoBoolean("sendSteps", registry);
            sendSteps.addListener(v ->
                               {
                                  sendStepsAtomic.set(sendSteps.getValue());
                               });
         }
      });

      new Thread(() ->
                 {
                    while (true)
                    {
                       if (loopPosesAtomic.get())
                       {
                          int pauseTime = 5000;

                          sender.sendHomePose();
                          if (!loopPosesAtomic.get())
                             continue;
                          ThreadTools.sleep(pauseTime);

                          sender.sendArmsUp();
                          if (!loopPosesAtomic.get())
                             continue;
                          ThreadTools.sleep(pauseTime);

                          sender.sendArmsDown();
                          if (!loopPosesAtomic.get())
                             continue;
                          ThreadTools.sleep(pauseTime);

                          sender.sendLookLeft();
                          if (!loopPosesAtomic.get())
                             continue;
                          ThreadTools.sleep(pauseTime);

                          sender.sendLookRight();
                          if (!loopPosesAtomic.get())
                             continue;
                          ThreadTools.sleep(pauseTime);

                          sender.sendHomePose();
                          if (!loopPosesAtomic.get())
                             continue;
                          ThreadTools.sleep(pauseTime);

                          if (sendStepsAtomic.get())
                          {
                             sender.sendSteps();
                             if (!loopPosesAtomic.get())
                                continue;
                             ThreadTools.sleep(pauseTime);
                          }
                       }
                       else
                       {
                          if (goHomeAtomic.getAndSet(false))
                             sender.sendHomePose();
                          ThreadTools.sleep(3000);
                       }
                    }
                 }).start();
   }

   public static void main(String[] args)
   {
      new ValkyrieSessionVisualizer();
   }
}
