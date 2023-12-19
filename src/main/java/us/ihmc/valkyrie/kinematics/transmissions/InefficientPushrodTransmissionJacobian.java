package us.ihmc.valkyrie.kinematics.transmissions;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;

public class InefficientPushrodTransmissionJacobian implements PushrodTransmissionJacobian
{
   private final boolean DEBUG = false;

   private static final double DEGREES = Math.PI/180.0;

   private Axis3D topJointAxis, bottomJointAxis;
   
   private double heightOfTopAxisAboveBottomAxis;    // meters (m)

   private double futekLength;    // futek link length (m)
   private double futekLengthSquared;

   private final Vector3D rod5PointInBoneFrame = new Vector3D();    // position where rod 5 passes through bone frame plane. x is forward. y is to the left. z is up. (m)
   private final Vector3D rod6PointInBoneFrame = new Vector3D();    // position where rod 6 passes through bone frame plane. x is forward. y is to the left. z is up. (m)
   private double actuatorSlider5PitchRotation; // actuator slider 5 pitch angle
   private double actuatorSlider6PitchRotation; // actuator slider 6 pitch angle
   
   private final Vector3D rodBottom5 = new Vector3D();  // position vector of futek link base for actuator 5 side in foot frame (m)
   private final Vector3D rodBottom6 = new Vector3D();   // position vector of futek link base for actuator 6 side in foot frame (m)
   
   private boolean useFuteks = true; 
   
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final TranslationReferenceFrame topFrame = new TranslationReferenceFrame("topFrame", worldFrame);
   private final TransformReferenceFrame actuator5SlideFrame = new TransformReferenceFrame("actuator5SlideFrame", topFrame);
   private final TransformReferenceFrame actuator6SlideFrame = new TransformReferenceFrame("actuator6SlideFrame", topFrame);
   private final TransformReferenceFrame afterTopJointFrame = new TransformReferenceFrame("afterTopJointFrame", topFrame);
   private final TranslationReferenceFrame beforeBottomJointFrame = new TranslationReferenceFrame("beforeBottomJointFrame", afterTopJointFrame);
   private final TransformReferenceFrame bottomFrame = new TransformReferenceFrame("bottomFrame", beforeBottomJointFrame);

   private final RigidBodyTransform topJointTransform3D = new RigidBodyTransform();
   private final RigidBodyTransform bottomJointTransform3D = new RigidBodyTransform();

   private final FramePoint3D b5InBottomFrame = new FramePoint3D();   
   private final FramePoint3D b6InBottomFrame = new FramePoint3D();   

   private final FramePoint3D b5InTopFrame = new FramePoint3D();
   private final FramePoint3D b6InTopFrame = new FramePoint3D();
   
   private final FramePoint3D b5InSlideFrame = new FramePoint3D();
   private final FramePoint3D b6InSlideFrame = new FramePoint3D();

   private final FramePoint3D t5InTopFrame = new FramePoint3D();
   private final FramePoint3D t6InTopFrame = new FramePoint3D();
   
   private final FramePoint3D t5InBottomFrame = new FramePoint3D();
   private final FramePoint3D t6InBottomFrame = new FramePoint3D();

   private final FrameVector3D f5VectorInTopFrame = new FrameVector3D(topFrame);
   private final FrameVector3D f6VectorInTopFrame = new FrameVector3D(topFrame);

   private final FrameVector3D f5VectorInBottomFrame = new FrameVector3D(bottomFrame);
   private final FrameVector3D f6VectorInBottomFrame = new FrameVector3D(bottomFrame);

   private final FrameVector3D tempRVector = new FrameVector3D();
   private final FrameVector3D tempCrossVector = new FrameVector3D();

   private boolean visualize = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble jTopJoint5 = new YoDouble("jTopJoint5", registry);
   private final YoDouble jTopJoint6 = new YoDouble("jTopJoint6", registry);
   private final YoDouble jBottomJoint5 = new YoDouble("jBottomJoint5", registry);
   private final YoDouble jBottomJoint6 = new YoDouble("jBottomJoint6", registry);

   private final YoGraphicPosition b5Viz, b6Viz, t5Viz, t6Viz;
   
   private final YoGraphicReferenceFrame actuator5SlideFrameViz, actuator6SlideFrameViz;
   private final YoGraphicReferenceFrame topFrameViz, afterTopJointFrameViz, beforeBottomJointFrameViz, bottomFrameViz;

   public InefficientPushrodTransmissionJacobian(PushRodTransmissionJoint pushRodTransmissionJoint, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      switch (pushRodTransmissionJoint)
      {
      case ANKLE:
      {
         setupForAnkleActuators();
         break;
      }
      case WAIST:
      {
         setupForWaistActuators();
         break;
      }
      case WRIST:
      {
         setupForWristActuators();
         break;
      }
      }
      
      topFrame.updateTranslation(new FrameVector3D(worldFrame, 0.0, 0.0, 1.0));    // Arbitrary. Just put it in the air. If we wanted to have things align with the real robot, then this should be at the top joint frame.
      
      RigidBodyTransform transformFromActuatorSlide5FrameToBoneFrame = new RigidBodyTransform();      
      transformFromActuatorSlide5FrameToBoneFrame.setRotationPitchAndZeroTranslation(-actuatorSlider5PitchRotation);
      transformFromActuatorSlide5FrameToBoneFrame.getTranslation().set(new Vector3D(rod5PointInBoneFrame));
      
      RigidBodyTransform transformFromActuatorSlide6FrameToBoneFrame = new RigidBodyTransform();      
      transformFromActuatorSlide6FrameToBoneFrame.setRotationPitchAndZeroTranslation(-actuatorSlider6PitchRotation);
      transformFromActuatorSlide6FrameToBoneFrame.getTranslation().set(new Vector3D(rod6PointInBoneFrame));

      actuator5SlideFrame.setTransformAndUpdate(transformFromActuatorSlide5FrameToBoneFrame);
      actuator6SlideFrame.setTransformAndUpdate(transformFromActuatorSlide6FrameToBoneFrame);
      
      beforeBottomJointFrame.updateTranslation(new FrameVector3D(afterTopJointFrame, 0.0, 0.0, -heightOfTopAxisAboveBottomAxis));

      if (yoGraphicsListRegistry == null)
      {
         visualize = false;
      }

      if (visualize)
      {
         YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());

         double ballRadius = 0.01;

         t5Viz = new YoGraphicPosition("t5Viz", "", registry, ballRadius, YoAppearance.Blue());
         t6Viz = new YoGraphicPosition("t6Viz", "", registry, ballRadius, YoAppearance.Green());

         b5Viz = new YoGraphicPosition("b5Viz", "", registry, ballRadius, YoAppearance.Red());
         b6Viz = new YoGraphicPosition("b6Viz", "", registry, ballRadius, YoAppearance.Black());

         double frameScale = 0.1;

         topFrameViz = new YoGraphicReferenceFrame(topFrame, registry, false, frameScale);
         afterTopJointFrameViz = new YoGraphicReferenceFrame(afterTopJointFrame, registry, false, frameScale * 0.8);
         beforeBottomJointFrameViz = new YoGraphicReferenceFrame(beforeBottomJointFrame, registry, false, frameScale * 0.6);
         bottomFrameViz = new YoGraphicReferenceFrame(bottomFrame, registry, false, frameScale * 0.4);

         actuator5SlideFrameViz = new YoGraphicReferenceFrame(actuator5SlideFrame, registry, false, frameScale * 0.6);
         actuator6SlideFrameViz = new YoGraphicReferenceFrame(actuator6SlideFrame, registry, false, frameScale * 0.6);
         
         yoGraphicsList.add(b5Viz);
         yoGraphicsList.add(b6Viz);
         yoGraphicsList.add(t5Viz);
         yoGraphicsList.add(t6Viz);

         yoGraphicsList.add(topFrameViz);
         yoGraphicsList.add(afterTopJointFrameViz);
         yoGraphicsList.add(beforeBottomJointFrameViz);
         yoGraphicsList.add(bottomFrameViz);
         
         yoGraphicsList.add(actuator5SlideFrameViz);
         yoGraphicsList.add(actuator6SlideFrameViz);

         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      }
      else
      {
         b5Viz = b6Viz = t5Viz = t6Viz = null;
         topFrameViz = afterTopJointFrameViz = beforeBottomJointFrameViz = bottomFrameViz = null;
         actuator5SlideFrameViz = actuator6SlideFrameViz = null;
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }
   
   public void setupForAnkleActuators()
   {
      topJointAxis = Axis3D.Y; // Pitch.
      bottomJointAxis = Axis3D.X; // Roll.
      
      heightOfTopAxisAboveBottomAxis = 0.0127;

      futekLength = 0.1049655;
      futekLengthSquared = futekLength * futekLength;

      rod5PointInBoneFrame.set(-0.0215689, -0.04128855, 0.05);  // z is arbitrary since already aligned in z.  
      rod6PointInBoneFrame.set(-0.0215689, 0.04128855, 0.05);  // z is arbitrary since already aligned in z.  
      actuatorSlider5PitchRotation = 0.0;
      actuatorSlider6PitchRotation = 0.0;
      
      rodBottom5.set(-0.0364, -0.0355, 0.0176);     
      rodBottom6.set(-0.0364, 0.0355, 0.0176);         
   }
   
   public void setupForWaistActuators()
   {
      topJointAxis = Axis3D.X; // Roll.
      bottomJointAxis = Axis3D.Y; // Pitch.
      
      heightOfTopAxisAboveBottomAxis = 0.02032;

//      futekLength = 0.1310005;
      futekLength = 0.131;
      futekLengthSquared = futekLength * futekLength;

//      rod5PointInBoneFrame.set(-0.00598410, -0.06985123, 0.08861994);    
//      rod6PointInBoneFrame.set(-0.00598410, 0.06985123, 0.08861994); 
      rod5PointInBoneFrame.set(-0.005984234, -0.06985124, 0.08861974);    
      rod6PointInBoneFrame.set(-0.005984234, 0.06985124, 0.08861974);    

      actuatorSlider5PitchRotation = -24.0001 * DEGREES;
      actuatorSlider6PitchRotation = -24.0001 * DEGREES;
      
//      rodBottom5.set(-0.0762, -0.0508, 0.0);     
//      rodBottom6.set(-0.0762, 0.0508, 0.0);  
      rodBottom5.set(-0.0762013, -0.0508029, 0.0);
      rodBottom6.set(-0.0762013, 0.0508029, 0.0);       

   }
   
   public void setupForWristActuators()
   {
      // TODO: Add Wrist parameters. Right now they are ankle parameters.
      heightOfTopAxisAboveBottomAxis = 0.0127;

      futekLength = 0.1049655;
      futekLengthSquared = futekLength * futekLength;

      rod5PointInBoneFrame.set(-0.0215689, -0.04128855, 0.0);    
      rod6PointInBoneFrame.set(-0.0215689, 0.04128855, 0.0);     
      actuatorSlider5PitchRotation = 0.0;
      actuatorSlider6PitchRotation = 0.0;
      
      rodBottom5.set(-0.0364, -0.0355, 0.0176);     
      rodBottom6.set(-0.0364, 0.0355, 0.0176);         
   }

   public void setUseFuteks(boolean useFuteks)
   {
      this.useFuteks = useFuteks;
   }
   
   public void computeJacobian(double[][] jacobianToPack, double topJointAngle, double bottomJointAngle)
   {
      // Update forward kinematics reference frames using roll and pitch.
      computeRotationTransform(topJointTransform3D, topJointAngle, topJointAxis);
      computeRotationTransform(bottomJointTransform3D, bottomJointAngle, bottomJointAxis);

      afterTopJointFrame.setTransformAndUpdate(topJointTransform3D);
      bottomFrame.setTransformAndUpdate(bottomJointTransform3D);

      b5InBottomFrame.setIncludingFrame(bottomFrame, rodBottom5);   
      b6InBottomFrame.setIncludingFrame(bottomFrame, rodBottom6);   
      
      b5InTopFrame.setIncludingFrame(b5InBottomFrame);
      b5InTopFrame.changeFrame(topFrame);

      b6InTopFrame.setIncludingFrame(b6InBottomFrame);
      b6InTopFrame.changeFrame(topFrame);
      
      b5InSlideFrame.setIncludingFrame(b5InBottomFrame);
      b5InSlideFrame.changeFrame(actuator5SlideFrame);

      b6InSlideFrame.setIncludingFrame(b6InBottomFrame);
      b6InSlideFrame.changeFrame(actuator6SlideFrame);

      if(DEBUG)
      {
         System.out.println("b5InBoneFrame = " + b5InTopFrame);
         System.out.println("b6InBoneFrame = " + b6InTopFrame);         
      }

      // Solve for t5, t6 in bone frame:
      
      double xDiff = b5InSlideFrame.getX();
      double yDiff = b5InSlideFrame.getY();
      double t5zInSlideFrame = b5InSlideFrame.getZ() + Math.sqrt(futekLengthSquared - xDiff * xDiff - yDiff * yDiff);

      xDiff = b6InSlideFrame.getX();
      yDiff = b6InSlideFrame.getY();
      double t6zInSlideFrame = b6InSlideFrame.getZ() + Math.sqrt(futekLengthSquared - xDiff * xDiff - yDiff * yDiff);

      if(DEBUG)
      {
         System.out.println("t5zInSlideFrame = " + t5zInSlideFrame);
         System.out.println("t6zInSlideFrame = " + t6zInSlideFrame);
      }

      t5InTopFrame.setToZero(actuator5SlideFrame);
      t6InTopFrame.setToZero(actuator6SlideFrame);

      t5InTopFrame.setZ(t5zInSlideFrame);
      t6InTopFrame.setZ(t6zInSlideFrame);

      t5InTopFrame.changeFrame(topFrame);
      t6InTopFrame.changeFrame(topFrame);

      if(DEBUG)
      {
         System.out.println("t5InBoneFrame = " + t5InTopFrame);
         System.out.println("t6InBoneFrame = " + t6InTopFrame);
      }
      // Compute topsInFootFrame:
      
      t5InBottomFrame.setIncludingFrame(t5InTopFrame);
      t6InBottomFrame.setIncludingFrame(t6InTopFrame);
      
      t5InBottomFrame.changeFrame(bottomFrame);
      t6InBottomFrame.changeFrame(bottomFrame);
      
      if (DEBUG)
      {
         System.out.println("t5InFootFrame = " + t5InBottomFrame);
         System.out.println("t6InFootFrame = " + t6InBottomFrame);
      }


      // Do R cross F to get Jacobian elements:

      if (useFuteks)
      {
         f5VectorInTopFrame.sub(b5InTopFrame, t5InTopFrame);
         f6VectorInTopFrame.sub(b6InTopFrame, t6InTopFrame);

         f5VectorInTopFrame.normalize();
         f6VectorInTopFrame.normalize();
         
         if (DEBUG)
         {
            System.out.println("f5VectorInBoneFrame = " + f5VectorInTopFrame);
            System.out.println("f6VectorInBoneFrame = " + f6VectorInTopFrame);            
         }
      }
      else
      {
         f5VectorInTopFrame.setIncludingFrame(actuator5SlideFrame, 0.0, 0.0, -1.0);
         f6VectorInTopFrame.setIncludingFrame(actuator6SlideFrame, 0.0, 0.0, -1.0);
         
         f5VectorInTopFrame.changeFrame(topFrame);
         f6VectorInTopFrame.changeFrame(topFrame);
      }
      
      f5VectorInBottomFrame.setIncludingFrame(f5VectorInTopFrame);
      f5VectorInBottomFrame.changeFrame(bottomFrame);
      f6VectorInBottomFrame.setIncludingFrame(f6VectorInTopFrame);
      f6VectorInBottomFrame.changeFrame(bottomFrame);

      tempRVector.setIncludingFrame(t5InTopFrame); //kjb5InBoneFrame);
      tempCrossVector.setToZero(tempRVector.getReferenceFrame());
      tempCrossVector.cross(tempRVector, f5VectorInTopFrame);
      setJacobianElement(jTopJoint5, tempCrossVector, topJointAxis);

      tempRVector.setIncludingFrame(t6InTopFrame); //b6InBoneFrame);
      tempCrossVector.cross(tempRVector, f6VectorInTopFrame);
      setJacobianElement(jTopJoint6, tempCrossVector, topJointAxis);


      tempRVector.setIncludingFrame(t5InBottomFrame); //b5InFootFrame);
      tempCrossVector.setToZero(tempRVector.getReferenceFrame());
      tempCrossVector.cross(tempRVector, f5VectorInBottomFrame);
      setJacobianElement(jBottomJoint5, tempCrossVector, bottomJointAxis);

      tempRVector.setIncludingFrame(t6InBottomFrame); //b6InFootFrame);
      tempCrossVector.cross(tempRVector, f6VectorInBottomFrame);
      setJacobianElement(jBottomJoint6, tempCrossVector, bottomJointAxis);

      //
      //NOTE: this setup will only work for ankles
      jacobianToPack[0][0] =  jTopJoint6.getDoubleValue(); // top to left
      jacobianToPack[0][1] =  jTopJoint5.getDoubleValue(); // top to right
      jacobianToPack[1][0] = jBottomJoint6.getDoubleValue(); // bottom to left
      jacobianToPack[1][1] = jBottomJoint5.getDoubleValue(); // bottom to right

      if (visualize)
      {
         FramePoint3D t5InWorld = new FramePoint3D(t5InTopFrame);
         FramePoint3D t6InWorld = new FramePoint3D(t6InTopFrame);
         t5InWorld.changeFrame(worldFrame);
         t6InWorld.changeFrame(worldFrame);
         t5Viz.setPosition(t5InWorld);
         t6Viz.setPosition(t6InWorld);

         FramePoint3D b5InWorld = new FramePoint3D(b5InBottomFrame);
         FramePoint3D b6InWorld = new FramePoint3D(b6InBottomFrame);
         b5InWorld.changeFrame(worldFrame);
         b6InWorld.changeFrame(worldFrame);
         b5Viz.setPosition(b5InWorld);
         b6Viz.setPosition(b6InWorld);

         topFrameViz.update();
         afterTopJointFrameViz.update();
         beforeBottomJointFrameViz.update();
         bottomFrameViz.update();

         actuator5SlideFrameViz.update();
         actuator6SlideFrameViz.update();
      }

   }
   
   private void setJacobianElement(YoDouble jacobianElement, FrameVector3D rCrossFVector, Axis3D jointAxis)
   {
      if (jointAxis == Axis3D.X)
      {
         jacobianElement.set(rCrossFVector.getX());
      }
      else if (jointAxis == Axis3D.Y)
      {
         jacobianElement.set(rCrossFVector.getY());
      }
      else
      {
         throw new RuntimeException("Shouldn't get here!");
      }
   }

   private static void computeRotationTransform(RigidBodyTransform transform3DToPack, double rotationAngle, Axis3D rotationAxis)
   {
      transform3DToPack.setIdentity();
      switch(rotationAxis)
      {
      case X:
      {
         transform3DToPack.setRotationRollAndZeroTranslation(rotationAngle);
         break;
      }
      case Y:
      {
         transform3DToPack.setRotationPitchAndZeroTranslation(rotationAngle);
         break;
      }
      case Z:
      {
         transform3DToPack.setRotationYawAndZeroTranslation(rotationAngle);
         break;
      }
      default: 
      {
         throw new RuntimeException("Shouldn't get here.");
      }
      }
   }

}
