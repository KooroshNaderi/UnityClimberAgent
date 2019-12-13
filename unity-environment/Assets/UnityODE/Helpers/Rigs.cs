using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;
using AaltoGames;

namespace AaltoGames
{
    /* The Rig class abstracts physics character manipulation complexity, providing features such as
    * - building a physics rig, i.e., adding physics components to a Mecanim character
    * - setting and getting of motor angle ranges
    * - converting a character pose to motor angles and back (useful for setting up initial conditions and dimension reduction)
    * - setting motor parameters such as maximum force for all motor or groups (torso, head, arms, legs) (useful for controlling relaxedness of the character)
    * - setting motor drive parameters so that stepping the simulation will drive the body towards desired motor angles or pose (useful for simulating a given motor space trajectory when optimizing)
    * 
    * Example use cases:
    * Optimization:
    * - add rig component to mecanim character
    * - get motor parameter range from rig
    * - initialize optimizer for optimizing splines where each control point is a vector of motor parameters and position along spline corresponds to time
    * - get spline samples (vector of control points and time values), evaluate them to get motor target angles for each physics step. Use the rig to drive the motors
    * - evaluate the fitness of the samples based on how well the character achieved goals
    * 
    * Building a dimension reduction prior from animation
    * - add rig component to mecanim character
    * - play animations and for each frame, use the rig to obtain a vector of corresponding motor angles
    * - use the vectors to initialize dimension reduction
    * 
    */

    public class Rig
	{
		public Vector3 rootUp=Vector3.up;
		public Transform targetCharacter;
        Animator anim=null;
        public bool hingeAnkle = true;

		public List<AnimHandle> handles = new List<AnimHandle>();
	//	float feetY=0;
        bool constrainTo2d;
		public bool createPhysicsObjects=true;  //set before calling buildMecanimBipedRig
        public Dictionary<Transform, BoneData> startTransformToBoneData = new Dictionary<Transform, BoneData>();
        public Dictionary<HumanBodyBones, BoneData> mecanimBoneToBoneData = new Dictionary<HumanBodyBones, BoneData>();
        public Dictionary<int, BoneData> odeBodyIdToBoneData = new Dictionary<int, BoneData>();
        public Dictionary<Transform, BonelessTransformData> bonelessTransformDefaults = new Dictionary<Transform, BonelessTransformData>();
        const float humanDensity = 1000.0f; //kg/m3
        public float chestThicknessMul = 1;
        public float armsThicknessMul = 0.8f;
        public float neckThicknessMul = 1;
        public bool useBoxesForFeet = true;
        public float maxTargetVel = Mathf.PI*2.0f; //radians per seconds, applies to all motors. See driveMotorToTarget()
        //character orientation vectors, computed in buildMecanimBipedRig() assuming character in t-pose
        Vector3 tposeForward, tposeRight;
        public bool relaxedJointLimits = false;
        public bool extraHeadJoint = false;
        public class BoneData
		{
            public enum Category
            {
                LEGS=0,
                TORSO=1,
                LEFTARM=2,
                RIGHTARM=3,
                COUNT=4,
                UNKNOWN=-1
            }

            public BoneData parent=null;
            public HumanBodyBones mecanimBone;
            public OdeBody rb = null;
            public OdeGeom c = null;
            public int hingeAxis = 0;
            public string relativeControlPath = "";
            public Transform controlTransform = null;
            public GameObject controlObject = null;
            public Transform parentTransform = null;
			public OdeJoint joint=null;
            public OdeJoint motor=null;
            public OdeJointHinge hinge = null;
            public OdeJointAngularMotor amotor = null;
            public float FmaxScale = 1.0f;
            public int firstAngleIndex; //index of first angle of the motor in a vector of all motor angles (total getRigNumFreedoms() elements)
            public bool isMotored
            {
                get
                {
                    return motor != null;
                }
            }

			public Quaternion defaultRotation=Quaternion.identity; //rotation at the bind pose, i.e., when joints are instantiated. The joint target rotations must be computed relative to this
            public Vector3 defaultEulers;
            public Vector3 defaultJointLocalPosition;
            public Quaternion initialRotation = Quaternion.identity;

            public bool tmpIsLeg = false;
            public Category category = Category.UNKNOWN;

            public int index;
            public int numFreedoms = 3;
            public Vector3 boneVector;
            public Vector3 relativeEulers;
            public Transform[] transformHierarchyToParent;
            public Vector3[] localPositionHierarchyToParent;
            public Quaternion[] localRotationHierarchyToParent;
            public float thickness;
            public bool? isRightSide;
            public string name;
            public OdeJointDefinition jointDef;
            public bool isExtremePoint;
            public float cachedMass;
            public override string ToString()
            {
                return base.ToString() + "(rb: " + po(rb) + ", c:" + po(c) + ", hingeAxis: " + po(hingeAxis) + ", relativeControlPath: " + relativeControlPath + ", controlTransform: " + controlTransform +
                    ", joint: " + joint + ", motor: " + motor + ", bone: " + controlObject + ", defaultRotation: " + defaultRotation + ", defaultEulers: " + defaultEulers + ")";
            }

            private static string po(object o)
            {
                if (o == null)
                {
                    return "null";
                }

                return o.ToString();
            }

            public bool isLegSupportPart()
            {
                return mecanimBone==HumanBodyBones.LeftFoot || mecanimBone==HumanBodyBones.RightFoot;
            }
		}
        public class BonelessTransformData
        {
            public Vector3 localPosition;
            public Quaternion localRotation;
        }
        int [] nBonesPerCategory=new int[(int)BoneData.Category.COUNT];

		public List<BoneData> bones=new List<BoneData>();
        int numFreedoms = 0;
        public int GetRigNumFreedoms()
        {
            return numFreedoms;
        }
		public static string GetPath(Transform current) {
		    if (current.parent == null)
    	       	return "/" + current.name;
	    	return GetPath(current.parent) + "/" + current.name;
		}
		public static string GetRelativePath(Transform current, Transform relativeTo)
		{
			if (current.parent == relativeTo)
				return current.name;
		    if (current.parent == null)
	       	return "/" + current.name;
	    	return GetRelativePath(current.parent,relativeTo) + "/" + current.name;
		}
		public Rig()
		{}
		Transform findRecursive(Transform parent, string name)
	    {
	        Transform result=parent.Find(name);
	        if (result!=null)
	            return result;
	        foreach(Transform t in parent){
	            Transform child = findRecursive(t, name);
	            if (child != null)
	                return child;
	        }
	        return null;
	    }
		Transform findRecursive(string name)
	    {
			return findRecursive(targetCharacter,name);
	    }

        private bool updateFromOdeVal = true;

        //a shortcut to disabling the fixedupdate of the ode joints and bodies of this rig
        public bool updateFromOde
        {
            set
            {
                if (value == updateFromOdeVal)
                {
                    return;
                }

                foreach (BoneData boneData in bones)
                {
                    if (boneData.rb != null)
                    {
                        boneData.rb.disableUpdate = !value;
                    }

                    if (boneData.joint != null)
                    {
                        boneData.joint.disableUpdate = !value;
                    }
                }

                updateFromOdeVal = value;
            }

            get
            {
                return updateFromOdeVal;
            }
        }
        //TODO: remove this, apparently only for testing
        public void TestAttachMass(BoneData boneData, float radius, float height, Vector3 translation)
        {
            GameObject massHolder = new GameObject();
            massHolder.name = "MassAttach_" + boneData.controlTransform.name;

            OdeGeomCapsule cc = massHolder.AddComponent<OdeGeomCapsule>();
            cc.radius = radius;
            cc.height = height;

            OdeBody rb = massHolder.AddComponent<OdeBody>();

            // These update position from Unity to ODE
            Vector3 position = boneData.controlTransform.position + translation + new Vector3(0.07f, 0, 0);
            //Debug.LogWarning("POS: " + position);
            rb.position = position;
            rb.updateTransfromFromPhysics();
            rb.rotation = Quaternion.identity;


            OdeJointDefinition holdingJointDef = OdeJointDefinition.CreateHinge();
            holdingJointDef.Create(massHolder.transform, boneData.rb, rb);
        }

        Dictionary<HumanBodyBones, BoneData.Category> boneCategories = new Dictionary<HumanBodyBones, BoneData.Category>
        {
            {HumanBodyBones.Chest,BoneData.Category.TORSO},
            {HumanBodyBones.Neck,BoneData.Category.TORSO},
            {HumanBodyBones.Head,BoneData.Category.TORSO},
            {HumanBodyBones.Hips,BoneData.Category.TORSO},
            {HumanBodyBones.LeftFoot,BoneData.Category.LEGS},
            {HumanBodyBones.LeftHand,BoneData.Category.LEFTARM},
            {HumanBodyBones.LeftLowerArm,BoneData.Category.LEFTARM},
            {HumanBodyBones.LeftLowerLeg,BoneData.Category.LEGS},
            {HumanBodyBones.LeftShoulder,BoneData.Category.LEFTARM},
            {HumanBodyBones.LeftUpperArm,BoneData.Category.LEFTARM},
            {HumanBodyBones.LeftUpperLeg,BoneData.Category.LEGS},
            {HumanBodyBones.RightFoot,BoneData.Category.LEGS},
            {HumanBodyBones.RightHand,BoneData.Category.RIGHTARM},
            {HumanBodyBones.RightLowerArm,BoneData.Category.RIGHTARM},
            {HumanBodyBones.RightLowerLeg,BoneData.Category.LEGS},
            {HumanBodyBones.RightShoulder,BoneData.Category.RIGHTARM},
            {HumanBodyBones.RightUpperArm,BoneData.Category.RIGHTARM},
            {HumanBodyBones.RightUpperLeg,BoneData.Category.LEGS},
            {HumanBodyBones.Spine,BoneData.Category.TORSO}
        };
        /*
        HumanBodyBones[] extremePoints =
        {
            HumanBodyBones.Head,
            HumanBodyBones.LeftFoot,
            HumanBodyBones.LeftHand,
            HumanBodyBones.RightFoot,
            HumanBodyBones.RightHand
        };
        */
        HumanBodyBones boneTransformToHumanBodyBone(Transform boneTransform)
        {
            foreach (var b in Enum.GetValues(typeof(HumanBodyBones)))
            {
                if (anim.GetBoneTransform((HumanBodyBones)b) == boneTransform)
                {
                    return (HumanBodyBones)b;
                }
            }
            Debug.LogException(new Exception("Could not map bone transform " + boneTransform.name + " to a mecanim HumanBodyBones enum!"));
            return HumanBodyBones.LastBone;
        }
        private  BoneData.Category getHierarchyCategory(HumanBodyBones bone, out bool? isRightSide, out bool isExtremePoint)
        {
            string boneName = bone.ToString();
            isExtremePoint = false;
            if (boneName.Contains("Foot") || boneName.Contains("LowerArm") || boneName.Contains("Neck") || boneName.Contains("Head"))
            {
                isExtremePoint = true;
            }
            isRightSide=boneName.Contains("Right");
            BoneData.Category category=BoneData.Category.UNKNOWN;
            if (boneCategories.ContainsKey(bone))
                category = boneCategories[bone];
            else
                Debug.LogException(new Exception("Could not determine category for bone " + bone));
            return category;
        }
        Vector3[] basisVectors = new Vector3[] { Vector3.right, Vector3.up, Vector3.forward, -Vector3.right, -Vector3.up, -Vector3.forward };
        int getClosestLocalAxisIdx(Transform transform, Vector3 target)
        {
            float maxProj = -10;
            int result = -1;
            for (int i=0; i<6; i++)
            {
                Vector3 rotated = transform.TransformDirection(basisVectors[i]);
                float proj = Vector3.Dot(rotated, target);
                if (proj > maxProj)
                {
                    result = i;
                    maxProj = proj;
                }
            }
            return result;
        }
        int getBipedHingeLocalAxisIdx(HumanBodyBones boneName)
        {
            BoneData data = startTransformToBoneData[anim.GetBoneTransform(boneName)];
            Vector3 desiredHingeVector;
            //All biped bones with hinge joints except feet seem to follow the same logic, assuming that one of the local axes points forward and one along the bone.
            //Feet bones are different since they are the only ones pointing forward in the t-pose
            if (boneName == HumanBodyBones.LeftFoot || boneName == HumanBodyBones.RightFoot)
            {

                desiredHingeVector = Vector3.Cross(Vector3.up, data.boneVector.normalized);
            }
            else
            {
                desiredHingeVector = Vector3.Cross(tposeForward, data.boneVector.normalized);
            }
            return getClosestLocalAxisIdx(data.controlTransform, desiredHingeVector);
        }
        Vector3 getBipedHingeLocalAxis(HumanBodyBones boneName)
        {
            return basisVectors[getBipedHingeLocalAxisIdx(boneName)];
        }
        //adds a bone (capsule). Also adds a joint if parent node already has a bone
        void AddBone(Transform startTransform, Vector3 boneEndPos, Transform parentTransform, OdeJointDefinition def, bool addHandle)
        {
            bool isFoot = (startTransform == anim.GetBoneTransform(HumanBodyBones.RightFoot)
                || startTransform == anim.GetBoneTransform(HumanBodyBones.LeftFoot));
            bool isHead=startTransform == anim.GetBoneTransform(HumanBodyBones.Neck);
            bool isHand = (startTransform == anim.GetBoneTransform(HumanBodyBones.RightHand)
                || startTransform == anim.GetBoneTransform(HumanBodyBones.LeftHand));
            //The physics bone controls the startTransform 
            GameObject controlObject = startTransform.gameObject;

            //Debug.Log("Joint " + startTransform.name + " is " + category + " parent " + parentTransform + " end " + endTransform);

            //Some bookkeeping. All relevant data is stored in a BoneData instance, and 
            //the startTransformToBoneData dictionary can be used to look up BoneData for a start transform.
            //Example: for a forearm bone, startTransform's origin is at elbow, endTransform's origin at wrist, and parentTransform's origin at shoulder.
            BoneData data = new BoneData();
            startTransformToBoneData.Add(startTransform, data);

            //Try to figure out the category (PERTTUTODO: this should work more reliably comparing the transform to 
            //mecanim's HumanBodyBone helpers
            bool? isRightSide = null;
            bool isExtremePoint = false;

            data.mecanimBone = boneTransformToHumanBodyBone(startTransform);
            data.category = getHierarchyCategory(data.mecanimBone, out isRightSide, out isExtremePoint);
            nBonesPerCategory[(int)data.category]++;
            data.isRightSide = isRightSide;
            data.isExtremePoint = isExtremePoint;
            mecanimBoneToBoneData.Add(data.mecanimBone,data);


            Debug.Log(startTransform.name + " is of category " + data.category);

            /*if ((boneEndPos-startTransform.position).magnitude<0.1f)
                boneEndPos=startTransform.position+(boneEndPos-startTransform.position).normalized*0.1f; //ensure reasonable minimum length for easier manipulation*/
            //bone.transform.position = startTransform.position + 0.5f * (boneEndPos - boneStartPos);
            float thickness = 0.1f;

            //TODO: thickness should not depend on force control category -> 
            //change this to make the decisions simply using the mecanim bone names and e.g., Transform.isChildOf
            switch (data.category)
            {
                case BoneData.Category.TORSO:
                    if (data.mecanimBone==HumanBodyBones.Neck)
                        thickness *= neckThicknessMul;
                    else
                        thickness *= chestThicknessMul;
                    break;
                case BoneData.Category.LEFTARM:
                    thickness *= armsThicknessMul;
                    break;
                case BoneData.Category.RIGHTARM:
                    thickness *= armsThicknessMul;
                    break;
            }


            //The bone will be represented using a physics capsule. We'll now figure out the
            //capsule's dimensions and orientation.
            Vector3 boneStartPos = startTransform.position;
            if (isFoot)
            {
                thickness *= 0.7f;
                //hack: in most biped rigs, the foot (ankle) transform is higher than the toe, but we want a horizontal foot bone in the T-pose
                boneEndPos.y += thickness * 0.5f;
                boneEndPos.y += 0.01f;
                boneStartPos.y = boneEndPos.y;
                boneStartPos += tposeForward * 0.02f;
                boneEndPos += tposeForward * 0.02f;
                data.FmaxScale = 1.0f;
            }
            if (isHand)
            {
                data.FmaxScale = 0.5f;
            }
            if (isHead)
                boneEndPos+=(boneEndPos-boneStartPos) * 0.5f;

            Vector3 boneVector = (boneEndPos - boneStartPos);
            //Vector3 bonePosition = boneStartPos + 0.5f * boneVector;
            //Quaternion boneRotation = Quaternion.LookRotation(boneVector.normalized, Vector3.up);

            //link to parent bone
            OdeBody parentBody = null;
            if (parentTransform != null)
            {
                data.parent = startTransformToBoneData[parentTransform];
                parentBody = data.parent.rb;
                if (parentBody == null && createPhysicsObjects)
                    Debug.LogError("Could not find parent bone for " + controlObject.name);
            }
            else
            {
                data.parent = null;
            }

            //add rotation handle for posing (PERTTUTODO: refactor handles out of here?)
            if (addHandle)
                handles.Add(new RotationHandle(startTransform.gameObject.name, startTransform, boneEndPos, constrainTo2d));

            //fill in rest of the BoneData
            data.rb = null;
            data.c = null;
            data.joint = null;
            data.relativeControlPath = GetRelativePath(startTransform, targetCharacter);
            data.controlTransform = startTransform;
            data.parentTransform = parentTransform;
            data.controlObject = controlObject;
            data.index = bones.Count;
            data.name = startTransform.name;


            data.boneVector = boneVector;
            data.jointDef = def;
            bones.Add(data);

            //RefreshBone(data);
            if (!createPhysicsObjects)
                return;

            //create and init a capsule geom

            float len = Mathf.Max(thickness, boneVector.magnitude);//-thickness*0.5f);

            if (isFoot)
            {
                len *= 3.4f; //hack the toe transform is actually in the middle of the foot -> elongate
            }


            if (isFoot && useBoxesForFeet)
            {
                OdeGeomBox cc = controlObject.AddComponent<OdeGeomBox>();
                cc.lengths = new Vector3(1.5f * thickness, thickness, len);
                data.c = cc;
                
                // disable self-collisions
                UnityOde.odeGeomSetCollideBits(cc.GeomId, 8);
                UnityOde.odeGeomSetCategoryBits(cc.GeomId, 8);
            }
            else
            {
                OdeGeomCapsule cc = controlObject.AddComponent<OdeGeomCapsule>();
                cc.radius = thickness * 0.5f;
                cc.height = len;
                data.c = cc;

                // disable self-collisions except for legs where they're crucial for plausible mmovement
                if (data.mecanimBone == HumanBodyBones.LeftUpperLeg || data.mecanimBone == HumanBodyBones.RightUpperLeg)
                {
                    UnityOde.odeGeomSetCollideBits(cc.GeomId, 2);
                    UnityOde.odeGeomSetCategoryBits(cc.GeomId, 2);
                }
                else if (data.mecanimBone == HumanBodyBones.LeftLowerLeg || data.mecanimBone == HumanBodyBones.RightLowerLeg)
                {
                    UnityOde.odeGeomSetCollideBits(cc.GeomId, 4);
                    UnityOde.odeGeomSetCategoryBits(cc.GeomId, 4);
                }
                else
                {
                    UnityOde.odeGeomSetCollideBits(cc.GeomId, 0);
                    UnityOde.odeGeomSetCategoryBits(cc.GeomId, 1);
                }
            }

            data.thickness = thickness;

            // TODO: set material properties
            /*c.material=new PhysicMaterial();
            c.material.staticFriction=0.5f;
            c.material.dynamicFriction=0.5f;
            c.material.bounciness=0;
            c.material.bounceCombine=PhysicMaterialCombine.Minimum;
            c.material.frictionCombine=PhysicMaterialCombine.Average;*/
            //bone.renderer.enabled=false;
            //bone.transform.localRotation=Quaternion.FromToRotation(bone.transform.forward, boneVector.normalized);
            //bone.transform.rotation.SetLookRotation(boneVector.normalized, Vector3.up);
            //bone.transform.parent=startTransform;

            //create and init the rigid body
            OdeBody rb = null;
            rb = controlObject.AddComponent<OdeBody>();
            odeBodyIdToBoneData.Add(rb.BodyId, data);
            rb.position = boneStartPos + 0.5f * boneVector;
            rb.rotation = Quaternion.LookRotation(boneVector.normalized, Vector3.up);
            rb.updateUnityToOdeTransform();  //body remembers its rotation and translation offsets relative to the controlled Unity transform
            rb.isKinematic = true;
            data.c.SetDensity(humanDensity); //will recompute mass

            data.rb = rb;
            data.initialRotation = rb.rotation;

            if (parentTransform != null)
            {
                data.parent = startTransformToBoneData[parentTransform];

                //TODO: get rid of this, currently BackgroundMotionOptimizer.SetRigToSplineTime depends on this
                if (data.parent.controlTransform != startTransform.parent)
                {
                    List<Transform> transformsToParent = new List<Transform>();
                    List<Vector3> localPositionsToParent = new List<Vector3>();
                    List<Quaternion> localRotationsToParent = new List<Quaternion>();

                    for (Transform parent = startTransform.parent; parent != null && parent != data.parent.controlTransform; parent = parent.parent)
                    {
                        transformsToParent.Add(parent);
                        localPositionsToParent.Add(parent.localPosition);
                        localRotationsToParent.Add(parent.localRotation);
                    }

                    if (transformsToParent.Count == 0)
                    {
                        Debug.LogError("Hierarchy error");
                    }

                    data.transformHierarchyToParent = transformsToParent.ToArray();
                    data.localPositionHierarchyToParent = localPositionsToParent.ToArray();
                    data.localRotationHierarchyToParent = localRotationsToParent.ToArray();
                }
            }
            else
            {
                data.parent = null;
            }
            Debug.Log(rb.name + " phys: " + rb.rotation + " unity: " + controlObject.transform.rotation);


            //Body and Geometry now initialized. Now we need to create the joint if there is a parent body
            if (parentBody == null)
            {
                data.numFreedoms = 0;
            }
            else
            {
                Debug.Log("Connecting bone " + controlObject.name + " to " + parentBody.name);
                if (rb.BodyId == parentBody.BodyId)
                {
                    Debug.LogError(controlObject.name + ": Trying to create hinge joint between two bodies with same IDs: " + rb.name + " and " + parentBody.name + "(" + rb.BodyId + ")");
                    return;
                }

                //create the joint, using the parent bone as Ode body 1
                data.joint = def.Create(startTransform, parentBody, rb);
                data.hinge = data.joint as OdeJointHinge;

                //create and init a joint
                if (data.hinge!=null)
                {
                    //For the hinges, we want the rotation axis that is perpendicular to character forward and the bone vector.
                    //For thighs and upperarms pointing down, this is equal to the tposeRight
                    //Note that we don't just assume, e.g., hinge axis = tposeRight, since sometimes the t-pose may have arms 45 degrees downwards
                    //for easier skin weight adjustments
                    int hingeAxis = getBipedHingeLocalAxisIdx(data.mecanimBone);
                    data.hinge.axis = startTransform.TransformDirection(basisVectors[hingeAxis]);  //map local axis to global axis
                    Debug.Log("Bone " + data.name + ": mapped hinge axis to local axis " + hingeAxis + ", " + data.hinge.axis);
                    data.numFreedoms = 1;
                    UnityOde.odeJointSetHingeParam(data.hinge.JointId, (int)OdeWrapper.OdeParam.Vel, 0);
                    data.motor = data.hinge; //hinges have their own motors
                    //the hingeAxis member is used by Pose class quite heavily, and the implementation assumes
                    //that the range is 0..2, i.e., the hinge axis would directly correspond to a local axis of the mecanim rig, which
                    //we have later found to not be enough. Some rigs have flipped axes, which is why we now have the 6 possible indices used above.
                    data.hingeAxis = hingeAxis % 3; 
                }
                else if (data.joint as OdeJointBallAndSocket != null)
                {
                    data.numFreedoms = 3;
                    OdeJointAngularMotor motor = startTransform.GetComponent<OdeJointAngularMotor>();
                    data.motor = motor;
                    data.amotor=motor;
                    if (motor == null)
                        Debug.LogException(new Exception("Bone ball and socket joint missing a motor"));

                    //For ball and joint motors, we want the first axis to align with the character forward vector
                    //=> sideways swing
                    int motorAxis0idx = getClosestLocalAxisIdx(startTransform, tposeForward);
                    motor.SetAxis(0, 1, startTransform.TransformDirection(basisVectors[motorAxis0idx]));

                    //The second axis is the forward-backward swing, i.e., vector perpendicular to the forward and bone vectors.
                    //This is the same as the hinge axis above
                    int motorAxis2idx = getClosestLocalAxisIdx(startTransform, Vector3.Cross(tposeForward, boneVector.normalized));
                    motor.SetAxis(2, 2, startTransform.TransformDirection(basisVectors[motorAxis2idx]));

                        
                    motor.vel = 0;
                    motor.vel2 = 0;
                    motor.vel3 = 0;
                    Debug.Log("Bone " + data.name + ": mapped motor axes 0,2 to local axes " + motorAxis0idx + "," + motorAxis2idx);
                }
                else
                    Debug.LogError("Unknown joint type!");
            }
        }
        void AddBone(string start, string end, string parentName, OdeJointDefinition def, bool addHandle)
        {
			Transform startTransform=findRecursive(start);
			Transform endTransform=findRecursive(end);
			Transform parentTransform=null;
			if (parentName!=null)
				parentTransform=findRecursive(parentName);
			AddBone(startTransform,endTransform.position,parentTransform,def,addHandle);
		}
		void AddBone(HumanBodyBones start, HumanBodyBones end, HumanBodyBones parentName, OdeJointDefinition def, bool addHandle){
			Transform endTransform=anim.GetBoneTransform(end);
			if (endTransform==null)
				Debug.LogError("Bone end transform null!");
            AddBone(start, endTransform.position, parentName, def, addHandle);
		}
        void AddBone(HumanBodyBones start, Vector3 boneEndPos, HumanBodyBones parentName, OdeJointDefinition def, bool addHandle)
        {
            Transform startTransform = anim.GetBoneTransform(start);
            if (startTransform == null)
                Debug.LogError("Bone start transform null!");
            Transform parentTransform = null;
            if (parentName != start)
            {
                parentTransform = anim.GetBoneTransform(parentName);
                if (parentTransform == null)
                    Debug.LogError("Bone parent transform null!");
            }
            AddBone(startTransform, boneEndPos, parentTransform, def, addHandle);
        }
        void AddBone(Transform startTransform, Transform endTransform, Transform parentTransform, OdeJointDefinition def, bool addHandle)
        {
            AddBone(startTransform, endTransform.position, parentTransform, def, addHandle);
        }
        void disableBoneToBoneCollisions()
		{
            // TODO: implement with ODE collision masks
			/*foreach (BoneData bd1 in bones)
			{
				foreach (BoneData bd2 in bones)
				{
					if (bd1!=bd2)
						Physics.IgnoreCollision(bd1.c,bd2.c);
				}
			}*/
			
		}
	
        public void printRig()
        {
            Dictionary<BoneData.Category, int> categoryCounts = new Dictionary<BoneData.Category, int>();

            for (int i = 0; i < bones.Count; i++)
            {
                BoneData.Category category = bones[i].category;

                if (categoryCounts.ContainsKey(category))
                {
                    categoryCounts[category] = categoryCounts[category] + 1;
                }
                else
                {
                    categoryCounts[category] = 1;
                }
            }

            foreach (KeyValuePair<BoneData.Category, int> kv in categoryCounts)
            {
                Debug.Log("Category " + kv.Key + " count: " + kv.Value);
            }

            for (int i = 0; i < bones.Count; i++)
            {
                Debug.Log("Bone " + i + ": " + bones[i].rb.name + " (BodyId = " + bones[i].rb.BodyId + ", FirstAngleIdx = " + bones[i].firstAngleIndex.ToString() + ", numFreedoms = " +bones[i].numFreedoms +")");
            }
        }

        public void buildMecanimBipedRig(bool constrainTo2d = false)
		{
            //Compute character forward and right vectors, assuming character upright in t-pose
            //These will be used to figure out which bone local vectors (right, up, forward) are mapped to which motor axes.
            //Note that we assume that the rig is built so that one vector points along the bone and one points forward.
            anim = targetCharacter.GetComponent<Animator>();
            tposeRight = anim.GetBoneTransform(HumanBodyBones.RightFoot).position - anim.GetBoneTransform(HumanBodyBones.LeftFoot).position;
            tposeRight.y = 0;
            tposeRight.Normalize();
            //Left hand rule: cross-product lhs is thumb, rhs is index finger, result is middle finger
            tposeForward = Vector3.Cross(tposeRight, Vector3.up);
            Debug.Log("Character forward " + tposeForward + ", right " + tposeRight);

            //For ball joints (others than knee, angle, elbow), 
            //axis 0 points forward, positive angles rotate counterclockwise
            //axis 1 points along the bone
            //axis 2 axis points right in t-pose for legs, up for arms
            float hipSwingFwd = 110.0f;
            float hipSwingBack =  20.0f;
            float hipSwingOutwards = 45.0f;
            float hipSwingInwards = 15.0f;
            float hipTwistInwards =  10.0f;
            float hipTwistOutwards = 45.0f;
            float shoulderSwingFwd = 120.0f;
            float shoulderSwingBack = 45.0f;
            float shoulderSwingOutwards = 45.0f;
            float shoulderSwingInwards = 100.0f;
            float shoulderTwistRange = 45.0f;
            float spineSwingSideways =  20.0f;
            float spineSwingForward =  40.0f;
            float spineSwingBack = 10.0f;
            float spineTwist = 45.0f;
            float wristSwingFwd = 15.0f;
            float wristSwingBack = 15.0f;
            float wristSwingOutwards = 70.0f;
            float wristSwingInwards = 15.0f;
            float wristTwistRange = 30.0f;
            float ankleSwingRange = 30.0f;
            float kneeSwingRange = 140.0f;

            if (relaxedJointLimits)
            {
                hipSwingFwd = 120.0f;
                hipSwingBack =  30.0f;
                hipSwingOutwards = 60.0f;
                hipSwingInwards = 30.0f;
                hipTwistInwards = 15.0f;
                hipTwistOutwards = 45.0f;
                shoulderSwingFwd = 120.0f;
                shoulderSwingBack = 45.0f;
                shoulderSwingOutwards = 45.0f;
                shoulderSwingInwards = 100.0f;
                shoulderTwistRange = 60.0f;
                spineSwingSideways = 45.0f;
                spineSwingForward = 80.0f;
                spineSwingBack = 50.0f;
                spineTwist = 45.0f;
                wristSwingFwd = 45.0f;
                wristSwingBack = 25.0f;
                wristSwingOutwards = 80.0f;
                wristSwingInwards = 15;
                wristTwistRange = 90.0f;
                ankleSwingRange = 55.0f;
                kneeSwingRange = 180.0f;
                hingeAnkle = false;
            }

            bool addSpineJoint = false;
            bool addHeadJoint = extraHeadJoint;
            this.constrainTo2d = constrainTo2d;
			
			rootUp=new Vector3(0,1,0);
			//spine to head
            Transform hip = anim.GetBoneTransform(HumanBodyBones.Hips);            
            Transform spine=anim.GetBoneTransform(HumanBodyBones.Spine);
			Transform chest=anim.GetBoneTransform(HumanBodyBones.Chest);
            Transform neck = anim.GetBoneTransform(HumanBodyBones.Neck);
            Transform head = anim.GetBoneTransform(HumanBodyBones.Head);
            AddBone(hip, addSpineJoint ? spine : chest, null, OdeJointDefinition.CreateFixed(), false);
            //fix pelvis to xy plane if in 2d mode
            if (constrainTo2d && createPhysicsObjects)
            {
                int jointId = UnityOde.odeJointCreatePlane2D();
                UnityOde.odeJointAttach(jointId, bones[0].rb.BodyId, 0);
            }
            handles.Add(new TranslationHandle("Root",bones[0].controlTransform,constrainTo2d));
            handles.Add(new RotationHandle("Hips", hip, hip.position - (spine.position - hip.position).normalized * 0.2f, constrainTo2d));
        //    (bones[bones.Count - 1].c as OdeGeomCapsule).SetDensity(2.0f*humanDensity); //double the density to account for the torso volume not represented by the capsule

            if (addSpineJoint)
            {
                var spineJoint = constrainTo2d ?
                    OdeJointDefinition.CreateHinge(-45.0f, 45.0f) :
                    OdeJointDefinition.CreateBallAndSocket(new Vector3(-spineSwingSideways, -spineTwist, -spineSwingBack), 
                        new Vector3(spineSwingSideways, spineTwist, spineSwingForward));
                AddBone(spine, chest, hip, spineJoint, true);
          //      (bones[bones.Count - 1].c as OdeGeomCapsule).SetDensity(2.0f * humanDensity); //double the density to account for the torso volume not represented by the capsule
            }

            //chest to head
            var chestJointDef = constrainTo2d ?
                OdeJointDefinition.CreateHinge(-45.0f, 45.0f) :
                    OdeJointDefinition.CreateBallAndSocket(new Vector3(-spineSwingSideways, -spineTwist, -spineSwingBack),
                        new Vector3(spineSwingSideways, spineTwist, spineSwingForward));
            AddBone(chest, neck, addSpineJoint ? spine : hip, chestJointDef, true);
            Debug.Log("Chest index: " + (bones.Count - 1));

            var neckJoint = constrainTo2d ?
                OdeJointDefinition.CreateHinge(-45, 45) :
                    OdeJointDefinition.CreateBallAndSocket(new Vector3(-spineSwingSideways*0.5f, -spineTwist*0.5f, -spineSwingBack),
                        new Vector3(spineSwingSideways*0.5f, spineTwist*0.5f, spineSwingForward));
            AddBone(HumanBodyBones.Neck, addHeadJoint ? HumanBodyBones.Head : HumanBodyBones.Jaw, HumanBodyBones.Chest, neckJoint, true);

            if (addHeadJoint)
            {
                var headJoint = constrainTo2d ?
                    OdeJointDefinition.CreateHinge(-45, 45) :
                        OdeJointDefinition.CreateBallAndSocket(new Vector3(-spineSwingSideways, -spineTwist, -spineSwingBack),
                            new Vector3(spineSwingSideways, spineTwist, spineSwingForward));
                AddBone(HumanBodyBones.Head, HumanBodyBones.Jaw, HumanBodyBones.Neck, headJoint, true);
            }

            //left leg
            var  leftHipJointDef = constrainTo2d ? 
                OdeJointDefinition.CreateHinge(-90.0f, 90.0f) :
                OdeJointDefinition.CreateBallAndSocket(new Vector3(-hipSwingInwards, -hipTwistOutwards, -hipSwingBack), 
                    new Vector3(hipSwingOutwards, hipTwistInwards, hipSwingFwd));
            AddBone(HumanBodyBones.LeftUpperLeg, HumanBodyBones.LeftLowerLeg, HumanBodyBones.Hips, leftHipJointDef, true);
            AddBone(HumanBodyBones.LeftLowerLeg, HumanBodyBones.LeftFoot, HumanBodyBones.LeftUpperLeg, OdeJointDefinition.CreateHinge(-kneeSwingRange, -10), false);
            handles.Add(new ArmLegIKHandle("Left Foot IK", anim.GetBoneTransform(HumanBodyBones.LeftUpperLeg),
                anim.GetBoneTransform(HumanBodyBones.LeftLowerLeg),
                anim.GetBoneTransform(HumanBodyBones.LeftFoot),
                Mathf.PI, -1.0f, getBipedHingeLocalAxis(HumanBodyBones.LeftLowerLeg), constrainTo2d));

            var ankleTwist = 10.0f;
            var ankleRotate = 30.0f;
            var ankleDef = hingeAnkle ? OdeJointDefinition.CreateHinge(-ankleSwingRange, ankleSwingRange)
                : OdeJointDefinition.CreateBallAndSocket(new Vector3(-ankleTwist, -ankleSwingRange, -ankleRotate),
                    new Vector3(ankleTwist, ankleSwingRange, ankleRotate));
            AddBone(HumanBodyBones.LeftFoot, HumanBodyBones.LeftToes, HumanBodyBones.LeftLowerLeg, ankleDef, true);
	
			//right leg
            var rightHipJointDef = constrainTo2d ?
                OdeJointDefinition.CreateHinge(-90.0f, 90.0f) :
                OdeJointDefinition.CreateBallAndSocket(new Vector3(-hipSwingOutwards, -hipTwistInwards, -hipSwingBack),
                    new Vector3(hipSwingInwards, hipTwistOutwards, hipSwingFwd));
            AddBone(HumanBodyBones.RightUpperLeg, HumanBodyBones.RightLowerLeg, HumanBodyBones.Hips, rightHipJointDef, true);
            AddBone(HumanBodyBones.RightLowerLeg, HumanBodyBones.RightFoot, HumanBodyBones.RightUpperLeg, OdeJointDefinition.CreateHinge(-kneeSwingRange, -10), false);
            handles.Add(new ArmLegIKHandle("Right Foot IK", anim.GetBoneTransform(HumanBodyBones.RightUpperLeg),
                anim.GetBoneTransform(HumanBodyBones.RightLowerLeg),
                anim.GetBoneTransform(HumanBodyBones.RightFoot),
                Mathf.PI, -1.0f, getBipedHingeLocalAxis(HumanBodyBones.RightLowerLeg), constrainTo2d));

            AddBone(HumanBodyBones.RightFoot, HumanBodyBones.RightToes, HumanBodyBones.RightLowerLeg, ankleDef, true);

            
			//left arm
            var leftShoulderJointDef = constrainTo2d ?
                OdeJointDefinition.CreateHinge(-90.0f, 90.0f) :
                OdeJointDefinition.CreateBallAndSocket(new Vector3(-shoulderSwingInwards, -shoulderTwistRange, -shoulderSwingBack),
                    new Vector3(shoulderSwingOutwards, shoulderTwistRange, shoulderSwingFwd));
            AddBone(HumanBodyBones.LeftUpperArm, HumanBodyBones.LeftLowerArm, HumanBodyBones.Chest, leftShoulderJointDef, true);

            AddBone(HumanBodyBones.LeftLowerArm, HumanBodyBones.LeftHand, HumanBodyBones.LeftUpperArm, OdeJointDefinition.CreateHinge(0,140.0f), false);
            handles.Add(new ArmLegIKHandle("Left Hand IK", anim.GetBoneTransform(HumanBodyBones.LeftUpperArm),
                anim.GetBoneTransform(HumanBodyBones.LeftLowerArm),
                anim.GetBoneTransform(HumanBodyBones.LeftHand),
                Mathf.PI, 1.0f, getBipedHingeLocalAxis(HumanBodyBones.LeftLowerArm), constrainTo2d));
            
            var leftWristJointDef = constrainTo2d ?
                OdeJointDefinition.CreateHinge(-30.0f, 30.0f) :
                OdeJointDefinition.CreateBallAndSocket(new Vector3(-wristSwingInwards, -wristTwistRange, -wristSwingBack),
                    new Vector3(wristSwingOutwards, wristTwistRange, wristSwingFwd));
            Vector3 elbowToWrist = anim.GetBoneTransform(HumanBodyBones.LeftHand).position - anim.GetBoneTransform(HumanBodyBones.LeftLowerArm).position;
            Vector3 fingerTipEstimate = anim.GetBoneTransform(HumanBodyBones.LeftMiddleDistal).position;
            AddBone(HumanBodyBones.LeftHand, fingerTipEstimate, HumanBodyBones.LeftLowerArm, leftWristJointDef, true);
            

            //right arm
            var rightShoulderJointDef = constrainTo2d ?
                OdeJointDefinition.CreateHinge(-90.0f, 90.0f) :
                OdeJointDefinition.CreateBallAndSocket(new Vector3(-shoulderSwingOutwards, -shoulderTwistRange, -shoulderSwingBack),
                    new Vector3(shoulderSwingInwards, shoulderTwistRange, shoulderSwingFwd));
            AddBone(HumanBodyBones.RightUpperArm, HumanBodyBones.RightLowerArm, HumanBodyBones.Chest, rightShoulderJointDef, true);

            AddBone(HumanBodyBones.RightLowerArm, HumanBodyBones.RightHand, HumanBodyBones.RightUpperArm, OdeJointDefinition.CreateHinge(0,140.0f), false);
            handles.Add(new ArmLegIKHandle("Right Hand IK", anim.GetBoneTransform(HumanBodyBones.RightUpperArm),
                anim.GetBoneTransform(HumanBodyBones.RightLowerArm),
                anim.GetBoneTransform(HumanBodyBones.RightHand),
                Mathf.PI, 1.0f, getBipedHingeLocalAxis(HumanBodyBones.RightLowerArm), constrainTo2d));
            
            var rightWristJointDef = constrainTo2d ?
                OdeJointDefinition.CreateHinge(-30.0f, 30.0f) :
                OdeJointDefinition.CreateBallAndSocket(new Vector3(-wristSwingOutwards, -wristTwistRange, -wristSwingBack),
                    new Vector3(wristSwingInwards, wristTwistRange, wristSwingFwd));
            elbowToWrist = anim.GetBoneTransform(HumanBodyBones.RightHand).position - anim.GetBoneTransform(HumanBodyBones.RightLowerArm).position;
            fingerTipEstimate = anim.GetBoneTransform(HumanBodyBones.RightMiddleDistal).position ;
            AddBone(HumanBodyBones.RightHand, fingerTipEstimate, HumanBodyBones.RightLowerArm, rightWristJointDef, true);
         
   
            //scale total mass to 70kg
            //Commented out as this was just a test and everything has now been tuned with the old mass
            if (createPhysicsObjects)
            {
                float mass = 0;
                foreach (BoneData data in bones)
                {
                    mass += data.rb.mass;
                }
                float scaleFactor = 70.0f / mass;
                Debug.LogWarning("Rig physics mass " + mass + ", scaling mass by " + scaleFactor);
                foreach (BoneData data in bones)
                {
                    data.rb.mass = data.rb.mass * scaleFactor;
                    data.cachedMass = data.rb.mass;
                }
            }
            //finalize
            if (createPhysicsObjects)
				disableBoneToBoneCollisions();
            numFreedoms = 0;
            foreach (BoneData bd in bones)
            {
                bd.firstAngleIndex = numFreedoms;
                numFreedoms += bd.numFreedoms;
            }

            initBonelessTransforms();
		}

        public void initBonelessTransforms()
        {
            Debug.Log("Init boneless");
            bonelessTransformDefaults = getBonelessTransformStates();
        }

		public void initJointMotors(float timeStep, float positionSpring, float stopSpring, float damper, float maximumForce)
		{
            float kp = positionSpring; 
            float kd = damper; 

            float erp = timeStep * kp / (timeStep * kp + kd);
            float cfm = 1.0f / (timeStep * kp + kd);

            float stopDamper = 1.0f; //stops best when critically damped
            float stopErp = timeStep * stopSpring / (timeStep * kp + stopDamper);
            float stopCfm = 1.0f / (timeStep * stopSpring + stopDamper);

			foreach (BoneData bd in bones)
			{
				if (bd.joint!=null)
				{
                    OdeJointHinge hinge = bd.hinge;
                    OdeJointAngularMotor motor = bd.amotor;
                    if (hinge!=null)
                    {

                        hinge.fmax = maximumForce;
                        hinge.fudgeFactor = -1;
                        hinge.cfm = cfm;
                        hinge.erp = erp;
                        hinge.stopCFM = stopCfm;
                        hinge.stopERP = stopErp;

                        bd.tmpIsLeg = bd.rb.name.IndexOf("Knee") >= 0;
                    }
                    else if(motor != null)
                    {

                        motor.fmax = motor.fmax2 = motor.fmax3 = maximumForce;
                        motor.fudgeFactor = motor.fudgeFactor2 = motor.fudgeFactor3 = -1;
                        motor.cfm = motor.cfm2 = motor.cfm3 = cfm;
                        motor.erp = motor.erp2 = motor.erp3 = erp;

                        motor.stopCfm = motor.stopCfm2 = motor.stopCfm3 = stopCfm;
                        motor.stopErp = motor.stopErp2 = motor.stopErp3 = stopErp;

                    }

                    bd.defaultRotation = bd.controlTransform.localRotation;
                    bd.defaultEulers = bd.defaultRotation.eulerAngles;
                    bd.defaultJointLocalPosition = bd.controlTransform.localPosition;
                }
			}
		}
        //Note: not thread-safe
        static Vector3 computeMotorEulerAnglesForBodyRotations(BoneData bd, Quaternion bodyRotation, Quaternion connectedBodyRotation)
        {
            Vector3 result;
            UnityOde.odeJointGetAMotorAnglesFromBodyRotations(bd.motor.JointId, bodyRotation, connectedBodyRotation, out result);
            result.x = Mathf.Clamp(result.x, bd.jointDef.lostop, bd.jointDef.histop);
            result.y = Mathf.Clamp(result.y, bd.jointDef.lostop2, bd.jointDef.histop2);
            result.z = Mathf.Clamp(result.z, bd.jointDef.lostop3, bd.jointDef.histop3);
            //Maybe best to only prevent singularity instead of clamping to all stops?
            //The ODE AMotor axis 1 angle should never go beyond -PI/2...PI/2
            //result.y = Mathf.Clamp(result.y, -Mathf.PI * 0.45f, Mathf.PI * 0.45f);
            return Mathf.Rad2Deg * result;
        }

        public void updatePhysicsFromUnityTransforms()
        {
            foreach (BoneData bd in bones)
            {
                bd.rb.updatePhysicsFromControlledObject();
            }
        }
        //Converts the current pose of the character with the poseRig (can be the same as this rig) to a vector of motor angles.
        //If the srcRig has physics objects, their rotations are used. Otherwise, the srcRig unity transform rotations are first converted
        //to corresponding physics object rotations, using the relative rotations stored in the physics objects of this rig.
        //Note: not thread-safe
        
        public void poseToMotorAngles(Rig poseRig, ref float[] motorAngles)
        {
            int angleIdx = 0;
            for (int i = 0; i < bones.Count; i++)
            {
                BoneData bd = bones[i];
                if (bd.parent != null && bd.rb!=null) //only compute angles for objects with rigid bodies and a parent
                {
                    Vector3 bodyTargetPos;
                    Quaternion bodyTargetRot;
                    Vector3 body2TargetPos;
                    Quaternion body2TargetRot;
                    BoneData srcBd = poseRig.bones[i];
                    if (srcBd.rb != null)
                    {
                        body2TargetRot = srcBd.rb.rotation;
                        bodyTargetRot = srcBd.parent.rb.rotation;
                    }
                    else
                    {
                        bd.rb.controlledToPhysics(srcBd.controlTransform.position, srcBd.controlTransform.rotation, out body2TargetPos, out body2TargetRot);
                        bd.parent.rb.controlledToPhysics(srcBd.parent.controlTransform.position, srcBd.parent.controlTransform.rotation, out bodyTargetPos, out bodyTargetRot);
                    }

                    OdeJointAngularMotor motor = bd.motor as OdeJointAngularMotor;
                    OdeJointHinge hinge = bd.joint as OdeJointHinge;

                    if (motor != null)
                    {
                        Vector3 angles = computeMotorEulerAnglesForBodyRotations(bd, bodyTargetRot, body2TargetRot);
                        motorAngles[angleIdx++] = angles.x;
                        motorAngles[angleIdx++] = angles.y;
                        motorAngles[angleIdx++] = angles.z;
                    }
                    else if (hinge != null)
                    {
                        motorAngles[angleIdx++] = Mathf.Rad2Deg * UnityOde.odeJointGetHingeAngleFromBodyRotations(bd.joint.JointId, bodyTargetRot, body2TargetRot);
                    }
                }
            }
            if (angleIdx == 0)
                Debug.LogException(new Exception("poseToMotorAngles wrote 0 angles. Possible reason: this rig has no physics objects."));
        }

        public Vector3 boneTargetToEulers(int boneIndex, Quaternion targetRot)
        {
            BoneData bd = bones[boneIndex];
            Quaternion parentRotation = bd.parent.rb.rotation;
            return computeMotorEulerAnglesForBodyRotations(bd, parentRotation, targetRot);
        }

        public float boneTargetToHingeAngle(int boneIndex, Quaternion targetRot)
        {
            BoneData bd = bones[boneIndex];
            Quaternion parentRotation = bd.parent.rb.rotation;
            return Mathf.Rad2Deg * UnityOde.odeJointGetHingeAngleFromBodyRotations(bd.joint.JointId, parentRotation, targetRot);
        }

        public void setHingeAngleToMotorAngles(int boneIndex, float hingeAngle, ref float[] motorAngles, float lerp = 1)
        {
            int angleIdx = 0;

            for (int i = 0; i < boneIndex; i++)
            {
                angleIdx += bones[i].numFreedoms;
            }

            if (lerp < 1)
            {
                hingeAngle = Mathf.Lerp(motorAngles[angleIdx], hingeAngle, lerp);
            }

            motorAngles[angleIdx] = hingeAngle;
        }

        void rotateAround(Vector3 pos, Quaternion q, Vector3 point)
        {
            pos = point + q * (pos-point);
        }
        //Poses the character according to a vector of motor angles. Assumes that bones are added in root-first, top-down manner.
        //If updateUnityTransforms is true, this this method updates the controlled object transforms. 
        //Note that this will crash the editor if called from multiple threads.
        //targetRig==null means that the pose of this rig is changed.
        public void motorAnglesToPose(float[] motorAngles, bool updateUnityTransforms)
        {
            int angleIdx = 0;
            for (int i = 0; i < bones.Count; i++)
            {
                BoneData bd = bones[i];
                if (bd.numFreedoms > 0)
                {
                    //Compute motor axes (using look rotations stored when creating the hinge or motor, look rotations computed using the relative body info)
                    OdeJointAngularMotor motor = bd.amotor;
                    OdeJointHinge hinge = bd.hinge;
                    if (hinge != null)
                    {
                        if (bd.numFreedoms != 1)
                            Debug.LogError("numFreedoms must be 1 for a hinge!");
                        //Debug.Log("Posing bone " + bd.name + " with 1 angle at idx " + angleIdx); 
                        OdeBody body = hinge.body;
                        OdeBody connectedBody = hinge.connectedBody;
                        //get current hinge axis and desired angle
                        Vector3 axis = body.rotation * hinge.localAxes[0];
                        float hingeAngle = motorAngles[angleIdx++];

                        //compute new connected body rotation and position
                        Quaternion q = Quaternion.AngleAxis(-hingeAngle, axis);
                        Quaternion connectedBodyRot = q * (body.rotation * hinge.initialConnectedBodyLocalRotation);
                        Vector3 connectedBodyPos = body.position + body.rotation * hinge.initialConnectedBodyLocalPosition;
                        Vector3 anchor = body.position + body.rotation * hinge.localAnchor;
                        connectedBodyPos = anchor + q * (connectedBodyPos - anchor);    //rotate pos around anchor
                        connectedBody.rotation = connectedBodyRot;
                        connectedBody.position = connectedBodyPos;
                        if (updateUnityTransforms)
                        {
                            connectedBody.updateTransfromFromPhysics();
                        }
                    }
                    else if (motor != null)
                    {
                        /*
                         * Ode manual says of aMotor:  To get the body 2 axes from the body 1 axes the following sequence of rotations is performed:

                        Rotate by theta 0 about a0.
                        Rotate by theta 1 about a1 (a1 has been rotated from its original position).
                        Rotate by theta 2 about a2 (a2 has been rotated twice from its original position).
                         * Furthermore, in aMotor euler mode, a0 is relative to body 1, and a2 is relative to body 2
                         * */
                        //Debug.Log("Posing bone " + bd.name + " with 3 angles at idx " + angleIdx);
                        OdeBody body = motor.body;
                        OdeBody connectedBody = motor.connectedBody;

                        //get current axes
                        Vector3 a0 = body.rotation * motor.localAxes[0];
                        Vector3 a2 = body.rotation * motor.localAxes[2];
                        Vector3 a1 = Vector3.Cross(a2, a0);


                        //rotate connected body and a1, a2 around a0 at anchor
                        float angle = motorAngles[angleIdx++];
                        Quaternion q = Quaternion.AngleAxis(-angle, a0);
                        Quaternion connectedBodyRot = q * (body.rotation * motor.initialConnectedBodyLocalRotation);
                        Vector3 connectedBodyPos = body.position + body.rotation * motor.initialConnectedBodyLocalPosition;
                        Vector3 anchor = body.position + body.rotation * motor.localAnchor;
                        connectedBodyPos = anchor + q * (connectedBodyPos - anchor);    //rotate pos around anchor
                        a1 = q * a1;
                        a2 = q * a2;

                        //rotate connected body and a2 around a1 at anchor
                        angle = motorAngles[angleIdx++];
                        q = Quaternion.AngleAxis(-angle, a1);
                        connectedBodyRot = q * connectedBodyRot;
                        connectedBodyPos = anchor + q * (connectedBodyPos - anchor);    //rotate pos around anchor
                        a2 = q * a2;

                        //rotate connected body around a2 at anchor
                        angle = motorAngles[angleIdx++];
                        q = Quaternion.AngleAxis(-angle, a2);
                        connectedBodyRot = q * connectedBodyRot;
                        connectedBodyPos = anchor + q * (connectedBodyPos - anchor);    //rotate pos around anchor

                        //store results
                        connectedBody.rotation = connectedBodyRot;
                        connectedBody.position = connectedBodyPos;

                        if (updateUnityTransforms)
                        {
                            connectedBody.updateTransfromFromPhysics();
                        }
                    }
                }
            }
        }
        public void clampMotorAnglesWithLimits(float[] motorAngles)
        {

            int paramIndex = 0;
            for (int i = 0; i < bones.Count; i++)
            {
                BoneData boneData = bones[i];
                if (boneData.numFreedoms > 0)
                {
                    OdeJointDefinition def = boneData.joint.def;
                    motorAngles[paramIndex] = Mathf.Clamp(motorAngles[paramIndex], def.lostop, def.histop);
                    paramIndex++;
                }
                if (boneData.numFreedoms > 1)
                {
                    OdeJointDefinition def = boneData.joint.def;
                    motorAngles[paramIndex] = Mathf.Clamp(motorAngles[paramIndex], def.lostop2, def.histop2);
                    paramIndex++;
                }
                if (boneData.numFreedoms > 2)
                {
                    OdeJointDefinition def = boneData.joint.def;
                    motorAngles[paramIndex] = Mathf.Clamp(motorAngles[paramIndex], def.lostop3, def.histop3);
                    paramIndex++;
                }
            }
        }
        public List<BoneData> findViolatedAngleLimits(float[] motorAngles)
        {
            List<BoneData> ret = new List<BoneData>();
            
            int paramIndex = 0;
            for (int i = 0; i < bones.Count; i++)
            {
                BoneData boneData = bones[i];
                if (boneData.numFreedoms > 0)
                {
                    float angle = motorAngles[paramIndex];
                    OdeJointDefinition def = boneData.joint.def;
                    if (angle < def.lostop || angle > def.histop)
                        ret.Add(boneData);
                    paramIndex++;
                }
                if (boneData.numFreedoms > 1)
                {
                    float angle = motorAngles[paramIndex];
                    OdeJointDefinition def = boneData.joint.def;
                    if (angle < def.lostop2 || angle > def.histop2)
                        ret.Add(boneData);
                    paramIndex++;
                }
                if (boneData.numFreedoms > 2)
                {
                    float angle = motorAngles[paramIndex];
                    OdeJointDefinition def = boneData.joint.def;
                    if (angle < def.lostop3 || angle > def.histop3)
                        ret.Add(boneData);
                    paramIndex++;
                }
            }

            return ret;
        }


        public void postProcessHeadIK(Quaternion rot, float blendRatio)
        {
            if (findBoneIndex(HumanBodyBones.Head) == -1)
            {
                // no extra head joint
                int headIndex = getHeadIndex();
                int chestIndex = getChestIndex();

                BoneData head = bones[headIndex];
                BoneData chest = bones[chestIndex];
                Vector3 anchor = chest.rb.position + chest.rb.rotation * head.amotor.localAnchor;

                Quaternion originalRotation = head.rb.rotation;
                Quaternion fixedRot = rot * head.rb.unityToOdeRotation;
                Quaternion blended = Quaternion.Lerp(originalRotation, fixedRot, blendRatio);
                head.rb.rotation = blended;
                head.rb.position = anchor + rot * (0.5f * head.boneVector);
            }
            else
            {
                int headIndex = getHeadIndex();
                int neckIndex = getNeckIndex();

                BoneData head = bones[headIndex];
                BoneData neck = bones[neckIndex];
                Vector3 anchor = neck.rb.position + neck.rb.rotation * head.amotor.localAnchor;

                // TODO: this stuff is probably wrong, don't trust it
                Quaternion blended = Quaternion.Lerp(head.rb.rotation, rot, blendRatio);
                head.rb.rotation = blended;
                head.rb.position = anchor + blended * (head.rb.position - anchor);
            }
        }

        const float Deg2Rad = 0.0174533f;
        const float Rad2Deg = 57.2958f;
        const float PI = 3.14159f;
        //the swig import creates an unnecessary heap alloc - we manually define the import to optimize
        [DllImport("UnityOde", EntryPoint = "CSharp_odeJointGetMotorAnglesDegrees")]
        public static extern void odeJointGetMotorAnglesDegrees(int jointId, out Vector3 result);
        public Vector3 getLastSimulatedMotorAngles(BoneData data)
        {
            Vector3 result;
            odeJointGetMotorAnglesDegrees(data.motor.JointId, out result);
            return result;
        }
        public Vector3 getCurrentFMax(BoneData data)
        {

            if (data.hinge != null)
            {
                return new Vector3(data.hinge.fmax, 0.0f, 0.0f);
            }
            else if (data.amotor != null)
            {
                return new Vector3(data.amotor.fmax, data.amotor.fmax2, data.amotor.fmax3);
            }
            Debug.LogException(new Exception("Unknown joint type!"));
            return Vector3.zero;
        }

        public void setFMax(BoneData data, Vector3 fmax)
        {
            if (data.hinge != null)
            {
                data.hinge.fmax = fmax.x;
            }
            else if (data.amotor != null)
            {
                data.amotor.fmax = fmax.x;
                data.amotor.fmax2 = fmax.y;
                data.amotor.fmax3 = fmax.z;
            }
        }

        public float mysteryMultiplier = 2.0f;

        // This function should be called with euler angles as seen by ODE joints (hinge, amotor)
        // NOTE: called from multiple threads, can't access Unity components
        public void driveMotorToTarget(float timestep, BoneData data, Vector3 targetMotorEulers)
        {
            timestep *= mysteryMultiplier; //one frame to accelerate, one frame to drive. Without this one get's heavy overshoots.
            //Debug.Log("Drive2");
            OdeJointAngularMotor motor = data.motor as OdeJointAngularMotor;
            OdeJointHinge hinge = data.joint as OdeJointHinge;
            if (hinge != null)
            {
                float targetAngle = targetMotorEulers.x;

                {
                    float tmpAngle = Mathf.Rad2Deg * UnityOde.odeJointGetHingeAngle(hinge.JointId);
                    float z = Mathf.Deg2Rad * Mathf.DeltaAngle(tmpAngle, targetAngle);
                    z = z / timestep;
                    z = Mathf.Clamp(z, -maxTargetVel, maxTargetVel);
                    UnityOde.odeJointSetHingeParam(hinge.JointId, (int)OdeWrapper.OdeParam.Vel, z);
                }
            }
            else if(motor != null)
            {
                // x-axis
                {
                    float tmpAngle = Mathf.Rad2Deg * UnityOde.odeJointGetAMotorAngle(motor.JointId, 0);
                    float z = Mathf.Deg2Rad * Mathf.DeltaAngle(tmpAngle, targetMotorEulers.x);
                    z = z / timestep;
                    z = Mathf.Clamp(z, -maxTargetVel, maxTargetVel);
                    UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel1, z);
                }
                // y-axis
                {
                    float tmpAngle = Mathf.Rad2Deg * UnityOde.odeJointGetAMotorAngle(motor.JointId, 1);
                    float z = Mathf.Deg2Rad * Mathf.DeltaAngle(tmpAngle, targetMotorEulers.y);
                    z = z / timestep;
                    z = Mathf.Clamp(z, -maxTargetVel, maxTargetVel);
                    UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel2, z);
                }
                // z-axis
                {
                    float tmpAngle = Mathf.Rad2Deg * UnityOde.odeJointGetAMotorAngle(motor.JointId, 2);
                    float z = Mathf.Deg2Rad * Mathf.DeltaAngle(tmpAngle, targetMotorEulers.z);
                    z = z / timestep;
                    z = Mathf.Clamp(z, -maxTargetVel, maxTargetVel);
                    UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel3, z);
                }
 
            }
        }
        //drives motors to reach a target. Also sets fmax for each bone category (assumes that anglesAndFmax is a concatenation of motor target angles and fmaxes for each category)
        public void driveMotorsToTarget(float timeStep, float[] angles)
        {
            int paramIndex = 0;

            for (int i = 0; i < bones.Count; i++)
            {
                BoneData boneData = bones[i];
                if (bones[i].motor != null)
                {
                    Vector3 target = Vector3.zero;
                    switch (boneData.numFreedoms)
                    {
                        case 1:
                            target = new Vector3(angles[paramIndex], 0, 0);
                            break;
                        case 2:
                            target = new Vector3(angles[paramIndex], angles[paramIndex + 1], 0);
                            break;
                        case 3:
                            target = new Vector3(angles[paramIndex], angles[paramIndex + 1], angles[paramIndex + 2]);
                            break;
                    }
                    driveMotorToTarget(timeStep, bones[i], target);
                    paramIndex += boneData.numFreedoms;
                }
            }
        }

        public void setMotorSpeeds(float[] speeds)
        {
            int paramIndex = 0;

            for (int i = 0; i < bones.Count; i++)
            {
                BoneData data = bones[i];

                if (data.amotor!=null)
                {
                    UnityOde.odeJointSetAMotorVelocitiesDegreesPerSecond(data.amotor.JointId,speeds[paramIndex],speeds[paramIndex+1],speeds[paramIndex+2]);
                    paramIndex+=3;
                }
                else if (data.hinge != null)
                {
                    UnityOde.odeJointSetHingeParam(data.hinge.JointId, (int)OdeWrapper.OdeParam.Vel, speeds[paramIndex++]*Mathf.Deg2Rad);
                }
            }
        }
        public void setMotorSpeedsBiasedTowardsTarget(float[] speeds, float [] targetAngles, float timeStep, float alpha)
        {
            int paramIndex = 0;
            int angleIndex=0;

            for (int i = 0; i < bones.Count; i++)
            {
                BoneData data = bones[i];

                if (bones[i].motor != null)
                {
                    OdeJointAngularMotor motor = data.motor as OdeJointAngularMotor;
                    OdeJointHinge hinge = data.joint as OdeJointHinge;
                    Vector3 currAngles = getLastSimulatedMotorAngles(data);
                    currAngles *= Mathf.Deg2Rad;

                    if (hinge != null)
                    {
                        float targetSpeed = speeds[paramIndex++];
                        float targetAngleSpeed = (targetAngles[angleIndex++] * Mathf.Deg2Rad - currAngles.x) / timeStep * 0.5f;
                        targetSpeed = (1.0f - alpha) * targetSpeed + alpha * targetAngleSpeed;

                        if ((targetSpeed < 0 && currAngles.x > data.hinge.loStop)
                            || (targetSpeed > 0 && currAngles.x < data.hinge.hiStop))
                        { 
                            UnityOde.odeJointSetHingeParam(hinge.JointId, (int)OdeWrapper.OdeParam.Vel, Mathf.Deg2Rad * targetSpeed);
                        }
                        else
                        {
                            UnityOde.odeJointSetHingeParam(hinge.JointId, (int)OdeWrapper.OdeParam.Vel, 0);
                        }
                    }
                    else if (motor != null)
                    {
                        float targetSpeed = speeds[paramIndex++];
                        float targetAngleSpeed = (targetAngles[angleIndex++] * Mathf.Deg2Rad - currAngles.x) / timeStep * 0.5f;
                        targetSpeed = (1.0f - alpha) * targetSpeed + alpha * targetAngleSpeed;
                        if ((targetSpeed < 0 && currAngles.x > data.amotor.loStop)
                            || (targetSpeed > 0 && currAngles.x < data.amotor.hiStop))
                        {
                            UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel1, Mathf.Deg2Rad * targetSpeed);
                        }
                        else
                        {
                            UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel1, 0);
                        }
                        targetSpeed = speeds[paramIndex++];
                        targetAngleSpeed = (targetAngles[angleIndex++] * Mathf.Deg2Rad - currAngles.x) / timeStep*0.5f;
                        targetSpeed = (1.0f - alpha) * targetSpeed + alpha * targetAngleSpeed;
                        if ((targetSpeed < 0 && currAngles.y > data.amotor.loStop2)
                            || (targetSpeed > 0 && currAngles.y < data.amotor.hiStop2))
                        {
                            UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel2, Mathf.Deg2Rad * targetSpeed);
                        }
                        else
                        {
                            UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel2, 0);
                        }
                        targetSpeed = speeds[paramIndex++];
                        targetAngleSpeed = (targetAngles[angleIndex++] * Mathf.Deg2Rad - currAngles.x) / timeStep * 0.5f;
                        targetSpeed = (1.0f - alpha) * targetSpeed + alpha * targetAngleSpeed;
                        if ((targetSpeed < 0 && currAngles.z > data.amotor.loStop3)
                            || (targetSpeed > 0 && currAngles.z < data.amotor.hiStop3))
                        {
                            UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel3, Mathf.Deg2Rad * targetSpeed);
                        }
                        else
                        {
                            UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel3, 0);
                        }
                        /*
                        UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel1, Mathf.Deg2Rad * speeds[paramIndex++]);
                        UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel2, Mathf.Deg2Rad * speeds[paramIndex++]);
                        UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel3, Mathf.Deg2Rad * speeds[paramIndex++]);
                         * */
                    }
                }
            }
        }
        /*        public void setMotorTorques(float[] torques)
                {
                    int paramIndex = 0;

                    for (int i = 0; i < bones.Count; i++)
                    {
                        BoneData data = bones[i];

                        if (bones[i].motor != null)
                        {
                            OdeJointAngularMotor motor = data.motor as OdeJointAngularMotor;
                            OdeJointHinge hinge = data.joint as OdeJointHinge;

                            if (hinge != null)
                            {
                                float val=torques[paramIndex++];
                                UnityOde.odeJointSetHingeParam(hinge.JointId, (int)OdeWrapper.OdeParam.Vel, Mathf.Sign(val)*Mathf.PI*2.0f);
                                UnityOde.odeJointSetHingeParam(hinge.JointId, (int)OdeWrapper.OdeParam.FMax, val);
                            }
                            else if (motor != null)
                            {
                                float val = torques[paramIndex++];
                                UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel1, Mathf.Sign(val) * Mathf.PI * 2.0f);
                                UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.FMax1, val);
                                val = torques[paramIndex++];
                                UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel2, Mathf.Sign(val) * Mathf.PI * 2.0f);
                                UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.FMax2, val);
                                val = torques[paramIndex++];
                                UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.Vel3, Mathf.Sign(val) * Mathf.PI * 2.0f);
                                UnityOde.odeJointSetAMotorParam(motor.JointId, (int)OdeWrapper.OdeParam.FMax3, val);
                            }
                        }
                    }
                }
                */
        public void setMotorTorques(float[] torques)
        {
            int paramIndex = 0;

            for (int i = 0; i < bones.Count; i++)
            {
                BoneData data = bones[i];

                if (bones[i].motor != null)
                {
                    OdeJointAngularMotor motor = data.motor as OdeJointAngularMotor;
                    OdeJointHinge hinge = data.joint as OdeJointHinge;

                    if (hinge != null)
                    {
                        UnityOde.odeJointAddHingeTorque(hinge.JointId, torques[paramIndex++]);
                    }
                    else if (motor != null)
                    {
                        float torque1=torques[paramIndex++];
                        float torque2=torques[paramIndex++];
                        float torque3=torques[paramIndex++];
                        float scale = 1;
                        if (data.mecanimBone == HumanBodyBones.Neck || data.mecanimBone == HumanBodyBones.LeftHand || data.mecanimBone == HumanBodyBones.RightHand)
                            scale *= 0.2f;
                        if (data.mecanimBone == HumanBodyBones.LeftFoot || data.mecanimBone == HumanBodyBones.RightFoot)
                            scale *= 0.5f;
                        UnityOde.odeJointAddAMotorTorques(motor.JointId, torque1*scale, torque2*scale, torque3*scale);
                    }
                }
            }
        }
        public Vector3 getMatchTargetMotorAngles(int boneIdx, Rig targetRig)
        {
            Vector3 bodyTargetPos;
            Quaternion bodyTargetRot;
            BoneData data = bones[boneIdx];
            data.rb.controlledToPhysics(
                targetRig.bones[boneIdx].controlTransform.position,
                targetRig.bones[boneIdx].controlTransform.rotation,
                out bodyTargetPos, out bodyTargetRot);
            Vector3 parentTargetPos;
            Quaternion parentTargetRot;
            data.parent.rb.controlledToPhysics(
                targetRig.bones[boneIdx].parent.controlTransform.position,
                targetRig.bones[boneIdx].parent.controlTransform.rotation,
                out parentTargetPos, out parentTargetRot);
            OdeJointAngularMotor motor = data.amotor;
            OdeJointHinge hinge = data.hinge;
            Vector3 motorTargetEulers = Vector3.zero;
            if (motor != null)
            {
                motorTargetEulers = computeMotorEulerAnglesForBodyRotations(bones[boneIdx], parentTargetRot,bodyTargetRot);
            }
            else if (hinge != null)
            {
                motorTargetEulers.x = Mathf.Rad2Deg * UnityOde.odeJointGetHingeAngleFromBodyRotations(data.motor.JointId, parentTargetRot,bodyTargetRot);
            }
            else
            {
                Debug.LogError("Must have either amotor or hinge!");
            }
            return motorTargetEulers;

        }
        public void driveMotoredBoneToTargetRig(float timestep,int boneIdx, Rig targetRig)
        {
            Vector3 targetEulers = getMatchTargetMotorAngles(boneIdx, targetRig);
            driveMotorToTarget(timestep,bones[boneIdx], targetEulers);
        }

        public void setKinematic(bool isKinematic)
		{
			foreach (BoneData bd in bones)
			{
				bd.rb.isKinematic=isKinematic;
			}
		}
		public void setBoneLayers(int layer)
		{
			foreach (BoneData bd in bones)
			{
				bd.controlObject.layer=layer;
			}
		}
        public void add2DSoftConstraintForce(BoneData bd, float multiplier)
        {
            OdeBody body = bd.rb;
            float z = body.position.z;
            Vector3 force = new Vector3(0, 0, -multiplier * z);
            UnityOde.odeBodyAddForce(body.BodyId, force);
        }
        public void add2DSoftConstraintTorque(BoneData bd, float multiplier, Quaternion aimToRotateTo)
        {
            OdeBody body = bd.rb;
            Quaternion currentRotation = body.rotation;
            Quaternion toRotation = aimToRotateTo * Quaternion.Inverse(currentRotation);
            Vector3 eulers = multiplier * Mathf.Deg2Rad * toRotation.eulerAngles;

            UnityOde.odeBodyAddTorque(body.BodyId, eulers);
        }
        public void getCurrentSteppedPoseFromCharacter(float[] pose)
        {
            getCurrentSteppedPoseFromCharacter(ref pose);
        }

        public void getCurrentSteppedPoseFromCharacter(ref float[] pose, int firstOutputIndex=0)
        {
            int paramIndex = firstOutputIndex;

            foreach (Rig.BoneData boneData in bones)
            {
                if (boneData.motor == null)
                {
                    continue;
                }

                Vector3 currentMotorAngles = getLastSimulatedMotorAngles(boneData);

                int freedomsLeft = boneData.numFreedoms;

                if (freedomsLeft-- > 0)
                {
                    pose[paramIndex++] = currentMotorAngles.x;
                }

                if (freedomsLeft-- > 0)
                {
                    pose[paramIndex++] = currentMotorAngles.y;
                }

                if (freedomsLeft-- > 0)
                {
                    pose[paramIndex++] = currentMotorAngles.z;
                }
            }
        }
        void printCategories()
        {
            foreach (BoneData data in bones)
            {
                Debug.Log("Bone " + data.name + ", mecanim name " + data.mecanimBone + " category: " + data.category);
            }

        }
        public void getCurrentFMaxForEachCategory(ref float [] fmax, int idxOffset=0)
        {
            for (int i = idxOffset; i < idxOffset+(int)BoneData.Category.COUNT; i++)
                fmax[i] = 0;
            foreach (BoneData data in bones)
            {
                if (data.motor != null)
                {
                    float boneFmax = getCurrentFMax(data).x;
                    int idx = (int)data.category;
                    if (data.category == BoneData.Category.UNKNOWN)
                        Debug.LogError("Category unknown for bone " + data.name +", mecanim name " + data.mecanimBone);
                    int nBones = nBonesPerCategory[idx];
                    fmax[idx + idxOffset] += boneFmax / (float)nBones;
                }
            }

        }
        float computeBoostedJointFmax(float stopViolationAmount, float stopFmax, float normalFmax)
        {
            float stopFmaxMix = Math.Min(stopViolationAmount / 25.0f, 1.0f);
            return stopFmax * stopFmaxMix + normalFmax * (1.0f - stopFmaxMix);
        }
        float stopViolationAmount(float stopAngle, float currAngle)
        {
            float maxViolation=60.0f;
            //handle wrapping
            if (stopAngle < 0)
            {
                stopAngle += 10.0f;
                if (currAngle - 360.0f > stopAngle - maxViolation)
                    currAngle -= 360.0f;
                return Mathf.Max(stopAngle - currAngle, 0);
            }
            else
            {
                stopAngle -= 10.0f;
                if (currAngle + 360.0f < stopAngle + maxViolation)
                    currAngle += 360.0f;
                return Mathf.Max(currAngle-stopAngle, 0);
            }
        }
        public void setFMaxForAllMotors(float fmax)
        {
            Vector3 v=new Vector3(fmax,fmax,fmax);
            foreach (BoneData data in bones)
            {
                if (data.motor != null)
                {
                    setFMax(data, v);
                }
            }
        }
        public void setFMaxForEachCategorySimple(float[] fmax, int idxOffset = 0)
        {
            for (int i = 0; i < bones.Count; i++) 
            {
                BoneData data = bones[i];
                if (data.motor != null)
                {
                    float newFmax = fmax[idxOffset + (int)data.category];
                    newFmax *= data.FmaxScale;
                    UnityOde.odeJointSetFmax(data.motor.JointId, newFmax, newFmax, newFmax);
                }
            }
        }
        public void setFMaxForEachCategory(float[] fmax, int idxOffset = 0)
        {
            foreach (BoneData data in bones)
            {
                if (data.motor != null)
                {
                    //get default category fmax
                    float newFmax = fmax[idxOffset + (int)data.category];
                    //if (data.category != BoneData.Category.ARMS)
                    //    newFmax *= 0.5f;
                   //     newFmax = Mathf.Max(newFmax, 20);

                    //boost fmax if we're close to a stop of a hinge joints - fmax no longer affects the limits of amotors
                    Vector3 boostedFmax = new Vector3(newFmax, newFmax, newFmax);

                    if (data.mecanimBone == HumanBodyBones.LeftLowerLeg || data.mecanimBone == HumanBodyBones.RightLowerLeg
                     )//   || data.mecanimBone == HumanBodyBones.LeftLowerArm || data.mecanimBone == HumanBodyBones.RightLowerArm)
                    {
                        Vector3 angles = getLastSimulatedMotorAngles(data);
                        float maxViolation = 0;
                        float stopFmax = 250.0f;
                        if (data.amotor != null)
                        {
                            maxViolation = stopViolationAmount(data.amotor.loStop * Rad2Deg, angles.x);
                            maxViolation = Math.Max(maxViolation, stopViolationAmount(data.amotor.hiStop * Rad2Deg, angles.x));
                            boostedFmax.x = computeBoostedJointFmax(maxViolation, stopFmax, newFmax);
                            maxViolation = stopViolationAmount(data.amotor.loStop2 * Rad2Deg, angles.y);
                            maxViolation = Math.Max(maxViolation, stopViolationAmount(data.amotor.hiStop2 * Rad2Deg, angles.y));
                            boostedFmax.y = computeBoostedJointFmax(maxViolation, stopFmax, newFmax);
                            maxViolation = stopViolationAmount(data.amotor.loStop3 * Rad2Deg, angles.z);
                            maxViolation = Math.Max(maxViolation, stopViolationAmount(data.amotor.hiStop3 * Rad2Deg, angles.z));
                            boostedFmax.z = computeBoostedJointFmax(maxViolation, stopFmax, newFmax);
                        }
                        else if (data.hinge != null)
                        {
                            maxViolation = stopViolationAmount(data.hinge.loStop * Rad2Deg, angles.x);
                            maxViolation = Math.Max(maxViolation, stopViolationAmount(data.hinge.hiStop * Rad2Deg, angles.x));
                            boostedFmax.x = computeBoostedJointFmax(maxViolation, stopFmax, newFmax);
                        }
                    }
                    
                    //special cases for wrists and feet
                    /*if (data.mecanimBone == HumanBodyBones.LeftHand ||
                        data.mecanimBone == HumanBodyBones.RightHand)
                    {
                        boostedFmax = new Vector3(30,30,30);
                    }*/
                    setFMax(data,boostedFmax);
                }
            }
        }
        public int getBoneDataIdxForMecanimPart(HumanBodyBones bone)
        {
            Transform t=anim.GetBoneTransform(bone);
            for (int i = 0; i < bones.Count; i++)
            {
                if (bones[i].controlTransform==t)
                    return i;
            }
            Debug.LogException(new Exception("Could not find BoneData for " + bone));
            return -1;
        }
        public Vector3 getCOM()
        {
            float mass = 0.0f;
            Vector3 comAccum = Vector3.zero;
            foreach (BoneData bd in bones)
            {
                mass += bd.rb.mass;
                comAccum += bd.rb.mass * bd.rb.position;
            }
            comAccum *= (1.0f / mass);
            return comAccum;
        }
        public Vector3 getCOMVelocity()
        {
            float mass = 0.0f;
            Vector3 comAccum = Vector3.zero;
            foreach (BoneData bd in bones)
            {
                mass += bd.rb.mass;
                comAccum += bd.rb.mass * bd.rb.velocity;
            }
            comAccum *= (1.0f / mass);
            return comAccum;
        }
        //utility for balancing goals
        public Vector3 getGetCOMRelativeToMeanFeet(Vector3 leftFoot, Vector3 rightFoot, Vector3 COM)
        {
            Vector3 meanFeet = 0.5f * (leftFoot + rightFoot);
            Vector3 leftToRight = (rightFoot - leftFoot);
            leftToRight.y = 0;
            leftToRight.Normalize();
            Quaternion rotation = Quaternion.FromToRotation(Vector3.forward, leftToRight);
            Vector3 comDispRotated = COM - meanFeet;
            //comDispRotated = rotation * comDisp
            Vector3 comDisp = Quaternion.Inverse(rotation) * comDispRotated;
            return comDisp;
        }

        public void getMirroredAngles(float[] angles, ref float[] toAngles)
        {
            int angleIndex = 0;

            for (int i = 0; i < bones.Count; i++)
            {
                BoneData data = bones[i];
                if (data.motor!=null)
                {
                    int sourceAngleIndex = angleIndex;
                    //check if this is a "sided" bone, and find the corresponding bone on the other side
                    if (data.mecanimBone.ToString().Contains("Left") || data.mecanimBone.ToString().Contains("Right"))
                    {
                        //mecanim enums are defined so that, e.g., lefthand and righthand are after each other
                        if (data.mecanimBone.ToString().Contains("Left"))
                            sourceAngleIndex=mecanimBoneToBoneData[(HumanBodyBones)(1+(int)data.mecanimBone)].firstAngleIndex;
                        else
                            sourceAngleIndex=mecanimBoneToBoneData[(HumanBodyBones)(-1+(int)data.mecanimBone)].firstAngleIndex;
                    }
                    if (data.hinge != null)
                    {
                        //for hinges, mirroring has no effect
                        toAngles[angleIndex++] = angles[sourceAngleIndex++];
                    }
                    else if (data.amotor!=null)
                    {
                        //the way axes are set up in buildMecanimBipedRig() and AddBone(), we only need to change the signe
                        //of amotor axis 0 rotation
                        toAngles[angleIndex++]=-angles[sourceAngleIndex++];
                        toAngles[angleIndex++] = -angles[sourceAngleIndex++];
                        toAngles[angleIndex++] = angles[sourceAngleIndex++];
                    }
                }
            }
        }
        //Mirrors both the Unity pose and physics objects. Requires a physics rig for reference (may be same as target rig if target rig has physics objects), 
        //as the mirroring is based on motor angles.
        public static void mirrorMotoredBones(Rig targetRig, Rig physicsRig)
        {
            //save physics rig state
            float[] currentPhysicsRigAngles=new float[targetRig.GetRigNumFreedoms()];
            physicsRig.poseToMotorAngles(physicsRig,ref currentPhysicsRigAngles);

            //compute motor angles from unity pose
            float[] currentAngles = new float[targetRig.GetRigNumFreedoms()];
            physicsRig.poseToMotorAngles(targetRig, ref currentAngles);
            float[] mirroredAngles = new float[targetRig.GetRigNumFreedoms()];
            physicsRig.getMirroredAngles(currentAngles, ref mirroredAngles);
            physicsRig.motorAnglesToPose(mirroredAngles, true);
 
            if (targetRig != physicsRig)
            {
                //copy from physics rig to target rig
                for (int i = 1; i < physicsRig.bones.Count; i++)
                {
                    targetRig.bones[i].controlTransform.localRotation = physicsRig.bones[i].controlTransform.localRotation;
                }

                //restore physics rig state
                physicsRig.motorAnglesToPose(currentPhysicsRigAngles, true);
            }

        }
        //Copies the Unity pose and physics objects form left to right. Requires a physics rig for reference (may be same as target rig if target rig has physics objects), 
        //as the copying is based on motor angles.
        public static void makeSymmetric(Rig targetRig, Rig physicsRig)
        {
            //save physics rig state
            float[] currentPhysicsRigAngles = new float[targetRig.GetRigNumFreedoms()];
            physicsRig.poseToMotorAngles(physicsRig, ref currentPhysicsRigAngles);
            //compute motor angles from unity pose
            float[] currentAngles = new float[targetRig.GetRigNumFreedoms()];
            physicsRig.poseToMotorAngles(targetRig, ref currentAngles);
            float[] mirroredAngles = new float[targetRig.GetRigNumFreedoms()];
            physicsRig.getMirroredAngles(currentAngles, ref mirroredAngles);
            float[] symmetricAngles = new float[targetRig.GetRigNumFreedoms()];
            for (int i = 0; i < physicsRig.bones.Count; i++)
            {
                BoneData bd = physicsRig.bones[i];
                if (bd.mecanimBone.ToString().Contains("Right"))
                {
                    //for right side bones, use the mirrored angles
                    for (int k = bd.firstAngleIndex; k < bd.firstAngleIndex + bd.numFreedoms; k++)
                    {
                        symmetricAngles[k] =  mirroredAngles[k];
                    }
                }
                else
                {
                    //for others, use the current angles
                    for (int k = bd.firstAngleIndex; k < bd.firstAngleIndex + bd.numFreedoms; k++)
                    {
                        symmetricAngles[k] = currentAngles[k];
                    }
                }
            }
            physicsRig.motorAnglesToPose(symmetricAngles, true); 
            if (targetRig != physicsRig)
            {
                //copy from physics rig to target rig
                for (int i = 1; i < physicsRig.bones.Count; i++)
                {
                    targetRig.bones[i].controlTransform.localRotation = physicsRig.bones[i].controlTransform.localRotation;
                }

                //restore physics rig state
                physicsRig.motorAnglesToPose(currentPhysicsRigAngles, true);
            }

        }

        public Dictionary<Transform, BonelessTransformData> getBonelessTransformStates()
        {
            Dictionary<Transform, BonelessTransformData> states = new Dictionary<Transform, BonelessTransformData>();
            getBonelessTransformStatesFrom(targetCharacter, states);
            return states;
        }

        private void getBonelessTransformStatesFrom(Transform transform, Dictionary<Transform, BonelessTransformData> states)
        {
            if (!startTransformToBoneData.ContainsKey(transform))
            {
                states.Add(transform, new BonelessTransformData()
                {
                    localPosition = transform.localPosition,
                    localRotation = transform.localRotation
                });
            }

            for (int i = 0; i < transform.childCount; i++)
            {
                Transform child = transform.GetChild(i);
                getBonelessTransformStatesFrom(child, states);
            }
        }

        public void resetBonelessTransforms()
        {
            setBonelessTransforms(bonelessTransformDefaults);
        }

        public void setBonelessTransforms(Dictionary<Transform, BonelessTransformData> states)
        {
            foreach (KeyValuePair<Transform, BonelessTransformData> kvp in states)
            {
                Transform transform = kvp.Key;
                BonelessTransformData data = kvp.Value;

                transform.localPosition = data.localPosition;
                transform.localRotation = data.localRotation;
            }
        }

        public static void copyPose(Rig src, Rig dst, bool copyRootRotation)
        {
            if (copyRootRotation)
                dst.bones[0].controlTransform.localRotation = src.bones[0].controlTransform.localRotation;
            for (int i = 1; i < src.bones.Count; i++)
            {
                dst.bones[i].controlTransform.localRotation = src.bones[i].controlTransform.localRotation;
                if (dst.bones[i].rb != null)
                    dst.bones[i].rb.updatePhysicsFromControlledObject();
            }
        }

        public List<int> getEndEffectorIndices()
        {
            List<int> ret = new List<int>();
            for (int i = 0; i != bones.Count; ++i)
            {
                BoneData b = bones[i];
                if (b.mecanimBone == HumanBodyBones.LeftFoot
                    || b.mecanimBone == HumanBodyBones.RightFoot
                    || b.mecanimBone == HumanBodyBones.LeftHand
                    || b.mecanimBone == HumanBodyBones.RightHand)
                {
                    ret.Add(i);
                }
            }
            return ret;
        }

        public float getCharacterHeight()
        {
            // ballpark estimate for standing height (used to scale COM error for balancing)
            return 2.0f;
        }

        public int getRootIndex()
        {
            return 0;
        }

        public int getChestIndex()
        {
            return findBoneIndex(HumanBodyBones.Chest);
        }

        public int getHeadIndex()
        {
            int headIndex = findBoneIndex(HumanBodyBones.Head);
            if (headIndex != -1)
                return headIndex;
            else
                return findBoneIndex(HumanBodyBones.Neck);
        }

        public int getNeckIndex()
        {
            int headIndex = findBoneIndex(HumanBodyBones.Neck);
            if (headIndex != -1)
                return headIndex;
            else
                return findBoneIndex(HumanBodyBones.Chest);
        }

        private int findBoneIndex(HumanBodyBones bone)
        {
            for (int i = 0; i != bones.Count; ++i)
            {
                BoneData b = bones[i];
                if (b.mecanimBone == bone)
                    return i;
            }
            return -1;
        }
        public void disableCollisionsExceptWithGround()
        {
            foreach (BoneData bd in bones)
            {
                if (bd.rb!=null)
                {
                    UnityOde.odeGeomSetCollideBits(bd.rb.Geometry.GeomId, 0);
                    UnityOde.odeGeomSetCategoryBits(bd.rb.Geometry.GeomId, 1);
                }
            }
        }
        public void setFixedUpdateEnabledForPhysics(bool enabled)
        {
            foreach (BoneData bd in bones)
            {
                if (bd.rb != null)
                {
                    bd.rb.disableUpdate = !enabled;
                }
                if (bd.joint != null)
                {
                    bd.joint.disableUpdate = !enabled;
                }
            }
        }
       
    } //Rig
} //AaltoGames

