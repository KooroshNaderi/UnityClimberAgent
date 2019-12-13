using System.Collections.Generic;
using UnityEngine;
using Assets.ML_Agents.Examples.SharedAssets.Scripts;
using Assets.ML_Agents.Examples.ClimberScripts;

public class ClimberMechanimRig : HumanoidMecanimRig
{
    public class ClimberBodyState
    {
        static int humanoid_action_dof = 30;
        public ClimberBodyState()
        {
            connectorPos = new Vector3[4];
            connectorRot = new Quaternion[4];
            connectorVel = new Vector3[4];
            connectorAVel = new Vector3[4];

            spline_init_values = new Vector2[humanoid_action_dof];
        }

        public int HumanoidBodyStateID { get; set; } = -1;
        public int[] hold_bodies_ids = { -1, -1, -1, -1 };
        public Vector3[] current_hold_pos = new Vector3[4];
        public Vector3[] end_bodies_pos = new Vector3[4];
        public Vector3 hipLocation = new Vector3();

        public Vector3[] connectorPos;
        public Quaternion[] connectorRot;
        public Vector3[] connectorVel;
        public Vector3[] connectorAVel;

        // spline action setting
        public Vector2[] spline_init_values;
    }

    public int _trajectoryIdx = -1;

    //HumanoidMecanimRig humanoidRig;
    [Header("End Points Connectors")]
    public Transform RHConnector;
    public Transform LHConnector;
    public Transform RLConnector;
    public Transform LLConnector;

    List<EndBodyPart> connectBodyParts = new List<EndBodyPart>();

    [HideInInspector] public SharedMemoryManager mClimberMemory = null;
    private ContextManager mContextInfo = null;

    public RecursiveTCBSpline[] spline;

    bool isInitialized = false;
    // Start is called before the first frame update
    void Start()
    {
        Initialize();
    }

    // Update is called once per frame
    void Update()
    {
        //Debug.Log(getEndBonePosition(0));
    }

    public override void Initialize()
    {
        if (isInitialized)
            return;

        base.Initialize();

        mContextInfo = this.transform.parent.GetComponent<ContextManager>();
        
        // initilize spline action settings
        spline = new RecursiveTCBSpline[this.numControlDOFs()];

        for (int i = 0; i < 4; i++)
        {
            switch (i)
            {
                case (int)LowLevelController.ControlledPoses.LeftLeg:
                    connectBodyParts.Add(LLConnector.GetComponent<EndBodyPart>());
                    break;
                case (int)LowLevelController.ControlledPoses.RightLeg:
                    connectBodyParts.Add(RLConnector.GetComponent<EndBodyPart>());
                    break;
                case (int)LowLevelController.ControlledPoses.LeftHand:
                    connectBodyParts.Add(LHConnector.GetComponent<EndBodyPart>());
                    break;
                case (int)LowLevelController.ControlledPoses.RightHand:
                    connectBodyParts.Add(RHConnector.GetComponent<EndBodyPart>());
                    break;
            }
        }

        isInitialized = true;
    }

    // this function saves the state of the current climber's rig
    public override bool SaveState(int _freeSlotIdx)
    {
        if (mClimberMemory == null)
            return false;
        if (mClimberMemory.savedStates == null)
            return false;

        if (_freeSlotIdx >= 0 && _freeSlotIdx < mClimberMemory.savedStates.Count)
        {
            if (mClimberMemory.savedStates[_freeSlotIdx].HumanoidBodyStateID < 0)
            {
                mClimberMemory.savedStates[_freeSlotIdx].HumanoidBodyStateID = this.getNextFreeSavingSlot();
            }

            // until this point all body state is saved
            base.SaveState(mClimberMemory.savedStates[_freeSlotIdx].HumanoidBodyStateID);

            // now save current stance ids and positions
            for (int h = 0; h < 4; h++)
            {
                mClimberMemory.savedStates[_freeSlotIdx].hold_bodies_ids[h] = connectBodyParts[h].current_hold_id;
                mClimberMemory.savedStates[_freeSlotIdx].current_hold_pos[h] = mContextInfo.GetHoldPosition(connectBodyParts[h].current_hold_id);
                mClimberMemory.savedStates[_freeSlotIdx].end_bodies_pos[h] = connectBodyParts[h]._bodyPartInfo.localPosition;
            }
            // save hip position
            mClimberMemory.savedStates[_freeSlotIdx].hipLocation = this.pos(HumanBodyBones.Hips);

            // save connector info
            for (int b = 0; b < 4; b++)
            {
                mClimberMemory.savedStates[_freeSlotIdx].connectorPos[b] = connectBodyParts[b].GetRigidBody().transform.localPosition;
                mClimberMemory.savedStates[_freeSlotIdx].connectorRot[b] = connectBodyParts[b].GetRigidBody().transform.localRotation;
                mClimberMemory.savedStates[_freeSlotIdx].connectorVel[b] = connectBodyParts[b].GetRigidBody().velocity;
                mClimberMemory.savedStates[_freeSlotIdx].connectorAVel[b] = connectBodyParts[b].GetRigidBody().angularVelocity;
            }

            if (mClimberMemory.savedStates[_freeSlotIdx].spline_init_values.Length != this.numControlDOFs())
            {
                mClimberMemory.savedStates[_freeSlotIdx].spline_init_values = new Vector2[this.numControlDOFs()];
            }
            // save spline values
            for (int i = 0; i < this.numControlDOFs(); i++)
            {
                mClimberMemory.savedStates[_freeSlotIdx].spline_init_values[i][0] = spline[i].currentValue;
                mClimberMemory.savedStates[_freeSlotIdx].spline_init_values[i][1] = spline[i].currentDerivativeValue;
            }

            return true; // successful saving
        }

        return false;
    }
    
    // this function loads the given state id to the current climber's rig
    public override bool LoadState(int _state_id)
    {
        return false;
    }

    int counterHandsDisconnected = 0;
    static int maxCountHandsDisconnected = 20;
    public void ConnectDisconnectContactPoint(int[] targetHoldIds, bool[] _allowRelease)
    {
        bool isHandsDisconnected = GetCurrentHoldBodyID(2) == -1 && GetCurrentHoldBodyID(3) == -1;

        if (isHandsDisconnected)
        {
            counterHandsDisconnected++;
        }
        else
        {
            counterHandsDisconnected = 0;
        }

        for (int i = 0; i < targetHoldIds.Length; i++)
        {
            if (targetHoldIds[i] != -1)
            {
                Vector3 hold_pos_i = GetHoldPositionInContext(targetHoldIds[i]);
                Vector3 contact_pos_i = GetEndBonePosition(i);
                float dis_i = (hold_pos_i - contact_pos_i).magnitude;

                float _connectionThreshold = 0.5f * ContextManager.HoldSize;

                if (dis_i <= _connectionThreshold)
                {
                    connectBodyParts[i].connectBodyPart(targetHoldIds[i]);
                }
                // safty measure or allowing to let go of the hold
                else if ((dis_i > _connectionThreshold + 0.1f) || (targetHoldIds[i] != GetCurrentHoldBodyID(i) && _allowRelease[i]))
                {
                    connectBodyParts[i].disconnectBodyPart();
                }
            }
            else
            {
                if (_allowRelease[i])
                {
                    connectBodyParts[i].disconnectBodyPart();
                }
            }
        }

        if (counterHandsDisconnected > maxCountHandsDisconnected)
        {
            for (int i = 0; i <= 1; i++)
            {
                connectBodyParts[i].disconnectBodyPart();
            }
            counterHandsDisconnected = 0;
        }

        return;
    }

    public List<EndBodyPart> GetEndBodyParts()
    {
        return connectBodyParts;
    }

    // return the global position of end limb poses with respect to the agent's environment
    public Vector3 GetEndBonePosition(int limb_id)
    {
        return connectBodyParts[limb_id].Pos;
    }

    public int GetCurrentHoldBodyID(int limb_id)
    {
        return connectBodyParts[limb_id].current_hold_id;
    }

    public List<float> ReadDesiredAnglePos()
    {
        return this.readAnglePos();
    }

    public void SetHoldPositionInContext(int holdId, Vector3 pos)
    {
        mContextInfo.SetHoldPosition(holdId, pos);
    }

    public Vector3 GetHoldPositionInContext(int holdId)
    {
        return mContextInfo.GetHoldPosition(holdId);
    }

    public ContextManager GetContextManager()
    {
        return mContextInfo;
    }

    public Transform GetBoneTransform(HumanBodyBones bone)
    {
        if (!extraHeadJoint && bone == HumanBodyBones.Head)
            bone = HumanBodyBones.Neck;
        return mecanimBoneToBoneData[(int)bone].controlTransform;
    }

    public int[] GetCurrentHoldIds()
    {
        return new int[] {connectBodyParts[0].current_hold_id,
                          connectBodyParts[1].current_hold_id,
                          connectBodyParts[2].current_hold_id,
                          connectBodyParts[3].current_hold_id};
    }

    public float GetSqForceOnFingers()
    {
        return 0.0f;
    }
}
