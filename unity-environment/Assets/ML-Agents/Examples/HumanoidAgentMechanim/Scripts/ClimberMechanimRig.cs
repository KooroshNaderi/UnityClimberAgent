using System.Collections.Generic;
using UnityEngine;
using Assets.ML_Agents.Examples.SharedAssets.Scripts;
using Assets.ML_Agents.Examples.ClimberScripts;

public class ClimberMechanimRig : HumanoidMecanimRig
{
    public class ClimberBodyState
    {
        static int humanoid_action_dof = 30;

        void InitVariables()
        {
            connectorPos = new Vector3[4];
            connectorRot = new Quaternion[4];
            connectorVel = new Vector3[4];
            connectorAVel = new Vector3[4];

            spline_init_values = new Vector2[humanoid_action_dof];
        }

        public ClimberBodyState()
        {
            InitVariables();
        }

        public ClimberBodyState(ClimberBodyState _c)
        {
            InitVariables();
            Copy(_c);
        }

        public void Copy(ClimberBodyState _c)
        {
            HumanoidBodyState.Copy(_c.HumanoidBodyState);

            for (int h = 0; h < 4; h++)
            {
                hold_bodies_ids[h] = _c.hold_bodies_ids[h];
                current_hold_pos[h] = _c.current_hold_pos[h];
                current_hold_rot[h] = _c.current_hold_rot[h];
                current_hold_type[h] = _c.current_hold_type[h];

                end_bodies_pos[h] = _c.end_bodies_pos[h];
            }

            hipLocation = _c.hipLocation;

            for (int h = 0; h < 4; h++)
            {
                connectorPos[h] = _c.connectorPos[h];
                connectorRot[h] = _c.connectorRot[h];
                connectorVel[h] = _c.connectorVel[h];
                connectorAVel[h] = _c.connectorAVel[h];
            }

            for (int i = 0; i < humanoid_action_dof; i++)
            {
                spline_init_values[i] = _c.spline_init_values[i];
            }
            return;
        }

        public HumanoidBodyState HumanoidBodyState = new HumanoidBodyState();

        public int[] hold_bodies_ids = { -1, -1, -1, -1 };
        public Vector3[] current_hold_pos = new Vector3[4];
        public Quaternion[] current_hold_rot = new Quaternion[4];
        public HoldInfo.HoldType[] current_hold_type = new HoldInfo.HoldType[4];

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
        for (int i = 0; i < this.numControlDOFs(); i++)
        {
            spline[i] = new RecursiveTCBSpline();
            spline[i].setState(0.0f, 0.0f);
        }

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
            // until this point all body state is saved
            base._SaveState(mClimberMemory.savedStates[_freeSlotIdx].HumanoidBodyState);

            // now save current stance ids and positions
            for (int h = 0; h < 4; h++)
            {
                mClimberMemory.savedStates[_freeSlotIdx].hold_bodies_ids[h] = connectBodyParts[h].current_hold_id;
                mClimberMemory.savedStates[_freeSlotIdx].current_hold_pos[h] = mContextInfo.GetHoldPosition(connectBodyParts[h].current_hold_id);
                mClimberMemory.savedStates[_freeSlotIdx].current_hold_rot[h] = mContextInfo.GetHoldRotation(connectBodyParts[h].current_hold_id);
                mClimberMemory.savedStates[_freeSlotIdx].current_hold_type[h] = mContextInfo.GetHoldType(connectBodyParts[h].current_hold_id);

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
        if (mClimberMemory == null)
            return false;
        if (mClimberMemory.savedStates == null)
            return false;

        if (_state_id >= 0 && _state_id < mClimberMemory.savedStates.Count)
        {
            // disconnect hands and feet
            for (int b = 0; b < 4; b++)
                connectBodyParts[b].disconnectBodyPart();

            // load all body state
            base._LoadState(mClimberMemory.savedStates[_state_id].HumanoidBodyState);
            
            // now load current stance info
            for (int h = 0; h < 4; h++)
            {
                int c_hold_id = mClimberMemory.savedStates[_state_id].hold_bodies_ids[h];
                if (c_hold_id >= 0)
                {
                    mContextInfo.SetHoldPosition(c_hold_id, mClimberMemory.savedStates[_state_id].current_hold_pos[h]);
                    mContextInfo.SetHoldRotation(c_hold_id, mClimberMemory.savedStates[_state_id].current_hold_rot[h]);
                    mContextInfo.SetHoldType(c_hold_id, mClimberMemory.savedStates[_state_id].current_hold_type[h]);
                }
                connectBodyParts[h]._bodyPartInfo.localPosition = mClimberMemory.savedStates[_state_id].end_bodies_pos[h];
            }
            
            // load connector info
            for (int b = 0; b < 4; b++)
            {
                connectBodyParts[b].GetRigidBody().transform.localPosition = mClimberMemory.savedStates[_state_id].connectorPos[b];
                connectBodyParts[b].GetRigidBody().transform.localRotation = mClimberMemory.savedStates[_state_id].connectorRot[b];
                connectBodyParts[b].GetRigidBody().velocity = mClimberMemory.savedStates[_state_id].connectorVel[b];
                connectBodyParts[b].GetRigidBody().angularVelocity = mClimberMemory.savedStates[_state_id].connectorAVel[b];
            }

            // save spline values
            for (int i = 0; i < mClimberMemory.savedStates[_state_id].spline_init_values.Length; i++)
            {
                spline[i].setState(mClimberMemory.savedStates[_state_id].spline_init_values[i][0], 
                    mClimberMemory.savedStates[_state_id].spline_init_values[i][1]);
            }

            // connect hands and feet given current hold-ids
            for (int b = 0; b < 4; b++)
            {
                int c_hold_id = mClimberMemory.savedStates[_state_id].hold_bodies_ids[b];
                if (c_hold_id >= 0)
                    connectBodyParts[b].connectBodyPart(c_hold_id);
            }

            return true; // successful saving
        }

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
                Vector3 hold_pos_i = mContextInfo.GetHoldPosition(targetHoldIds[i]);
                Vector3 contact_pos_i = GetEndBonePosition(i);
                float dis_i = (hold_pos_i - contact_pos_i).magnitude;

                float _connectionThreshold = mContextInfo.ConnectionThreshold;

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

    public void SetColorToLimb(int limb_id, Color _color)
    {
        connectBodyParts[limb_id]._bodyPartInfo.gameObject.GetComponent<Renderer>().material.color = _color;
    }

    public Vector3 GetEndBoneGlobalPosition(int limb_id)
    {
        return connectBodyParts[limb_id]._bodyPartInfo.position;
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

    public ContextManager GetRigsContext()
    {
        return mContextInfo;
    }

    //public Vector3 GetHoldPositionInContext(int holdId)
    //{
    //    return mContextInfo.GetHoldPosition(holdId);
    //}

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
        return connectBodyParts[2]._joint.currentForce.sqrMagnitude + connectBodyParts[3]._joint.currentForce.sqrMagnitude;
    }
}
