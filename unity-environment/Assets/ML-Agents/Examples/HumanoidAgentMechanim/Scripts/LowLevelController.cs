using Assets.ML_Agents.Examples.SharedAssets.Scripts;
using System.Collections.Generic;
using UnityEngine;
using ICM;
using System.IO;
using System.Diagnostics;

namespace Assets.ML_Agents.Examples.ClimberScripts
{
    public class LowLevelController
    {
        public enum ControlledPoses { LeftLeg = 0, RightLeg = 1, LeftHand = 2, RightHand = 3, MiddleTrunk = 4, TorsoDir = 5 };

        // this class can do optimization along side with forward simulation of samples from neural network
        // optimization is only done with splines, but neural network samples (PPO samples) can be 1 step 
        const bool flag_writeToFile = false;

        public class SamplingTrajectoryStr
        {
            public SamplingTrajectoryStr(int _sampleActionSize, bool _use_spline)
            {
                _sampledAction = new float[_sampleActionSize];
                sample_action_size = _sampleActionSize;
                _UseSpline = _use_spline;
            }

            public int sample_action_size = -1;
            public bool _UseSpline = false;

            public int datum_index = -1;
            public int[] targetHoldIds = { -1, -1, -1, -1 };

            public List<ControlledPoses> sourcePos = new List<ControlledPoses>();
            public List<Vector3> targetPos = new List<Vector3>();

            public double objectiveFuncVal = double.MinValue;
            public float[] _sampledAction;
            public bool isReached = false;

            public int _sampledMaxStep = 0;
            public int _cSimulationStep = 0;
            public int _cMemoryStepUsed = 0;// used for showing animation + showing states in one trajectory action used (stance-to-stance)
            public bool isActionDone = false; // means the planned trajectory has completed _sampledMaxStep
            public bool ResetFlag = true;

            public float state_cost = 0.0f; // the state cost value after opt
            public float control_cost = 0.0f; // the control cost value after opt

            public int _cSlotStateIdx = -1; // current state of the climber
            public int _startingSlotIndex = -1; // the beginning state of the climber
            public List<int> _fromToStates = new List<int>();
        };

        private int _MaxSampleSize = 10;
        private float _MaxSampleDis = 0.25f;
        private bool _PlayAnimation = false;
        private bool _UsePPOReward = false;
        bool _ReplayExpreinces = false;
        public bool ReplayExpreinces
        {
            get { return _ReplayExpreinces; }
            set { _ReplayExpreinces = value; }
        }
        public bool PlayAnimation
        {
            get { return _PlayAnimation; }
            set { _PlayAnimation = value; }
        }
        float _PercentFollowRef = 0.8f;
        public float PercentFollowRef
        {
            get { return _PercentFollowRef; }
            set { _PercentFollowRef = value; }
        }
        public float MaxSampleDis
        {
            get { return _MaxSampleDis; }
            set { _MaxSampleDis = value; }
        }
        public int MaxSampleSize
        {
            get { return _MaxSampleSize; }
            set { _MaxSampleSize = value; }
        }

        private const bool cmaesLinearInterpolation = false;
        private const int nControlPoints = 2;
        private const int nAngle = 30; // if 30 - 3, i.e. not controlling head!
                                       //private const int _sampleSize = _UseSpline 
                                       //                              ? nControlPoints * (1 + nAngle /*time + nAngle*/) + 4 /*time to let go of the hands and feet*/
                                       //                              : nAngle;
        private const int _maxOptIter = 120;
        private const int _maxFixedOptIter = 20;
        private const float _fixedThresholdCost = 50.0f;
        private const int maxCountHandsDisconnected = 5;
        private const int nNNEvalTrajectories = 0;
        private const int nTrajectorySamplingOpt = 36;

        const float forceCostSd = 300.0f;  //for computing control cost (maximumForce)

        List<ClimberMechanimRig> _controlRigs = new List<ClimberMechanimRig>();

        MAES _opt = new MAES();
        const float minSampleVal = -2.5f;
        const float maxSampleVal = 2.5f;
        const float minSegmentTime = 0.25f;
        const float maxSegmentTime = 1.0f;
        int MaxStepsInOptSample = -1;

        int nOptimizationSamples;
        OptimizationSample[] _samples;
        OptimizationSample BestSample; // this sample is in the form that is generated from cma-es
        double[] _initialMean;
        double neuralNetSampleVal = double.MinValue;

        public List<SamplingTrajectoryStr> _trajectorySamples = new List<SamplingTrajectoryStr>();
        // this sample is used in optimization and transformed in the form of actual applying angle
        SamplingTrajectoryStr bestPreSample = null;//new SamplingTrajectoryStr(_sampleSize); 

        List<float>[] nnEvalStartingState = new List<float>[nNNEvalTrajectories];
        List<float>[] nnEvalStartingHolds = new List<float>[nNNEvalTrajectories];
        List<double>[] nnEvalPredictedPolicy = new List<double>[nNNEvalTrajectories];
        List<float>[] nnEvalReply = new List<float>[nNNEvalTrajectories];
        float[] nnEvalValuePolicy = new float[nNNEvalTrajectories];

        List<float[]> interpolatedControl = new List<float[]>();
        List<bool[]> letGoControl = new List<bool[]>();

        //int[] segmentIdTrajectories;

        List<float> _desiredClimbingAnglePos;
        int _iter = 0;
        int _fixedIter = 0;
        int _maxActionStepsEachFrame = 0;
        int nTrajectories;

        SharedMemoryManager mMemory;
        NetworkManager mNetworkManager;
        int startingStateIdx = -1;
        int masterContextIdx = -1;
        List<float> predicted_policy_mean_std = new List<float>();
        StreamReader mFileReader = null;
        StreamWriter mFileWriter = null;
        int current_opened_file = 0;
        int current_written_file = 0;
        int current_written_data_index = 0;
        int curShownState = 0;

        // saves T-pose state
        int reset_memory_idx = -1;
        List<SamplingTrajectoryStr> initial_trajectory_points = new List<SamplingTrajectoryStr>();
        private readonly Stopwatch mStopWatch = new Stopwatch();

        public LowLevelController(GameObject _instantObj, SharedMemoryManager iMemory, NetworkManager iNetworkManager)
        {
            Physics.autoSimulation = false;

            mMemory = iMemory;
            mNetworkManager = iNetworkManager;

            // copy from given object for forward simulation
            int trajectoryIdx = 0;
            int Column = 2; // having two columns side by side (col = 0 is for opt, col = 1 is for PPO)
            for (int i = 0; i < Column; i++)
            {
                int cRow = nTrajectorySamplingOpt;
                if (i == 1)
                    cRow = nNNEvalTrajectories;
                for (int j = 0; j < cRow; j++)
                {
                    GameObject g = Object.Instantiate(_instantObj);
                    g.name = "HumanTraj" + (i * Column + j).ToString();

                    ClimberMechanimRig _climberMechanimRig = g.GetComponentsInChildren<ClimberMechanimRig>()[0];
                    _climberMechanimRig._trajectoryIdx = trajectoryIdx++;
                    _climberMechanimRig.mClimberMemory = iMemory;
                    _climberMechanimRig.Initialize();
                    _controlRigs.Add(_climberMechanimRig);

                    // g.GetComponent<MeshRenderer>().enabled = false;
                    g.transform.position = new Vector3(i * 8.5f, 2 + 10, j * 4.5f);
                }
            }

            //init trajectory samples, each trajectory sample can have 
            //-- different action type (spline,1-step)
            //-- defferent step, but max step size is 100 (1 sec) forward simulation
            const int opt_sample_size = nControlPoints * (1 + nAngle /*time + nAngle*/) + 4 /*time to let go of the hands and feet*/;
            const int nn_sample_size = nAngle;// + 4;
            nTrajectories = trajectoryIdx;
            nOptimizationSamples = nTrajectories - nNNEvalTrajectories;
            MaxStepsInOptSample = (int)(maxSegmentTime / Time.fixedDeltaTime);
            for (int t = 0; t < nTrajectories; t++)
            {
                if (t < nOptimizationSamples)
                    _trajectorySamples.Add(new SamplingTrajectoryStr(opt_sample_size, true));
                else
                    _trajectorySamples.Add(new SamplingTrajectoryStr(nn_sample_size, false));
                _trajectorySamples[t]._cSlotStateIdx = mMemory.GetNextFreeSlotIdx();
                _trajectorySamples[t]._startingSlotIndex = mMemory.GetNextFreeSlotIdx();

                for (int _s = 0; _s < MaxStepsInOptSample + 5; _s++)
                    _trajectorySamples[t]._fromToStates.Add(mMemory.GetNextFreeSlotIdx());

                interpolatedControl.Add(new float[nAngle]);
                letGoControl.Add(new bool[4]);
            }
            //segmentIdTrajectories = new int[nTrajectories];

            //_tmp_creating_init_states = mMemory.GetNextFreeSlotIdx();

            bestPreSample = new SamplingTrajectoryStr(opt_sample_size, true);
            bestPreSample._cSlotStateIdx = mMemory.GetNextFreeSlotIdx();
            bestPreSample._startingSlotIndex = mMemory.GetNextFreeSlotIdx();
            for (int _s = 0; _s < MaxStepsInOptSample + 5; _s++)
                bestPreSample._fromToStates.Add(mMemory.GetNextFreeSlotIdx());

            // showing the best trajectory on master context
            ClimberMechanimRig _climberMechanimMasterRig = _instantObj.GetComponentsInChildren<ClimberMechanimRig>()[0];
            _climberMechanimMasterRig._trajectoryIdx = trajectoryIdx;
            _climberMechanimMasterRig.mClimberMemory = iMemory;
            _climberMechanimMasterRig.Initialize();
            _controlRigs.Add(_climberMechanimMasterRig);
            masterContextIdx = trajectoryIdx;

            // reset memory index
            reset_memory_idx = mMemory.GetNextFreeSlotIdx();

            // init start and current slot indecies
            _controlRigs[masterContextIdx].SaveState(reset_memory_idx);
            for (int t = 0; t < nTrajectories; t++)
            {
                _controlRigs[masterContextIdx].SaveState(_trajectorySamples[t]._cSlotStateIdx);
                _controlRigs[masterContextIdx].SaveState(_trajectorySamples[t]._startingSlotIndex);
            }
            // init the best sample memory slots
            _controlRigs[masterContextIdx].SaveState(bestPreSample._cSlotStateIdx);
            _controlRigs[masterContextIdx].SaveState(bestPreSample._startingSlotIndex);

            //// add reset state to initial states
            //initial_states.Add(reset_memory_idx);
            //_counter_memory_init_states = 1;

            // initialization of cma-es samples, opt only works with splines
            _desiredClimbingAnglePos = _controlRigs[GetMasterTrajectoryIdx()].ReadDesiredAnglePos();
            _samples = new OptimizationSample[nOptimizationSamples];
            BestSample = new OptimizationSample(opt_sample_size);

            _initialMean = new double[opt_sample_size];
            for (int i = 0; i < nOptimizationSamples; i++)
            {
                _samples[i] = new OptimizationSample(opt_sample_size);
            }

            // message passing parameters
            for (int i = 0; i < nNNEvalTrajectories; i++)
            {
                nnEvalStartingState[i] = new List<float>();
                nnEvalStartingHolds[i] = new List<float>();
                nnEvalPredictedPolicy[i] = new List<double>();
                nnEvalReply[i] = new List<float>();
            }
            return;
        }

        public void LoadToMaster(int state_id)
        {
            _controlRigs[GetMasterTrajectoryIdx()].LoadState(state_id);
        }

        //    void ResetEnvAndTargetForForwardSimulation(int _trajectoryIdx, bool _isFirstItr, ref int[] optTragetIds)
        //    {
        //        return;
        //        //bool change_target = true;
        //        //if (_trajectoryIdx < nOptimizationSamples)
        //        //{
        //        //    _controlRigs[_trajectoryIdx].loadState(startingStateIdx);
        //        //    if (_isFirstItr)
        //        //    {
        //        //        for (int h = 0; h < 4; h++)
        //        //            _trajectorySamples[_trajectoryIdx].targetHoldIds[h] = optTragetIds[h];
        //        //    }
        //        //    else
        //        //    {
        //        //        change_target = false;
        //        //    }
        //        //}
        //        //else if (mNetworkManager.IsConnected)
        //        //{
        //        //    _controlRigs[_trajectoryIdx].loadState(_trajectorySamples[_trajectoryIdx]._startingSlotIndex);
        //        //}
        //        //else if (_trajectoryIdx - nOptimizationSamples < nnEvalStartingState.Length)
        //        //{
        //        //    HumanoidMecanimRig.HumanoidBodyState nState = new HumanoidMecanimRig.HumanoidBodyState();
        //        //    nState.LoadStateFromList(ref nnEvalStartingState[_trajectoryIdx - nOptimizationSamples]);

        //        //    SetHoldPositionsFromList(_trajectoryIdx, _trajectoryIdx - nOptimizationSamples, nState.end_bodies_pos);
        //        //    _controlRigs[_trajectoryIdx].loadState(ref nState);
        //        //}

        //        //if (change_target)
        //        //{
        //        //    _trajectorySamples[_trajectoryIdx].sourcePos.Clear();
        //        //    _trajectorySamples[_trajectoryIdx].targetPos.Clear();
        //        //    for (int _h = 0; _h < _trajectorySamples[_trajectoryIdx].targetHoldIds.Length; _h++)
        //        //    {
        //        //        if (_trajectorySamples[_trajectoryIdx].targetHoldIds[_h] >= 0)
        //        //        {
        //        //            _trajectorySamples[_trajectoryIdx].sourcePos.Add((ControlledPoses)(ControlledPoses.LeftLeg + _h));
        //        //            _trajectorySamples[_trajectoryIdx].targetPos.Add(GetHoldPositionInContext(_trajectoryIdx, _trajectorySamples[_trajectoryIdx].targetHoldIds[_h]));
        //        //        }
        //        //    }
        //        //}
        //        //return;
        //    }

        int InitTrajectories(ref SamplingHighLevelPlan _samplePlan)
        {
            int curNumTrajectories = nOptimizationSamples;
            // new starting state and stance target is set when opt _iter is zero
            int initial_state_opt = _samplePlan._sampledInitialStateSlotIdx;

            // if high-level planner is not used, use a simple cyclic movement planner as follows
            if (initial_state_opt < 0)
            {
                if (startingStateIdx < 0)
                {
                    // start from t-pose
                    initial_state_opt = reset_memory_idx;
                }
                else
                {
                    initial_state_opt = startingStateIdx;
                }
            }

            // our opt should create a cyclic clip of climbing, which sample stance is coming from
            for (int t = 0; t < nOptimizationSamples; t++)
            {
                if (_trajectorySamples[t].ResetFlag)
                {
                    if (t == 0)
                    {
                        _controlRigs[t].LoadState(initial_state_opt);
                        for (int id = 0; id < 4; id++)
                        {
                            _trajectorySamples[t].targetHoldIds[id] = _samplePlan._sampledTargetStanceID[id];
                        }
                        //if (_iter == 0)
                        //{
                        //    // only for _iter = 0 of opt we can set the target stance with their position
                        //    RandomizeHoldScence(t, initial_state_opt);
                        //    SampleRandomStanceIDs(t);
                        //}
                    }
                    else
                    {
                        _controlRigs[t].LoadState(_trajectorySamples[0]._startingSlotIndex);
                        for (int id = 0; id < 4; id++)
                        {
                            _trajectorySamples[t].targetHoldIds[id] = _trajectorySamples[0].targetHoldIds[id];
                        }
                    }
                    // save the scene for forward simulation
                    _controlRigs[t].SaveState(_trajectorySamples[t]._startingSlotIndex);
                    _controlRigs[t].SaveState(_trajectorySamples[t]._cSlotStateIdx);
                    _trajectorySamples[t].ResetFlag = false;
                    _trajectorySamples[t].isActionDone = true;

                    if (_iter == 0)
                    {
                        startingStateIdx = _trajectorySamples[0]._startingSlotIndex;

                        _trajectorySamples[t].sourcePos.Clear();
                        _trajectorySamples[t].targetPos.Clear();
                        for (int _h = 0; _h < _trajectorySamples[t].targetHoldIds.Length; _h++)
                        {
                            if (_trajectorySamples[t].targetHoldIds[_h] >= 0)
                            {
                                _trajectorySamples[t].sourcePos.Add((ControlledPoses)(ControlledPoses.LeftLeg + _h));
                                _trajectorySamples[t].targetPos.Add(_controlRigs[t].GetContextManager().GetHoldPosition(_trajectorySamples[t].targetHoldIds[_h]));
                            }
                        }
                    }
                }
                else
                {
                    _controlRigs[t].LoadState(_trajectorySamples[t]._cSlotStateIdx);
                }
            }

            if (_iter == 0 && nOptimizationSamples > 0)
            {
                //startingStateIdx = _samplePlan._sampledInitialStateSlotIdx;
                //float[] state_feature = GetFeatureState(startingStateIdx, _samplePlan._sampledTargetStanceID, 0);

                //string network_data = "1|" + MyTools.ParseArrFloatInToString(state_feature);
                //string network_message = MyTools.ParseIntoString('Q', network_data);

                //if (mNetworkManager.IsConnected)
                //{
                //    int _sampleSize = nControlPoints * (1 + nAngle /*time + nAngle*/) + 4 /*time to let go of the hands and feet*/;
                //    string network_reply = mNetworkManager.Send(ref network_message);
                //    bool success_try = false;
                //    if (network_reply != "" && network_reply != "N/A")
                //    {
                //        MyTools.ParseStringIntoFloatArr(network_reply, ref predicted_policy_mean_std);
                //        success_try = true;
                //        if (predicted_policy_mean_std.Count == _sampleSize || predicted_policy_mean_std.Count == 2 * _sampleSize)
                //        {
                //            success_try = false;
                //        }
                //    }

                //    if (!success_try)
                //    {
                //        predicted_policy_mean_std.Clear();
                //    }
                //}
                //else
                //{
                predicted_policy_mean_std.Clear();
                //}
                //init opt
                InitOptimization();
            }

            return curNumTrajectories;
            // Init environment for forward simulation
            //if (_ReplayExpreinces)
            //// in this scope we only initialize previousely recorded exps
            //{
            //    for (int nt = nOptimizationSamples; nt < nOptimizationSamples + nNNEvalTrajectories; nt++)
            //    {
            //        if (_trajectorySamples[nt].ResetFlag)
            //        {
            //            int rnd_index_trajectory = Random.Range(0, initial_trajectory_points.Count);
            //            int state_in_trajectory_idx = Random.Range(0, (int)(initial_trajectory_points[rnd_index_trajectory]._fromToStates.Count / 5)) * 5;
            //            // loading scene and climber state
            //            for (int iid = 0; iid < 2; iid++)
            //            {
            //                int t = nt - iid * nOptimizationSamples;

            //                _controlRigs[t].loadState(initial_trajectory_points[rnd_index_trajectory]._fromToStates[state_in_trajectory_idx]);

            //                // loading optimized target stance
            //                for (int id = 0; id < 4; id++)
            //                {
            //                    _trajectorySamples[t].targetHoldIds[id] = initial_trajectory_points[rnd_index_trajectory].targetHoldIds[id];
            //                }

            //                if (t >= nOptimizationSamples)
            //                {
            //                    RandomizeHoldScence(t, initial_trajectory_points[rnd_index_trajectory]._fromToStates[state_in_trajectory_idx]);
            //                }

            //                // save the scene for forward simulation
            //                _controlRigs[t].saveState(_trajectorySamples[t]._startingSlotIndex);
            //                _controlRigs[t].saveState(_trajectorySamples[t]._cSlotStateIdx);

            //                _trajectorySamples[t].datum_index = rnd_index_trajectory;
            //                _trajectorySamples[t]._cSimulationStep = state_in_trajectory_idx;
            //                _trajectorySamples[t].ResetFlag = false;
            //                _trajectorySamples[t].isActionDone = true;

            //                // setting target pose for computing cost
            //                _trajectorySamples[t].sourcePos.Clear();
            //                _trajectorySamples[t].targetPos.Clear();
            //                for (int _h = 0; _h < _trajectorySamples[t].targetHoldIds.Length; _h++)
            //                {
            //                    if (_trajectorySamples[t].targetHoldIds[_h] >= 0)
            //                    {
            //                        _trajectorySamples[t].sourcePos.Add((ControlledPoses)(ControlledPoses.LeftLeg + _h));
            //                        _trajectorySamples[t].targetPos.Add(GetHoldPositionInContext(t, _trajectorySamples[t].targetHoldIds[_h]));
            //                    }
            //                }

            //                if (t < nOptimizationSamples)
            //                {
            //                    _trajectorySamples[t]._sampledMaxStep = initial_trajectory_points[rnd_index_trajectory]._sampledMaxStep;
            //                    for (int i = 0; i < bestPreSample._sampledAction.Length; i++)
            //                    {
            //                        _trajectorySamples[t]._sampledAction[i] = initial_trajectory_points[rnd_index_trajectory]._sampledAction[i];
            //                    }
            //                }
            //            }
            //        }
            //        else
            //        {
            //            for (int iid = 0; iid < 2; iid++)
            //            {
            //                int t = nt - iid * nOptimizationSamples;
            //                _controlRigs[t].loadState(_trajectorySamples[t]._cSlotStateIdx);
            //            }
            //        }
            //    }
            //}
            // in this scope we have both opt env + neural network env
            //else
            //{
            //// new starting state and stance target is set when opt _iter is zero
            //int initial_state_opt = _samplePlan._sampledInitialStateSlotIdx;

            //// if high-level planner is not used, use a simple cyclic movement planner as follows
            //if (initial_state_opt < 0)
            //{
            //    if (startingStateIdx < 0)
            //    {
            //        // start from t-pose
            //        initial_state_opt = reset_memory_idx;
            //    }
            //    else
            //    {
            //        initial_state_opt = startingStateIdx;
            //    }
            //}

            //// our opt should create a cyclic clip of climbing, which sample stance is coming from
            //for (int t = 0; t < nOptimizationSamples; t++)
            //{
            //    if (_trajectorySamples[t].ResetFlag)
            //    {
            //        if (t == 0)
            //        {
            //            _controlRigs[t].loadState(initial_state_opt);
            //            if (_iter == 0)
            //            {
            //                // only for _iter = 0 of opt we can set the target stance with their position
            //                RandomizeHoldScence(t, initial_state_opt);
            //                SampleRandomStanceIDs(t);
            //            }
            //        }
            //        else
            //        {
            //            _controlRigs[t].loadState(_trajectorySamples[0]._startingSlotIndex);
            //            for (int id = 0; id < 4; id++)
            //            {
            //                _trajectorySamples[t].targetHoldIds[id] = _trajectorySamples[0].targetHoldIds[id];
            //            }
            //        }
            //        // save the scene for forward simulation
            //        _controlRigs[t].saveState(_trajectorySamples[t]._startingSlotIndex);
            //        _controlRigs[t].saveState(_trajectorySamples[t]._cSlotStateIdx);
            //        _trajectorySamples[t].ResetFlag = false;
            //        _trajectorySamples[t].isActionDone = true;

            //        if (_iter == 0)
            //        {
            //            startingStateIdx = _trajectorySamples[0]._startingSlotIndex;

            //            _trajectorySamples[t].sourcePos.Clear();
            //            _trajectorySamples[t].targetPos.Clear();
            //            for (int _h = 0; _h < _trajectorySamples[t].targetHoldIds.Length; _h++)
            //            {
            //                if (_trajectorySamples[t].targetHoldIds[_h] >= 0)
            //                {
            //                    _trajectorySamples[t].sourcePos.Add((ControlledPoses)(ControlledPoses.LeftLeg + _h));
            //                    _trajectorySamples[t].targetPos.Add(GetHoldPositionInContext(t, _trajectorySamples[t].targetHoldIds[_h]));
            //                }
            //            }
            //        }
            //    }
            //    else
            //    {
            //        _controlRigs[t].loadState(_trajectorySamples[t]._cSlotStateIdx);
            //    }
            //}

            //// for neural network, each trajectory can have its own scene
            //for (int t = nOptimizationSamples; t < nOptimizationSamples + nNNEvalTrajectories; t++)
            //{
            //    if (_trajectorySamples[t].ResetFlag)
            //    {
            //        if (initial_trajectory_points.Count > 0)
            //        {
            //            int rnd_index_trajectory = Random.Range(0, initial_trajectory_points.Count);
            //            initial_state_opt = initial_trajectory_points[rnd_index_trajectory]._startingSlotIndex;
            //        }
            //        _controlRigs[t].loadState(initial_state_opt);
            //        // only for _iter = 0 of opt we can set the target stance with their position
            //        RandomizeHoldScence(t, initial_state_opt);
            //        SampleRandomStanceIDs(t);
            //        // save the scene for forward simulation
            //        _controlRigs[t].saveState(_trajectorySamples[t]._startingSlotIndex);
            //        _controlRigs[t].saveState(_trajectorySamples[t]._cSlotStateIdx);
            //        _trajectorySamples[t].ResetFlag = false;

            //        _trajectorySamples[t].sourcePos.Clear();
            //        _trajectorySamples[t].targetPos.Clear();
            //        for (int _h = 0; _h < _trajectorySamples[t].targetHoldIds.Length; _h++)
            //        {
            //            if (_trajectorySamples[t].targetHoldIds[_h] >= 0)
            //            {
            //                _trajectorySamples[t].sourcePos.Add((ControlledPoses)(ControlledPoses.LeftLeg + _h));
            //                _trajectorySamples[t].targetPos.Add(GetHoldPositionInContext(t, _trajectorySamples[t].targetHoldIds[_h]));
            //            }
            //        }
            //    }
            //    else
            //    {
            //        _controlRigs[t].loadState(_trajectorySamples[t]._cSlotStateIdx);
            //    }
            //}
            //}

            //List<float[]> trajetories_feature_states = new List<float[]>();
            //// get a couple of init state - proposed actions from neural net to evalue them
            //if (mNetworkManager.IsConnected && nNNEvalTrajectories > 0 && _ReplayExpreinces)
            //{
            //    for (int i = 0; i < nNNEvalTrajectories; i++)
            //    {
            //        float[] featureState = GetFeatureState(_trajectorySamples[nOptimizationSamples + i]._startingSlotIndex,
            //            _trajectorySamples[nOptimizationSamples + i].targetHoldIds, nOptimizationSamples + i);

            //        trajetories_feature_states.Add(featureState);
            //    }
            //    string snd_e_msg = MyTools.ParseIntoString('E', MyTools.ParseListArrFloatInToString(ref trajetories_feature_states));
            //    string network_reply = mNetworkManager.Send(ref snd_e_msg);

            //    if (network_reply != "" && !network_reply.Contains("N/A"))
            //    {
            //        MyTools.ParseStringIntoListFloatArr(ref network_reply, ref nnEvalReply);
            //    }
            //    string reply_to_net = 'A'.ToString();
            //    mNetworkManager.Send(ref reply_to_net);

            //    for (int i = 0; i < nNNEvalTrajectories; i++)
            //    {
            //        int cSampleNum = curNumTrajectories - nOptimizationSamples;
            //        _trajectorySamples[curNumTrajectories].datum_index = -1;

            //        if (nnEvalReply[cSampleNum].Count == 1 + 334 + 32 + _trajectorySamples[curNumTrajectories].sample_action_size + 3)
            //        {
            //            nnEvalStartingState[cSampleNum].Clear();
            //            nnEvalStartingHolds[cSampleNum].Clear();
            //            nnEvalPredictedPolicy[cSampleNum].Clear();

            //            int cIndex = 0;
            //            int datum_index = (int)nnEvalReply[cSampleNum][cIndex]; cIndex++;
            //            _trajectorySamples[curNumTrajectories].datum_index = datum_index;

            //            int cLength = (int)nnEvalReply[cSampleNum][cIndex]; cIndex++;
            //            for (int s = 0; s < cLength; s++) // 334
            //            {
            //                nnEvalStartingState[cSampleNum].Add(nnEvalReply[cSampleNum][cIndex]); cIndex++;
            //            }

            //            cLength = (int)nnEvalReply[cSampleNum][cIndex]; cIndex++;
            //            for (int s = 0; s < cLength; s++) // 32
            //            {
            //                nnEvalStartingHolds[cSampleNum].Add(nnEvalReply[cSampleNum][cIndex]); cIndex++;
            //            }

            //            cLength = (int)nnEvalReply[cSampleNum][cIndex]; cIndex++;
            //            for (int s = 0; s < cLength; s++) // 66
            //            {
            //                nnEvalPredictedPolicy[cSampleNum].Add(nnEvalReply[cSampleNum][cIndex]); cIndex++;
            //            }
            //            curNumTrajectories++;

            //        }
            //        else if (nnEvalReply[cSampleNum].Count == 1 + 1 + _trajectorySamples[curNumTrajectories].sample_action_size)
            //        {
            //            nnEvalPredictedPolicy[cSampleNum].Clear();

            //            int cIndex = 0;
            //            int datum_index = (int)nnEvalReply[cSampleNum][cIndex]; cIndex++;
            //            _trajectorySamples[curNumTrajectories].datum_index = datum_index;

            //            int cLength = (int)nnEvalReply[cSampleNum][cIndex]; cIndex++;
            //            for (int s = 0; s < cLength; s++)
            //            {
            //                nnEvalPredictedPolicy[cSampleNum].Add(nnEvalReply[cSampleNum][cIndex]); cIndex++;
            //            }
            //            curNumTrajectories++;
            //        }
            //        else
            //        {
            //            nnEvalReply[cSampleNum].Clear();
            //            if (_trajectorySamples[nOptimizationSamples + i]._startingSlotIndex >= 0)
            //                _controlRigs[nOptimizationSamples + i].loadState(_trajectorySamples[nOptimizationSamples + i]._startingSlotIndex);
            //        }
            //    }
            //}
            ////else
            ////{
            ////    curNumTrajectories = ReadFromFile();
            ////}

            //if (_iter == 0 && nOptimizationSamples > 0 && !_ReplayExpreinces)
            //{
            //    //startingStateIdx = _samplePlan._sampledInitialStateSlotIdx;
            //    float[] state_feature = GetFeatureState(startingStateIdx, _samplePlan._sampledTargetStanceID, 0);

            //    string network_data = "1|" + MyTools.ParseArrFloatInToString(state_feature);
            //    string network_message = MyTools.ParseIntoString('Q', network_data);

            //    if (mNetworkManager.IsConnected)
            //    {
            //        int _sampleSize = nControlPoints * (1 + nAngle /*time + nAngle*/) + 4 /*time to let go of the hands and feet*/;
            //        string network_reply = mNetworkManager.Send(ref network_message);
            //        bool success_try = false;
            //        if (network_reply != "" && network_reply != "N/A")
            //        {
            //            MyTools.ParseStringIntoFloatArr(network_reply, ref predicted_policy_mean_std);
            //            success_try = true;
            //            if (predicted_policy_mean_std.Count == _sampleSize || predicted_policy_mean_std.Count == 2 * _sampleSize)
            //            {
            //                success_try = false;
            //            }
            //        }

            //        if (!success_try)
            //        {
            //            predicted_policy_mean_std.Clear();
            //        }
            //    }
            //    else
            //    {
            //        predicted_policy_mean_std.Clear();
            //    }
            //    //init opt
            //    InitOptimization();
            //}

            //return curNumTrajectories;
        }

        // we create control points and re-start all trajectories
        void GenerateControlPoints(bool _isFirstItr)
        {
            //if (_ReplayExpreinces)
            //    return;

            // this function is just used for cmaes optimization with spline, and sampling for optimization trajectories
            for (int _t = 0; _t < nOptimizationSamples; _t++)
            {
                if (!_trajectorySamples[_t].isActionDone)
                    return;
            }

            if (!_isFirstItr)
                _opt.generateSamples(_samples);

            _maxActionStepsEachFrame = 0;
            int _sampleSize = nControlPoints * (1 + nAngle /*time + nAngle*/) + 4 /*time to let go of the hands and feet*/;
            for (int _t = 0; _t < nOptimizationSamples; _t++)
            {
                _trajectorySamples[_t].objectiveFuncVal = 0;
                _trajectorySamples[_t].isReached = false;

                for (int sIdx = 0; sIdx < _sampleSize; sIdx++)
                {
                    if (_t == 0 && !_isFirstItr)
                    {
                        _samples[_t].x[sIdx] = BestSample.x[sIdx];
                    }
                    //if (_t < nOptimizationSamples)
                    _samples[_t].x[sIdx] = Mathf.Clamp((float)_samples[_t].x[sIdx], minSampleVal, maxSampleVal);
                }

                ConvertToSampleAction(_t, true);

                if (_trajectorySamples[_t]._sampledMaxStep > _maxActionStepsEachFrame)
                    _maxActionStepsEachFrame = _trajectorySamples[_t]._sampledMaxStep;
            }
            _maxActionStepsEachFrame = 5;
            return;
        }

        bool ForwardSimulation(int curNumTrajectories)
        {
            bool flag_opt_trajetories_done = true;
            // To keep the efficiency of the code, every-time we want simulate _maxStep (e.g. 5) steps of a big trajectory
            for (int _sActionTime = 0; _sActionTime < _maxActionStepsEachFrame; _sActionTime++)
            {
                flag_opt_trajetories_done = true;
                for (int t = 0; t < curNumTrajectories; t++)
                {
                    /////////////////////////////////////////////////// 
                    bool flag_apply_one_step = (_trajectorySamples[t]._UseSpline
                        && _trajectorySamples[t]._cSimulationStep < _trajectorySamples[t]._sampledMaxStep)
                        || (!_trajectorySamples[t]._UseSpline && _sActionTime < _trajectorySamples[t]._sampledMaxStep);
                    if (flag_apply_one_step)
                    {
                        // simulation step is used for spline
                        ApplyControl(t, _trajectorySamples[t]._cSimulationStep);
                        // based on current state of climber, we decide to let go or attach the body parts to wall
                        _controlRigs[t].ConnectDisconnectContactPoint(_trajectorySamples[t].targetHoldIds, letGoControl[t]);

                        bool calculate_action_cost = (_trajectorySamples[t]._UseSpline
                                && _trajectorySamples[t]._cSimulationStep == _trajectorySamples[t]._sampledMaxStep - 1)
                            || (!_trajectorySamples[t]._UseSpline && _sActionTime == _trajectorySamples[t]._sampledMaxStep - 1);

                        float stateCost = ComputeStateCost(t, false, calculate_action_cost);
                        float controlCost = ComputeControlCost(t);

                        // saving state for animation
                        _controlRigs[t].SaveState(_trajectorySamples[t]._cSlotStateIdx);
                        _controlRigs[t].SaveState(_trajectorySamples[t]._fromToStates[_trajectorySamples[t]._cSimulationStep]);

                        bool flag_reset_costs = (_trajectorySamples[t]._UseSpline && _trajectorySamples[t]._cSimulationStep == 0)
                            || (!_trajectorySamples[t]._UseSpline && _sActionTime == 0);
                        if (flag_reset_costs)
                        {
                            _trajectorySamples[t].control_cost = controlCost;
                            _trajectorySamples[t].state_cost = stateCost;
                        }
                        else
                        {
                            _trajectorySamples[t].control_cost += controlCost;
                            _trajectorySamples[t].state_cost += stateCost;
                        }

                        _trajectorySamples[t].isReached =
                            MyTools.IsStanceAEqualStanceB(_trajectorySamples[t].targetHoldIds, _controlRigs[t].GetCurrentHoldIds());
                        _trajectorySamples[t]._cSimulationStep++;
                        // for keeping consistency in the synthesized trajectory
                        _trajectorySamples[t]._cMemoryStepUsed = _trajectorySamples[t]._cSimulationStep;

                        _trajectorySamples[t].isActionDone = calculate_action_cost;
                        if (t < nOptimizationSamples && !_trajectorySamples[t].isActionDone)
                            flag_opt_trajetories_done = false;
                    }
                    else
                    {
                        _trajectorySamples[t].isActionDone = true;
                        if (_trajectorySamples[t]._cSlotStateIdx >= 0)
                            _controlRigs[t].LoadState(_trajectorySamples[t]._cSlotStateIdx);
                    }
                    if (_trajectorySamples[t].isActionDone)
                    {
                        _trajectorySamples[t].objectiveFuncVal = -(_trajectorySamples[t].state_cost + _trajectorySamples[t].control_cost);
                        if (t < nOptimizationSamples)
                            _samples[t].objectiveFuncVal = _trajectorySamples[t].objectiveFuncVal;
                    }
                }

                Physics.Simulate(Time.fixedDeltaTime);
            }

            //if (flag_opt_trajetories_done)
            //{
            //    List<float> state_values = new List<float>();
            //    for (int t = 0; t < nOptimizationSamples; t++)
            //    {
            //        state_values.Add(_trajectorySamples[t].state_cost);
            //    }
            //}
            return flag_opt_trajetories_done;
        }

        void ApplyControl(int _targetContext, int step)
        {
            if (_trajectorySamples[_targetContext]._UseSpline)
            {
                // this function re-create spline given 'step' and (v,dv) at 'step' of spline
                // assuming we are at correct step of spline (i.e. spline v,dv are set properly)
                //, then given 'step' we can re-create same spline

                // maxStep calculated such that step is not violating segmentID
                float[] sample = _trajectorySamples[_targetContext]._sampledAction;
                int nValuesPerSegment = 1 + nAngle;

                // total passed time
                float totalTime = step * Time.fixedDeltaTime;

                // restarting segmentIDs in step = 0
                int segmentIdx = 0;
                if (step == 0)
                    segmentIdx = 0;
                //segmentIdTrajectories[_targetContext] = 0;
                else
                {
                    float s_t1 = sample[0 * nValuesPerSegment];
                    float s_t2 = sample[1 * nValuesPerSegment];
                    if (s_t1 < totalTime)
                        segmentIdx = 1;
                    //segmentIdTrajectories[_targetContext] = 1;
                    else if (s_t1 + s_t2 < totalTime)
                        segmentIdx = 2;
                    //segmentIdTrajectories[_targetContext] = 2;
                }
                //int segmentIdx = segmentIdTrajectories[_targetContext];

                // calculating t1 based on all segments and total passed time
                float t1 = 0.0f;
                for (int _sID = 0; _sID <= segmentIdx; _sID++)
                    t1 += sample[segmentIdx * nValuesPerSegment];
                t1 = t1 - totalTime;

                float t2 = t1 + sample[Mathf.Max(segmentIdx + 1, nControlPoints - 1) * nValuesPerSegment];
                if (t1 < 0)
                {
                    t1 = t2 - Time.fixedDeltaTime;
                    //segmentIdTrajectories[_targetContext] += 1;
                    //segmentIdx = segmentIdTrajectories[_targetContext];
                    segmentIdx++;

                    t2 = t1 + sample[Mathf.Max(segmentIdx + 1, nControlPoints - 1) * nValuesPerSegment];
                }

                for (int i = 0; i < nAngle; i++)
                {
                    float p1 = sample[segmentIdx * nValuesPerSegment + 1 + i], p2 = sample[Mathf.Max(segmentIdx + 1, nControlPoints - 1) * nValuesPerSegment + 1 + i];
                    _controlRigs[_targetContext].spline[i].step(Time.fixedDeltaTime, p1, t1, p2, t2);
                    _controlRigs[_targetContext].spline[i].setCurrentValue(Mathf.Clamp(_controlRigs[_targetContext].spline[i].currentValue, _controlRigs[_targetContext].minAngle[i], _controlRigs[_targetContext].maxAngle[i]));
                    interpolatedControl[_targetContext][i] = _controlRigs[_targetContext].spline[i].currentValue;
                }

                _controlRigs[_targetContext].driveToPose(interpolatedControl[_targetContext]);

                // apply connection control
                for (int id = 0; id < 4; id++)
                {
                    letGoControl[_targetContext][id] = totalTime > sample[sample.Length - 1 - id];
                }
            }
            else
            {
                for (int i = 0; i < nAngle; i++)
                {
                    interpolatedControl[_targetContext][i] = Mathf.Clamp(_trajectorySamples[_targetContext]._sampledAction[i], _controlRigs[_targetContext].minAngle[i], _controlRigs[_targetContext].maxAngle[i]);
                }
                _controlRigs[_targetContext].driveToPose(interpolatedControl[_targetContext]);
                for (int id = 0; id < 4; id++)
                {
                    //int sID = nAngle + id;
                    letGoControl[_targetContext][id] = true;// _trajectorySamples[_targetContext].failed_counter > _trajectorySamples[_targetContext]._sampledAction[sID];
                }
            }
            return;
        }

        //void ConnectDisconnectContactPoint(int targetContext)
        //{
        //    bool[] _allowRelease = letGoControl[targetContext];
        //    int[] targetHoldIds = _trajectorySamples[targetContext].targetHoldIds;
        //    bool isHandsDisconnected = _controlRigs[targetContext].getCurrentHoldBodyID(2) == -1 && _controlRigs[targetContext].getCurrentHoldBodyID(3) == -1;

        //    if (isHandsDisconnected)
        //    {
        //        _controlRigs[targetContext].counterHandsDisconnected++;
        //    }
        //    else
        //    {
        //        _controlRigs[targetContext].counterHandsDisconnected = 0;
        //    }

        //    for (int i = 0; i < targetHoldIds.Length; i++)
        //    {
        //        if (targetHoldIds[i] != -1)
        //        {
        //            Vector3 hold_pos_i = GetHoldPositionInContext(targetContext, targetHoldIds[i]);
        //            Vector3 contact_pos_i = _controlRigs[targetContext].getEndBonePosition(i);
        //            float dis_i = (hold_pos_i - contact_pos_i).magnitude;

        //            float _connectionThreshold = 0.5f * ContextManager.HoldSize;

        //            if (dis_i <= _connectionThreshold)
        //            {
        //                _controlRigs[targetContext].connectEndBoneToHold(i, targetHoldIds[i]);
        //            }
        //            // safty measure or allowing to let go of the hold
        //            else if ((dis_i > _connectionThreshold + 0.1f) || (targetHoldIds[i] != _controlRigs[targetContext].getCurrentHoldBodyID(i) && _allowRelease[i]))
        //            {
        //                _controlRigs[targetContext].disconnectEndBone(i);
        //            }
        //        }
        //        else
        //        {
        //            if (_allowRelease[i])
        //            {
        //                _controlRigs[targetContext].disconnectEndBone(i);
        //            }
        //        }
        //    }

        //    if (_controlRigs[targetContext].counterHandsDisconnected > maxCountHandsDisconnected)
        //    {
        //        for (int i = 0; i <= 1; i++)
        //        {
        //            _controlRigs[targetContext].disconnectEndBone(i);
        //        }
        //        _controlRigs[targetContext].counterHandsDisconnected = 0;
        //    }

        //    return;
        //}

        float ComputeControlCost(int targetContext)
        {
            if (_UsePPOReward)
            {
                return 0.0f;
            }
            const bool calculateForceOnBodies = true;
            float result = 0;

            float elbowTorqueSd = 16.0f * 9.81f * _controlRigs[targetContext].boneSize(HumanBodyBones.LeftLowerArm);
            float legTorqueSd = 70.0f * 9.81f * _controlRigs[targetContext].boneSize(HumanBodyBones.LeftUpperLeg);

            for (int j = 0; j < _controlRigs[targetContext].numControlDOFs(); j++)
            {
                HumanBodyBones b_i = _controlRigs[targetContext].getBodyFromJointIndex(j).mecanimBone;

                float torqueSD = forceCostSd;
                float tourque_value = _controlRigs[targetContext].getMotorAppliedSqTorque(j);
                switch (b_i)
                {
                    case HumanBodyBones.LeftUpperArm:
                    case HumanBodyBones.RightUpperArm:
                        torqueSD = elbowTorqueSd;
                        break;
                    case HumanBodyBones.LeftLowerArm:
                    case HumanBodyBones.RightLowerArm:
                        torqueSD = elbowTorqueSd;
                        break;
                    case HumanBodyBones.LeftHand:
                    case HumanBodyBones.RightHand:
                        torqueSD = elbowTorqueSd;
                        break;
                    case HumanBodyBones.Chest:
                    case HumanBodyBones.Hips:
                    case HumanBodyBones.UpperChest:
                        break;
                    case HumanBodyBones.LeftUpperLeg:
                    case HumanBodyBones.RightUpperLeg:
                        torqueSD = legTorqueSd;
                        break;
                    case HumanBodyBones.LeftLowerLeg:
                    case HumanBodyBones.RightLowerLeg:
                        torqueSD = legTorqueSd;
                        break;
                    case HumanBodyBones.LeftFoot:
                    case HumanBodyBones.RightFoot:
                        torqueSD = 100.0f * 9.81f * _controlRigs[targetContext].boneSize(HumanBodyBones.LeftFoot);
                        break;
                    default:
                        break;
                }

                result += (tourque_value) / (torqueSD * torqueSD);
            }

            if (calculateForceOnBodies)
            {
                float elbowForceSd = 16.0f * 9.81f;
                float legForceSd = 70.0f * 9.81f;
                for (int b = 0; b < _controlRigs[targetContext].getNumBones(); b++)
                {
                    HumanBodyBones b_i = _controlRigs[targetContext].getBodyFromBoneIdx(b).mecanimBone;

                    float forceSD = forceCostSd;
                    float force_value = _controlRigs[targetContext].getMotorAppliedSqForce(b);
                    switch (b_i)
                    {
                        case HumanBodyBones.LeftUpperArm:
                        case HumanBodyBones.RightUpperArm:
                            forceSD = elbowForceSd;
                            break;
                        case HumanBodyBones.LeftLowerArm:
                        case HumanBodyBones.RightLowerArm:
                            forceSD = elbowForceSd;
                            break;
                        case HumanBodyBones.LeftHand:
                        case HumanBodyBones.RightHand:
                            forceSD = elbowForceSd;
                            break;
                        case HumanBodyBones.Chest:
                        case HumanBodyBones.Hips:
                        case HumanBodyBones.UpperChest:
                            break;
                        case HumanBodyBones.LeftUpperLeg:
                        case HumanBodyBones.RightUpperLeg:
                            forceSD = legForceSd;
                            break;
                        case HumanBodyBones.LeftLowerLeg:
                        case HumanBodyBones.RightLowerLeg:
                            forceSD = legForceSd;
                            break;
                        case HumanBodyBones.LeftFoot:
                        case HumanBodyBones.RightFoot:
                            forceSD = 100.0f * 9.81f;
                            break;
                        default:
                            break;
                    }

                    result += (force_value) / (forceSD * forceSD);
                }

                result /= 2.0f; // energy used on body calculated twice (in two ways!)
            }
            float handForceSd = 70.0f * 9.81f;
            result += (_controlRigs[targetContext].GetSqForceOnFingers() / (handForceSd * handForceSd));

            return result;
        }

        float ComputeStateCost(int _trajectoryIdx, bool printDebug, bool isLastStep)
        {
            //        // we assume that we are in the correct simulation context using restoreOdeState
            //        float trunkDistSd = 20.0f;
            //float angleSd = poseAngleSd;  //loose prior, seems to make more natural movement
            //float tPoseAngleSd = 0.1f;
            float tPoseSd = 60f * Mathf.Deg2Rad;

            float endEffectorDistSd = 0.0025f;
            float velSd = 100.0f;  //velSd only needed for C-PBP to reduce noise
            float chestDirSd = 0.05f;

            if (_UsePPOReward)
            {
                //int ref_trajectory_idx = _trajectoryIdx - nOptimizationSamples;

                //float pose_rew = 0, vel_rew = 0, end_effector_ref_rew = 0, end_effector_target, com_rew = 0;

                //Vector3 com = _controlRigs[_trajectoryIdx].COM();
                ////Vector3 target_dir = new Vector3(0, 0, 1);
                ////Vector3 initialDir(-1, 0, 0);
                ////Quaternion targetRotation = Quaternion.FromToRotation(initialDir, target_dir);
                ////Quaternion stateRotation = Quaternion.Inverse(_controlRigs[_trajectoryIdx].getBoneAngle(0));

                //if (ref_trajectory_idx >= 0)
                //{
                //    for (int i = 0; i < 15; ++i)
                //    {
                //        Vector3 avel = _controlRigs[_trajectoryIdx].getBoneAVel(i);
                //        //Vector3 avel = stateRotation * avel_tmp;

                //        Quaternion q = _controlRigs[_trajectoryIdx].getBoneAngle(i);
                //        //Quaternion q = stateRotation * q_tmp;

                //        float tmp = MyTools.GetAngularDistance(q, _controlRigs[ref_trajectory_idx].getBoneAngle(i));
                //        if (float.IsNaN(tmp))
                //        {
                //            int notifyme = 1;
                //        }
                //        pose_rew += MyTools.GetSquared(tmp);
                //        vel_rew += (avel - _controlRigs[ref_trajectory_idx].getBoneAVel(i)).sqrMagnitude;
                //    }
                //    for (int i = 0; i < 4; ++i)
                //    {
                //        end_effector_ref_rew += (_controlRigs[ref_trajectory_idx].GetEndBonePosition(i) - _controlRigs[_trajectoryIdx].GetEndBonePosition(i)).sqrMagnitude;
                //    }
                //    com_rew = (_controlRigs[ref_trajectory_idx].COM() - com).sqrMagnitude;
                //}

                //Vector2 _dis_sDis = GetDisCurEndPointToTarget(_trajectorySamples[_trajectoryIdx]._cSlotStateIdx, _trajectoryIdx);
                //end_effector_target = _dis_sDis[0];

                //vel_rew = Mathf.Exp(-1e-3f * vel_rew);
                //pose_rew = Mathf.Exp(-1e-1f * pose_rew);
                //end_effector_ref_rew = Mathf.Exp(-40.0f * end_effector_ref_rew);
                //end_effector_target = Mathf.Exp(-4.0f * end_effector_target);
                //com_rew = Mathf.Exp(-10.0f * com_rew);

                //const float pose_w = 0.65f,
                //            vel_w = 0.1f,
                //            end_effector_ref_w = 0.15f,
                //            com_w = 0.1f;

                //float rew_follow_ref = pose_w * pose_rew
                //            + vel_w * vel_rew
                //            + end_effector_ref_w * end_effector_ref_rew
                //            + com_w * com_rew;

                //if (_trajectoryIdx >= nOptimizationSamples)
                //{
                //    if (_dis_sDis[0] > (2.0f - _PercentFollowRef) * _dis_sDis[1])
                //        _trajectorySamples[_trajectoryIdx].ResetFlag = true;
                //}

                //float rew = rew_follow_ref * 0.2f + 0.8f * end_effector_target;

                //if (isLastStep)
                //    return -rew;
                //return 0.0f;
            }
            int[] targetHoldIDs = _trajectorySamples[_trajectoryIdx].targetHoldIds;
            float stateCost = 0;
            float preStateCost = 0;

            /////////////////////////////////////////////// Velocity
            for (int k = 0; k < _controlRigs[_trajectoryIdx].getNumBones(); k++)
            {
                stateCost += (_controlRigs[_trajectoryIdx].getBoneVel(k) / velSd).sqrMagnitude;
            }

            if (printDebug)
            {
                UnityEngine.Debug.Log("Velocity cost: " + (stateCost - preStateCost).ToString());
                preStateCost = stateCost;
            }

            ///////////////////////////////////////////// Posture
            if (isLastStep)
            {
                //float[] readPose = new float[_controlRigs[_trajectoryIdx].numControlDOFs()];
                //_controlRigs[_trajectoryIdx].poseToMotorAngles(ref readPose);
                if (_controlRigs[_trajectoryIdx].GetCurrentHoldBodyID(2) == -1 && _controlRigs[_trajectoryIdx].GetCurrentHoldBodyID(3) == -1)
                {
                    for (int k = 0; k < _controlRigs[_trajectoryIdx].numControlDOFs(); k++) // all vectors are relative to bone 0
                    {
                        float _diff = (-_controlRigs[_trajectoryIdx].spline[k].currentValue) * Mathf.Deg2Rad;//readPose[k]
                        stateCost += (_diff * _diff) / (tPoseSd * tPoseSd);
                    }
                }
                else
                {
                    for (int k = 0; k < _controlRigs[_trajectoryIdx].numControlDOFs(); k++) // all vectors are relative to bone 0
                    {
                        float _diff = (_desiredClimbingAnglePos[k] - _controlRigs[_trajectoryIdx].spline[k].currentValue) * Mathf.Deg2Rad;//readPose[k]
                        stateCost += (_diff * _diff) / (tPoseSd * tPoseSd);
                    }
                }

                if (printDebug)
                {
                    UnityEngine.Debug.Log("Pose cost " + (stateCost - preStateCost).ToString());
                    preStateCost = stateCost;
                }
            }

            bool isTorsoDirAdded = false;
            for (int i = 0; i < _trajectorySamples[_trajectoryIdx].sourcePos.Count; i++)
            {
                ControlledPoses posID = _trajectorySamples[_trajectoryIdx].sourcePos[i];
                Vector3 dPos = _trajectorySamples[_trajectoryIdx].targetPos[i];

                float cWeight = 1.0f;

                Vector3 cPos = new Vector3(0, 0, 0);
                if ((int)posID <= (int)ControlledPoses.RightHand && (int)posID >= (int)ControlledPoses.LeftLeg)
                {
                    if (isLastStep)
                    {
                        cPos = _controlRigs[_trajectoryIdx].GetEndBonePosition(posID - ControlledPoses.LeftLeg);
                        cWeight = 1.0f / endEffectorDistSd;

                        int cHoldID = _controlRigs[_trajectoryIdx].GetCurrentHoldBodyID(posID - ControlledPoses.LeftLeg);
                        if (cHoldID == targetHoldIDs[(int)(posID - ControlledPoses.LeftLeg)])
                        {
                            cWeight = 0.0f;
                        }
                    }
                    else
                    {
                        cWeight = 0.0f;
                    }
                }
                //        else if (posID == ControlledPoses::MiddleTrunk)
                //        {
                //            cPos = iContextCPBP->computeCOM();
                //            dPos[0] = cPos[0];//only distance from wall matters
                //            dPos[2] = cPos[2];

                //            if (DemoID == mDemoTestClimber::DemoPillar)
                //            {
                //                dPos = minGeomPos;
                //                dPos[2] = cPos[2];
                //            }

                //            cWeight = 1 / trunkDistSd; //weight_average_important;
                //        }
                //        else if (posID == ControlledPoses::TorsoDir)
                //        {
                //            // user defined direction
                //            Vector3 dirTrunk = iContextCPBP->getBodyDirectionY(SimulationContext::BodyName::BodyTrunk);
                //            cWeight = 1 / chestDirSd;
                //            cPos = dirTrunk;

                //            if (DemoID == mDemoTestClimber::DemoPillar)
                //            {
                //                dPos = (minGeomPos - posTrunk).normalized();
                //            }

                //            isTorsoDirAdded = true;
                //        }

                stateCost += (cWeight * (cPos - dPos)).sqrMagnitude;

                if (printDebug)
                {
                    if (posID == ControlledPoses.LeftLeg)
                    {
                        UnityEngine.Debug.Log("Left leg cost " + (stateCost - preStateCost).ToString());
                    }
                    else if (posID == ControlledPoses.RightLeg)
                    {
                        UnityEngine.Debug.Log("Right leg cost " + (stateCost - preStateCost).ToString());
                    }
                    else if (posID == ControlledPoses.LeftHand)
                    {
                        UnityEngine.Debug.Log("Left hand cost " + (stateCost - preStateCost).ToString());
                    }
                    else if (posID == ControlledPoses.RightHand)
                    {
                        UnityEngine.Debug.Log("Right hand cost " + (stateCost - preStateCost).ToString());
                    }
                    else if (posID == ControlledPoses.MiddleTrunk)
                    {
                        UnityEngine.Debug.Log("Torso pos cost " + (stateCost - preStateCost).ToString());
                    }
                    else if (posID == ControlledPoses.TorsoDir)
                    {
                        UnityEngine.Debug.Log("Chest direction cost " + (stateCost - preStateCost).ToString());
                    }
                    preStateCost = stateCost;
                }
            }

            // some default direction for torso
            if (!isTorsoDirAdded)
            {
                Vector3 dirTrunk = new Vector3(0, 1, 0); //iContextCPBP->getBodyDirectionY(SimulationContext::BodyName::BodyTrunk);
                Vector3 desDir = new Vector3(0, 1, 0);

                stateCost += ((dirTrunk - desDir) / chestDirSd).sqrMagnitude; // chest toward the wall
                if (printDebug)
                {
                    UnityEngine.Debug.Log("Chest direction cost " + (stateCost - preStateCost).ToString());
                    preStateCost = stateCost;
                }
            }

            if (float.IsNaN(stateCost))
            {
                stateCost = float.MaxValue;
            }

            //if (isLastStep && stateCost < 1.0f)
            //{
            //    if (MyTools.GetDiffBtwSetASetB(_controlRigs[_trajectoryIdx].getCurrentHoldIds(), _trajectorySamples[_trajectoryIdx].targetHoldIds) > 0)
            //    {
            //        int notifyme = 1;
            //    }
            //}
            return stateCost;
        }

        public bool OptimizeCost(ref SamplingHighLevelPlan _samplePlan)
        {
            if (_PlayAnimation)
            {
                bool flag_start_over = true;
                for (int t = 0; t < nOptimizationSamples + nNNEvalTrajectories; t++)
                {
                    if (curShownState < _trajectorySamples[t]._cMemoryStepUsed
                        && curShownState < _trajectorySamples[t]._fromToStates.Count
                        && _trajectorySamples[t]._fromToStates[curShownState] >= 0)
                    {
                        _controlRigs[t].LoadState(_trajectorySamples[t]._fromToStates[curShownState]);
                        flag_start_over = false;
                    }
                    else
                    {
                        if (_trajectorySamples[t]._cSlotStateIdx >= 0)
                            _controlRigs[t].LoadState(_trajectorySamples[t]._cSlotStateIdx);
                        else
                            _controlRigs[t].LoadState(reset_memory_idx);
                    }
                }
                curShownState += 3;
                if (flag_start_over)
                    curShownState = 0;
            }
            else
            {
                int curNumTrajectories = InitTrajectories(ref _samplePlan);
                // in each iteration we generate control points, in iter = 0 we generate them from uniform dist otherwise it is generated from cma-es
                GenerateControlPoints(_iter == 0);

                // Update actions for nn evaluation
                for (int _t = nOptimizationSamples; _t < curNumTrajectories; _t++)
                {
                    ConvertToSampleAction(_t, false);
                }

                bool flag_opt_samples_done = ForwardSimulation(curNumTrajectories);

                // update trajectory info
                for (int t = nOptimizationSamples; t < curNumTrajectories; t++)
                {
                    if (_trajectorySamples[t].isActionDone)
                    {
                        if (_trajectorySamples[t]._cSimulationStep >= MaxStepsInOptSample
                            || _trajectorySamples[t]._UseSpline
                            || _trajectorySamples[t].isReached)
                        {
                            _trajectorySamples[t]._cMemoryStepUsed = _trajectorySamples[t]._cSimulationStep;
                            _trajectorySamples[t]._cSimulationStep = 0;
                            // send signal that trajectory is finished, restart for another trajectory
                            _trajectorySamples[t].ResetFlag = true;
                        }
                    }
                }

                if (nOptimizationSamples > 0 && flag_opt_samples_done && !_ReplayExpreinces)
                {
                    // here one iter of opt is done
                    // reset opt samples
                    for (int t = 0; t < nOptimizationSamples; t++)
                    {
                        _trajectorySamples[t]._cMemoryStepUsed = _trajectorySamples[t]._cSimulationStep;
                        _trajectorySamples[t]._cSimulationStep = 0;
                        _trajectorySamples[t].ResetFlag = true;
                    }
                    // update cma-es mean and variance
                    _opt.update(_samples);

                    //bestPreSample.objectiveFuncVal = _trajectorySamples[0].objectiveFuncVal;

                    int _bestTrajectoryIdx = -1;
                    double _opt_obj_value = float.MinValue;
                    if (nOptimizationSamples > 0)
                    {
                        _bestTrajectoryIdx = _opt.getBestIndex();
                        _opt_obj_value = _opt.getBestObjectiveFuncValue();
                    }
                    else if (curNumTrajectories > 0)
                    {
                        /////////////////////////////////////// TODO: if we do not have any opt, search for best manually
                        _bestTrajectoryIdx = 0;
                    }
                    // store best sample
                    if (_bestTrajectoryIdx > -1)
                    {
                        int _sampleSize = nControlPoints * (1 + nAngle /*time + nAngle*/) + 4 /*time to let go of the hands and feet*/;
                        if (_iter == 0 || _opt_obj_value > bestPreSample.objectiveFuncVal + _fixedThresholdCost)
                        {
                            if (_iter == 0)
                            {
                                neuralNetSampleVal = _trajectorySamples[0].objectiveFuncVal;
                            }

                            if (nOptimizationSamples > 0)
                            {
                                BestSample.objectiveFuncVal = _opt.getBestObjectiveFuncValue();
                                for (int c = 0; c < _sampleSize; c++)
                                    BestSample.x[c] = _opt.getBest()[c];
                            }

                            bestPreSample.objectiveFuncVal = _trajectorySamples[_bestTrajectoryIdx].objectiveFuncVal;
                            bestPreSample.state_cost = _trajectorySamples[_bestTrajectoryIdx].state_cost;
                            bestPreSample.control_cost = _trajectorySamples[_bestTrajectoryIdx].control_cost;
                            bestPreSample._sampledMaxStep = _trajectorySamples[_bestTrajectoryIdx]._sampledMaxStep;
                            for (int i = 0; i < _sampleSize; i++)
                                bestPreSample._sampledAction[i] = _trajectorySamples[_bestTrajectoryIdx]._sampledAction[i];

                            for (int i = 0; i < 4; i++)
                            {
                                bestPreSample.targetHoldIds[i] = _trajectorySamples[_bestTrajectoryIdx].targetHoldIds[i];
                            }

                            bestPreSample._sampledMaxStep = _trajectorySamples[_bestTrajectoryIdx]._sampledMaxStep;

                            mMemory.SaveState(bestPreSample._cSlotStateIdx, _trajectorySamples[_bestTrajectoryIdx]._cSlotStateIdx);
                            mMemory.SaveState(bestPreSample._startingSlotIndex, _trajectorySamples[_bestTrajectoryIdx]._startingSlotIndex);
                            for (int _s = 0; _s < _trajectorySamples[_bestTrajectoryIdx]._sampledMaxStep; _s++)
                            {
                                mMemory.SaveState(bestPreSample._fromToStates[_s], _trajectorySamples[_bestTrajectoryIdx]._fromToStates[_s]);
                            }

                            bestPreSample.isReached = _trajectorySamples[_bestTrajectoryIdx].isReached;
                        }
                    }

                    //debuggingText.text += ", state cost:" + (_trajectorySamples[_bestTrajectoryIdx].state_cost).ToString("f3");// UnityEngine.Debug.Log();

                    _iter++;
                    if (_opt.getBestObjectiveFuncValue() - bestPreSample.objectiveFuncVal > _fixedThresholdCost)
                    {
                        _fixedIter = 0;
                    }
                    else
                    {
                        _fixedIter++;
                    }
                }

                _controlRigs[GetMasterTrajectoryIdx()].LoadState(bestPreSample._cSlotStateIdx);

                //int reached_eval = 0;
                if (mNetworkManager.IsConnected)
                {
                    string snd_msg = (curNumTrajectories - nOptimizationSamples).ToString() + "|";

                    for (int t = nOptimizationSamples; t < curNumTrajectories; t++)
                    {
                        //if (_trajectorySamples[t].isReached)
                        //    AddToRandomInitStates(_trajectorySamples[t]._cSlotStateIdx);

                        int is_reached = _trajectorySamples[t].isReached ? 1 : 0;
                        //Vector2 ret_dis_startingDis = GetDisCurEndPointToTarget(_trajectorySamples[t]._cSlotStateIdx, t);
                        float reward = -(_trajectorySamples[t].state_cost + _trajectorySamples[t].control_cost);
                        int is_trajectory_done = _trajectorySamples[t].ResetFlag ? 1 : 0;
                        int is_addable = 1;

                        snd_msg += is_trajectory_done.ToString()
                            + "|" + is_addable.ToString()
                            + "|" + reward.ToString()
                            + "|" + is_reached.ToString();
                        if (t != curNumTrajectories - 1)
                        {
                            snd_msg += "|";
                        }
                    }

                    string snd_v_msg = MyTools.ParseIntoString('V', snd_msg);
                    mNetworkManager.Send(ref snd_v_msg);
                }
                else
                {
                    WriteToFile(curNumTrajectories);
                }

                //int valid_num_trajectories = 0;
                //for (int t = nOptimizationSamples; t < curNumTrajectories; t++)
                //{
                //    int[] target_holds = _trajectorySamples[t].targetHoldIds;
                //    int[] cur_holds = _controlRigs[t].getCurrentHoldIds();
                //    if (MyTools.GetDiffBtwSetASetB(target_holds, cur_holds) <= 1)
                //    {
                //        valid_num_trajectories++;
                //    }
                //    if (MyTools.IsStanceAEqualStanceB(target_holds, cur_holds))
                //    {
                //        reached_eval++;
                //    }
                //}
                //if (valid_num_trajectories > 0)
                //    Debug.Log("Eval:" + ((reached_eval + 0.0f) / (valid_num_trajectories + 0.001f)).ToString("f3") + ", state:"
                //        + _trajectorySamples[_bestTrajectoryIdx].state_cost.ToString("f3") + ", control:" + _trajectorySamples[_bestTrajectoryIdx].control_cost.ToString("f3"));

            }

            if ((_iter > _maxOptIter || _fixedIter > _maxFixedOptIter) && _iter > 0 && !_PlayAnimation)
            {
                //finalize opt
                EndOptimization();
                return true; // opt is done
            }

            return false; // opt is not done
        }

        //// trajectory_idx for loading holds ids and positions
        Vector2 GetDisCurEndPointToTarget(int _CurSlotIdx, int trajectory_idx)
        {
            float _dis = 0.0f;

            int[] tholdIDs = _trajectorySamples[trajectory_idx].targetHoldIds;

            int[] sholdIds = { -1, -1, -1, -1 };
            List<Vector3> s_poses = new List<Vector3>();
            mMemory.GetHoldIds(_trajectorySamples[trajectory_idx]._startingSlotIndex, ref sholdIds);
            mMemory.GetEndPointPoses(_trajectorySamples[trajectory_idx]._startingSlotIndex, ref s_poses);

            int[] choldIds = { -1, -1, -1, -1 };
            List<Vector3> e_poses = new List<Vector3>();
            mMemory.GetHoldIds(_CurSlotIdx, ref choldIds);
            mMemory.GetEndPointPoses(_CurSlotIdx, ref e_poses);


            int _diff = 0;
            float starting_dis = 0.0f;
            for (int i = 0; i < 4; i++)
            {
                if (tholdIDs[i] != choldIds[i] || tholdIDs[i] != sholdIds[i])
                {
                    _diff++;
                    Vector3 tHPos = _controlRigs[trajectory_idx].GetContextManager().GetHoldPosition(tholdIDs[i]);
                    if (tholdIDs[i] != choldIds[i])
                    {
                        _dis += (e_poses[i] - tHPos).magnitude;
                    }

                    if (tholdIDs[i] != sholdIds[i])
                    {
                        starting_dis += (s_poses[i] - tHPos).magnitude;
                    }
                }
            }

            return new Vector2(_dis, starting_dis);
        }

        //    void RandomizeHoldScence(int _targetContext, int random_initial_state_idx)
        //    {
        //        return;
        //        List<int> rnd_hold_ids = new List<int>(new int[] { 0, 1, 2, 3, 4, 5, 6, 7 });

        //        for (int i = 0; i < 4; i++)
        //        {
        //            rnd_hold_ids.Remove(mMemory.GetCurrentHoldId(random_initial_state_idx, i));
        //        }

        //        for (int i = 0; i < rnd_hold_ids.Count; i++)
        //        {
        //            int h = rnd_hold_ids[i];
        //            int row = (int)(h / 2);
        //            int col = h % 2;

        //            Vector3 nPos = new Vector3(0.325f + (col - 1) * 0.75f, row * 0.5f + 0.25f, 1.525f);
        //            // add noise
        //            nPos += new Vector3(Random.Range(-0.5f, 0.5f), Random.Range(-0.5f, 0.5f), 0.0f).normalized * Random.Range(0.0f, _MaxSampleDis);
        //            SetPositionInContextForHold(_targetContext, h, nPos);
        //        }

        //        return;
        //    }

        //    void SampleRandomStanceIDs(int _t)
        //    {
        //        return;
        //        switch (initial_trajectory_points.Count)
        //        {
        //            case 0:
        //                _trajectorySamples[_t].targetHoldIds[3] = 5;
        //                return;
        //                break;
        //            case 1:
        //                _trajectorySamples[_t].targetHoldIds[2] = 4;
        //                return;
        //                break;
        //            case 2:
        //                _trajectorySamples[_t].targetHoldIds[1] = 1;
        //                return;
        //                break;
        //            case 3:
        //                _trajectorySamples[_t].targetHoldIds[0] = 0;
        //                return;
        //                break;
        //            default:
        //                break;
        //        }

        //        ///////////////////////////// random body movement
        //        int numHandsConnected = (_controlRigs[_t].getCurrentHoldBodyID(2) > -1) ? 1 : 0;
        //        numHandsConnected += (_controlRigs[_t].getCurrentHoldBodyID(3) > -1) ? 1 : 0;

        //        int numLegsConnected = (_controlRigs[_t].getCurrentHoldBodyID(0) > -1) ? 1 : 0;
        //        numLegsConnected += (_controlRigs[_t].getCurrentHoldBodyID(1) > -1) ? 1 : 0;
        //        int[] body_ids = { 0, 1, 2, 3 };

        //        if (numHandsConnected == 0)
        //        {
        //            body_ids = new int[] { 2, 3 };
        //        }
        //        else if (numHandsConnected == 2 && numLegsConnected == 0)
        //        {
        //            body_ids = new int[] { 0, 1 };
        //        }
        //        else if (numHandsConnected == 1)
        //        {
        //            // move the other hand if the connected hand is chosen to be moved
        //            if (_controlRigs[_t].getCurrentHoldBodyID(2) > -1)
        //            {
        //                body_ids = new int[] { 0, 1, 3 };
        //            }
        //            if (_controlRigs[_t].getCurrentHoldBodyID(3) > -1)
        //            {
        //                body_ids = new int[] { 0, 1, 2 };
        //            }
        //        }
        //        int rnd_body_id = body_ids[(int)Random.Range((int)0, (int)body_ids.Length)];

        //        ////////////////////////////// find closest hold
        //        for (int b = 0; b < 4; b++)
        //        {
        //            _trajectorySamples[_t].targetHoldIds[b] = _controlRigs[_t].getCurrentHoldBodyID(b);
        //        }
        //        int current_hold = _controlRigs[_t].getCurrentHoldBodyID(rnd_body_id);

        //        int closest_hold_id = -1;
        //        float minDis = float.MaxValue;
        //        for (int h = 0; h < 8; h++)
        //        {
        //            if (h == current_hold)
        //                continue;

        //            Vector3 hPos = GetHoldPositionInContext(_t, h);
        //            float dis = (hPos - _controlRigs[_t].getEndBonePosition(rnd_body_id)).magnitude;
        //            if (dis < minDis)
        //            {
        //                minDis = dis;
        //                closest_hold_id = h;
        //            }
        //        }

        //        _trajectorySamples[_t].targetHoldIds[rnd_body_id] = closest_hold_id;
        //        return;
        //    }

        //    void SampleHoldPositionsInTrajectory(int _t)
        //    {
        //        //_controlRigs[_t].loadState(random_initial_state_idx);
        //        int[] cHoldIds = _controlRigs[_t].getCurrentHoldIds();

        //        // setting position of initial state holds
        //        for (int i = 0; i < 4; i++)
        //        {
        //            if (cHoldIds[i] >= 0)
        //            {
        //                Vector3 ePos = _controlRigs[_t].getEndBonePosition(i);
        //                SetPositionInContextForHold(_t, _controlRigs[_t].getCurrentHoldBodyID(i), ePos);
        //            }
        //        }

        //        ///////////////////////////// random body movement
        //        int numHandsConnected = (cHoldIds[2] > -1) ? 1 : 0;
        //        numHandsConnected += (cHoldIds[3] > -1) ? 1 : 0;
        //        int[] body_ids = { 0, 1, 2, 3 };

        //        if (numHandsConnected == 0)
        //        {
        //            body_ids = new int[] { 2, 3 };
        //        }

        //        if (numHandsConnected == 1)
        //        {
        //            // move the other hand if the connected hand is chosen to be moved
        //            if (cHoldIds[2] > -1)
        //            {
        //                body_ids = new int[] { 0, 1, 3 };
        //            }
        //            if (cHoldIds[3] > -1)
        //            {
        //                body_ids = new int[] { 0, 1, 2 };
        //            }
        //        }
        //        int rnd_body_id = body_ids[(int)Random.Range((int)0, (int)body_ids.Length)];
        //        ////////////////////////////// find free hold
        //        List<int> hold_ids = new List<int>();
        //        for (int _h = 0; _h < 8; _h++)
        //        {
        //            hold_ids.Add(_h);
        //        }
        //        for (int _b = 0; _b < 4; _b++)
        //        {
        //            if (cHoldIds[_b] >= 0)
        //                hold_ids.Remove(cHoldIds[_b]);
        //        }
        //        ////////////////////////////// choose free limb
        //        int next_free_hold_id = hold_ids[0];
        //        //if (Random.Range(0.0f, 1.0f) < 0.1f)
        //        //{
        //        //    next_free_hold_id = -1;
        //        //}

        //        // setting target hold Ids
        //        for (int _h = 0; _h < 4; _h++)
        //        {
        //            _trajectorySamples[_t].targetHoldIds[_h] = cHoldIds[_h];
        //        }
        //        _trajectorySamples[_t].targetHoldIds[rnd_body_id] = next_free_hold_id;

        //        ///////////////////////////////// random target
        //        if (next_free_hold_id >= 0)
        //        {
        //            Vector3 avg_pos = _controlRigs[_t].getBonePos(0);
        //            avg_pos[2] = 1.525f;
        //            Vector3 nPos = avg_pos + (new Vector3(Random.Range(-2.0f, 2.0f), Random.Range(-2.0f, 2.0f), 0.0f)).normalized * Random.Range(0.1f, 1.5f);

        //            nPos = _controlRigs[_t].getEndBonePosition(rnd_body_id)
        //                + (nPos - _controlRigs[_t].getEndBonePosition(rnd_body_id)).normalized * Random.Range(0.01f, _MaxSampleDis);
        //            nPos[2] = 1.525f;
        //            SetPositionInContextForHold(_t, next_free_hold_id, nPos);
        //        }

        //        return;
        //    }

        void AddToInitStatesTargets()
        {
            //if (!_ReplayExpreinces)
            //{
            //    SamplingTrajectoryStr nT = new SamplingTrajectoryStr(bestPreSample.sample_action_size, bestPreSample._UseSpline);
            //    nT._cSlotStateIdx = mMemory.GetNextFreeSlotIdx();
            //    mMemory.SaveState(nT._cSlotStateIdx, bestPreSample._cSlotStateIdx);

            //    nT._startingSlotIndex = mMemory.GetNextFreeSlotIdx();
            //    mMemory.SaveState(nT._startingSlotIndex, bestPreSample._startingSlotIndex);

            //    for (int i = 0; i < bestPreSample._sampledAction.Length; i++)
            //    {
            //        nT._sampledAction[i] = bestPreSample._sampledAction[i];
            //    }

            //    nT._sampledMaxStep = bestPreSample._sampledMaxStep;
            //    for (int i = 0; i < nT._sampledMaxStep; i++)
            //    {
            //        nT._fromToStates.Add(mMemory.GetNextFreeSlotIdx());
            //        mMemory.SaveState(nT._fromToStates[i], bestPreSample._fromToStates[i]);
            //    }

            //    for (int i = 0; i < 4; i++)
            //    {
            //        nT.targetHoldIds[i] = bestPreSample.targetHoldIds[i];
            //    }
            //    initial_trajectory_points.Add(nT);

            //    if (initial_trajectory_points.Count >= _MaxSampleSize)
            //    {
            //        _ReplayExpreinces = true;
            //        _UsePPOReward = true;
            //    }
            //}
            return;
            //if (!IsValidPosture(_memoryIdx, true, true))
            //{
            //    return -1;
            //}

            //int chosen_index = -1;
            //if (_counter_memory_init_states < initial_states.Count)
            //{
            //    chosen_index = initial_states[_counter_memory_init_states];
            //}
            //else
            //{
            //    if (initial_states.Count < _MaxSampleSize)
            //    {
            //        chosen_index = mMemory.GetNextFreeSlotIdx();
            //        initial_states.Add(chosen_index);
            //        chosen_index = initial_states[_counter_memory_init_states];
            //    }
            //    else
            //    {
            //        _counter_memory_init_states = 1; // zero is for reset index
            //    }

            //}

            //if (chosen_index >= 0)
            //{
            //    mMemory.SaveState(chosen_index, _memoryIdx);

            //    _counter_memory_init_states++;
            //}
            //return chosen_index;
        }

        //    bool IsValidPosture(int _memoryIdx, bool flag_restrict_v, bool flag_consider_hands)
        //    {
        //        if (mMemory.GetHipPos(_memoryIdx)[1] < 0.75f)
        //            return false;

        //        // if hands are disconnected
        //        if (mMemory.GetCurrentHoldId(_memoryIdx, 2) < 0 && mMemory.GetCurrentHoldId(_memoryIdx, 3) < 0 && flag_consider_hands)
        //            return false;

        //        for (int i = 0; i < 15 && flag_restrict_v; i++)
        //        {
        //            Vector3 v0 = mMemory.GetVel(_memoryIdx, i);
        //            // comes from statistics of opt data
        //            if (v0.magnitude > 10.0f)
        //                return false;
        //        }

        //        for (int i = 0; i < 5; i++)
        //        {
        //            Vector3 p0 = i > 0 ? mMemory.GetEndBonePosition(_memoryIdx, i - 1) : mMemory.GetHipPos(_memoryIdx);

        //            bool flag_in_bound = p0[1] > -0.01f && p0[1] < 8.5f && p0[2] < 1.625f && p0[0] > -2.5f && p0[0] < 2.5f;
        //            if (!flag_in_bound)
        //            {
        //                return false;
        //            }
        //        }

        //        Vector3 p0_ = mMemory.GetHipPos(_memoryIdx);

        //        for (int i = 0; i < 4; i++)
        //        {
        //            Vector3 p_i = mMemory.GetEndBonePosition(_memoryIdx, i);
        //            if (p_i[2] < 1.0f)
        //                return false;
        //            float dis = (p_i - p0_).magnitude;

        //            if (dis > 1.5f)
        //                return false;
        //        }

        //        return true;
        //    }

        //    int ReadFromFile()
        //    {
        //        int curNumTrajectories = nOptimizationSamples;
        //        bool reached_end = false;

        //        bool flag_loop = !flag_writeToFile;
        //        int _sampleSize = nControlPoints * (1 + nAngle /*time + nAngle*/) + 4 /*time to let go of the hands and feet*/;
        //        for (int i = 0; i < nNNEvalTrajectories; i++)
        //        {
        //            string cPath = "Learning\\Data_0\\Data" + current_opened_file.ToString() + ".txt";

        //            if (File.Exists(cPath))
        //            {
        //                if (mFileReader == null)
        //                    mFileReader = new StreamReader(cPath, true);
        //            }
        //            else
        //            {
        //                reached_end = true;
        //            }

        //            string l = "";
        //            if (mFileReader != null)
        //            {
        //                l = mFileReader.ReadLine();
        //                if (mFileReader.EndOfStream)
        //                {
        //                    current_opened_file++;
        //                    mFileReader.Close();
        //                    mFileReader = null;
        //                }
        //            }

        //            MyTools.ParseStringIntoFloatArr(l, ref nnEvalReply[i], ',');
        //            if (nnEvalReply[i].Count > 0)
        //            {
        //                nnEvalStartingState[i].Clear();
        //                nnEvalStartingHolds[i].Clear();
        //                nnEvalPredictedPolicy[i].Clear();
        //                _trajectorySamples[curNumTrajectories].datum_index = i;
        //                int cIndex = 0;

        //                int experience_num = (int)nnEvalReply[i][cIndex]; cIndex++;
        //                int success_count = (int)nnEvalReply[i][cIndex]; cIndex++;

        //                int cLength = (int)nnEvalReply[i][cIndex]; cIndex++;
        //                for (int s = 0; s < cLength; s++) // 334
        //                {
        //                    nnEvalStartingState[i].Add(nnEvalReply[i][cIndex]); cIndex++;
        //                }
        //                cLength = (int)nnEvalReply[i][cIndex]; cIndex++;
        //                for (int s = 0; s < cLength; s++) // 32
        //                {
        //                    nnEvalStartingHolds[i].Add(nnEvalReply[i][cIndex]); cIndex++;
        //                }
        //                cLength = _sampleSize; cIndex++;
        //                for (int s = 0; s < cLength; s++) // 66
        //                {
        //                    nnEvalPredictedPolicy[i].Add(nnEvalReply[i][cIndex]); cIndex++;
        //                }
        //                curNumTrajectories++;
        //            }
        //        }
        //        if (reached_end && flag_loop)
        //            current_opened_file = 0;
        //        return curNumTrajectories;
        //    }

        void WriteToFile(int curNumTrajectories, bool write_all_steps = false)
        {
            if (!flag_writeToFile)
                return;

            //for (int t = nOptimizationSamples; t < curNumTrajectories; t++)
            //{
            //    int[] target_holds = _trajectorySamples[t].targetHoldIds;
            //    int[] cur_holds = _controlRigs[t].GetCurrentHoldIds();
            //    bool success_transition = MyTools.IsStanceAEqualStanceB(target_holds, cur_holds);

            //    string cPath = "Learning\\Data\\Data" + current_written_file.ToString() + ".txt";
            //    if (mFileWriter == null)
            //    {
            //        mFileWriter = new StreamWriter(cPath, true);
            //    }
            //    string write_eval = "";
            //    int cIndex = 0;

            //    // write exprience and success
            //    int cur_itr_expreince = 0; cIndex++;// (int)nnEvalReply[t - nOptimizationSamples][cIndex]; cIndex++;
            //    int cur_success_itr = 0; cIndex++;// (int)nnEvalReply[t - nOptimizationSamples][cIndex]; cIndex++;
            //    if (success_transition)
            //    {
            //        cur_success_itr++;
            //    }
            //    write_eval = write_eval + (cur_itr_expreince + 1).ToString() + ",";
            //    write_eval = write_eval + cur_success_itr.ToString() + ",";

            //    int state_length = (int)nnEvalReply[t - nOptimizationSamples][cIndex]; cIndex++;
            //    write_eval = write_eval + state_length.ToString() + ",";
            //    for (int i = 0; i < state_length; i++)
            //    {
            //        write_eval = write_eval + nnEvalReply[t - nOptimizationSamples][cIndex].ToString() + ","; cIndex++;
            //    }

            //    int holds_length = (int)nnEvalReply[t - nOptimizationSamples][cIndex]; cIndex++;
            //    write_eval = write_eval + holds_length.ToString() + ",";
            //    for (int i = 0; i < holds_length; i++)
            //    {
            //        write_eval = write_eval + nnEvalReply[t - nOptimizationSamples][cIndex].ToString() + ","; cIndex++;
            //    }

            //    int _sampleSize = nControlPoints * (1 + nAngle /*time + nAngle*/) + 4 /*time to let go of the hands and feet*/;
            //    int action_length = (int)_sampleSize; cIndex++;
            //    write_eval = write_eval + (action_length).ToString() + ",";
            //    for (int i = 0; i < action_length; i++)
            //    {
            //        write_eval = write_eval + nnEvalReply[t - nOptimizationSamples][cIndex].ToString() + ","; cIndex++;
            //    }

            //    Vector2 cValue = GetDisCurEndPointToTarget(_trajectorySamples[t]._cSlotStateIdx, t);//_trajectorySamples[t].objectiveFuncVal;// + nnEvalReply[t - nOptimizationSamples][cIndex];
            //    write_eval = write_eval + (cValue[0]).ToString() + ",";

            //    float[] featureState = GetFeatureState(_trajectorySamples[t]._startingSlotIndex, target_holds, t);
            //    for (int i = 0; i < featureState.Length; i++)
            //    {
            //        write_eval = write_eval + featureState[i].ToString() + ",";
            //    }

            //    float[] nfeatureState = GetFeatureState(_trajectorySamples[t]._cSlotStateIdx, target_holds, t);
            //    for (int i = 0; i < featureState.Length; i++)
            //    {
            //        if (i == featureState.Length - 1)
            //        {
            //            write_eval = write_eval + featureState[i].ToString();
            //        }
            //        else
            //        {
            //            write_eval = write_eval + featureState[i].ToString() + ",";
            //        }
            //    }

            //    mFileWriter.WriteLine(write_eval);
            //    current_written_data_index++;
            //    if (current_written_data_index > 100)
            //    {
            //        current_written_data_index = 0;
            //        current_written_file++;

            //        mFileWriter.Flush();
            //        mFileWriter.Close();
            //        mFileWriter = null;
            //    }
            //}
        }

        //    public void FinilizeLowLevelController()
        //    {
        //        if (mFileWriter != null)
        //        {
        //            mFileWriter.Flush();
        //            mFileWriter.Close();
        //            mFileWriter = null;
        //        }
        //    }

        void InitOptimization()
        {
            // reset optimization samples and values
            int _sampleSize = nControlPoints * (1 + nAngle /*time + nAngle*/) + 4 /*time to let go of the hands and feet*/;
            for (int i = 0; i < nOptimizationSamples; i++)
            {
                _samples[i].objectiveFuncVal = 0.0f;
                for (int c = 0; c < _sampleSize; c++)
                {
                    // sample control points randomly in the first iteration
                    if (predicted_policy_mean_std.Count == _sampleSize)
                    {
                        if (i != 0)
                        {
                            _samples[i].x[c] = predicted_policy_mean_std[c] + Random.Range(minSampleVal, maxSampleVal);
                        }
                        else
                        {
                            _samples[i].x[c] = predicted_policy_mean_std[c];
                            BestSample.x[c] = predicted_policy_mean_std[c];
                        }
                    }
                    else if (predicted_policy_mean_std.Count == 2 * _sampleSize)
                    {
                        if (i != 0)
                        {
                            _samples[i].x[c] = MyTools.NextGussian(predicted_policy_mean_std[c], predicted_policy_mean_std[c + _sampleSize]);
                        }
                        else
                        {
                            _samples[i].x[c] = predicted_policy_mean_std[c];
                            BestSample.x[c] = predicted_policy_mean_std[c];
                        }
                    }
                    else
                    {
                        _samples[i].x[c] = Random.Range(minSampleVal, maxSampleVal);
                    }

                    _samples[i].x[c] = Mathf.Clamp((float)_samples[i].x[c], minSampleVal, maxSampleVal);
                    // compute initial mean
                    if (i == 0)
                    {
                        _initialMean[c] = _samples[i].x[c] / nOptimizationSamples;
                    }
                    else
                    {
                        _initialMean[c] += _samples[i].x[c] / nOptimizationSamples;
                    }
                }
            }

            //double[] _initialSD = new double[_sampleSize];
            //for (int i = 0; i < nOptimizationSamples; i++)
            //{
            //    for (int c = 0; c < _sampleSize; c++)
            //    {
            //        double sd = _samples[i].x[c] - _initialMean[c];
            //        _initialSD[c] += (sd*sd)/ nOptimizationSamples;
            //    }
            //}

            _iter = 0;
            // restarting ma-es for new optimization
            float step_size = 0.5f * (maxSampleVal - minSampleVal);
            _opt.init(_sampleSize, nOptimizationSamples, _initialMean, step_size, OptimizationModes.maximize);//_initialSD

            Physics.autoSimulation = false;
            // reset best sample
            bestPreSample.objectiveFuncVal = double.MinValue;
            bestPreSample.isReached = false;
            bestPreSample._sampledMaxStep = -1;

            neuralNetSampleVal = double.MinValue;

            CopyMasterContextToOtherContexts();
        }

        void ConvertToSampleAction(int _t, bool use_opt_samples)
        {
            if (_trajectorySamples[_t]._UseSpline)
            {
                const int nValuePerSegments = nAngle + 1;
                // min_effect_time > 0, means the climber should let go of the limb if it planned for moving
                const float min_effect_time = minSegmentTime;
                float total_time_sample = 0.0f;
                // sample nControlPoints * (1 + nAngle)
                for (int cp = 0; cp < nControlPoints; cp++)
                {
                    for (int c = 0; c < nValuePerSegments; c++)
                    {
                        int sIdx = nValuePerSegments * cp + c;

                        float _v = 0.0f;
                        if (use_opt_samples)
                            _v = (float)_samples[_t].x[sIdx];
                        else
                            _v = (float)nnEvalPredictedPolicy[_t - nOptimizationSamples][sIdx];
                        //clamp
                        _v = Mathf.Clamp(_v, minSampleVal, maxSampleVal);

                        float theta = (_v - minSampleVal) / (maxSampleVal - minSampleVal);

                        if (c == 0) // time
                        {
                            _trajectorySamples[_t]._sampledAction[sIdx] = minSegmentTime + theta * (maxSegmentTime - minSegmentTime);

                            if (cp <= nControlPoints - 2)
                            {
                                total_time_sample += _trajectorySamples[_t]._sampledAction[sIdx];
                            }
                        }
                        else
                        {
                            _v = _controlRigs[_t].minAngle[c - 1] + theta * (_controlRigs[_t].maxAngle[c - 1] - _controlRigs[_t].minAngle[c - 1]);
                            _trajectorySamples[_t]._sampledAction[sIdx] = Mathf.Clamp(_v, _controlRigs[_t].minAngle[c - 1], _controlRigs[_t].maxAngle[c - 1]);
                        }
                    }
                }

                _trajectorySamples[_t]._sampledMaxStep = Mathf.Clamp((int)(total_time_sample / Time.fixedDeltaTime)
                                                , (int)(minSegmentTime / Time.fixedDeltaTime), (int)(maxSegmentTime / Time.fixedDeltaTime));

                // sample let go time
                for (int id = 0; id < 4; id++)
                {
                    int sIdx = nValuePerSegments * nControlPoints + id;
                    float _v = 0.0f;
                    if (use_opt_samples)
                        _v = (float)_samples[_t].x[sIdx];
                    else
                        _v = (float)nnEvalPredictedPolicy[_t - nOptimizationSamples][sIdx];
                    //clamp
                    _v = Mathf.Clamp(_v, minSampleVal, maxSampleVal);

                    float theta = (_v - minSampleVal) / (maxSampleVal - minSampleVal);

                    _trajectorySamples[_t]._sampledAction[sIdx] = (total_time_sample - min_effect_time) * theta;
                }
            }
            else
            {
                _trajectorySamples[_t]._sampledMaxStep = 3;
                for (int c = 0; c < nAngle; c++)
                {
                    int sIdx = c;
                    //clamp
                    float _v = 0.0f;
                    if (use_opt_samples)
                        _v = (float)_samples[_t].x[sIdx];
                    else
                        _v = (float)nnEvalPredictedPolicy[_t - nOptimizationSamples][sIdx];

                    _v = Mathf.Clamp(_v, minSampleVal, maxSampleVal);
                    float theta = (_v - minSampleVal) / (maxSampleVal - minSampleVal);
                    _v = _controlRigs[_t].minAngle[c] + theta * (_controlRigs[_t].maxAngle[c] - _controlRigs[_t].minAngle[c]);
                    _trajectorySamples[_t]._sampledAction[sIdx] = Mathf.Clamp(_v, _controlRigs[_t].minAngle[c], _controlRigs[_t].maxAngle[c]);
                }
                //// sample let go time
                //for (int id = 0; id < 4; id++)
                //{
                //    int sIdx = nAngle + id;
                //    //clamp
                //    float tmp_v = Mathf.Clamp((float)_samplePoint[sIdx], minSampleVal, maxSampleVal);
                //    float theta = (tmp_v - minSampleVal) / (maxSampleVal - minSampleVal);
                //    _trajectorySamples[_t]._sampledAction[sIdx] = theta * _MaxContinueSample;
                //}
            }

            return;
        }

        //    bool SetHoldPositionsFromList(int contextNum, int in_list_index, Vector3[] ePoses, bool target_from_endPos = false)
        //    {
        //        bool isInitialHoldsAwayFromWall = false;

        //        int cIndex = 0;
        //        float[] cHoldIDs = new float[4];
        //        for (int i = 0; i < 4; i++)
        //        {
        //            cIndex = MyTools.LoadFromList(cIndex, ref nnEvalStartingHolds[in_list_index], ref cHoldIDs[i]);
        //        }
        //        Vector3 cHoldPos = new Vector3();
        //        for (int i = 0; i < 4; i++)
        //        {
        //            cIndex = MyTools.LoadFromList(cIndex, ref nnEvalStartingHolds[in_list_index], ref cHoldPos);
        //            int cHID = (int)cHoldIDs[i];
        //            if (cHID != -1)
        //            {
        //                SetPositionInContextForHold(contextNum, cHID, cHoldPos);

        //                if (Mathf.Abs(cHoldPos[2] - 1.525f) > 0.25f)
        //                {
        //                    isInitialHoldsAwayFromWall = true;
        //                }
        //            }
        //        }

        //        // reading target ids
        //        float[] tHoldIDs = new float[4];
        //        for (int i = 0; i < 4; i++)
        //        {
        //            cIndex = MyTools.LoadFromList(cIndex, ref nnEvalStartingHolds[in_list_index], ref tHoldIDs[i]);
        //        }

        //        for (int i = 0; i < 4; i++)
        //        {
        //            cIndex = MyTools.LoadFromList(cIndex, ref nnEvalStartingHolds[in_list_index], ref cHoldPos);
        //            int cHID = (int)tHoldIDs[i];
        //            if (cHID != -1)
        //            {
        //                Vector3 target_pos = cHoldPos;
        //                if (target_from_endPos && (int)cHoldIDs[i] != (int)tHoldIDs[i])
        //                {
        //                    target_pos = ePoses[i];
        //                }
        //                SetPositionInContextForHold(contextNum, cHID, target_pos);
        //            }
        //            if (contextNum < _trajectorySamples.Count && contextNum >= 0)
        //                _trajectorySamples[contextNum].targetHoldIds[i] = cHID;
        //        }

        //        return isInitialHoldsAwayFromWall;
        //    }

        void EndOptimization()
        {
            _iter = 0;
            _fixedIter = 0;

            if (bestPreSample.isReached)
            {
                // continue from where you left off
                startingStateIdx = bestPreSample._cSlotStateIdx;
                // only store the reference path when the transition was succeeded
                AddToInitStatesTargets();
            }
            else
            {
                // do the opt again untill success (it can be same state with diff target stance)
                startingStateIdx = bestPreSample._startingSlotIndex;
            }

            //int[] cBHoldIds = GetBestSampleCurrentHoldIds();

            //AddToRandomInitStates(bestPreSample._cSlotStateIdx);
            return;
        }

        public int GetMasterTrajectoryIdx()
        {
            return masterContextIdx;
        }

        public ContextManager GetMasterContext()
        {
            return _controlRigs[masterContextIdx].GetContextManager();
        }

        public SamplingTrajectoryStr GetBestSample()
        {
            return bestPreSample;
        }

        public bool IsBestSampleReached()
        {
            return bestPreSample.isReached;
        }

        public double[] GetBestSampledPolicy()
        {
            return BestSample.x;
        }

        public double GetBestSampleValue()
        {
            return BestSample.objectiveFuncVal;
        }

        //    public int[] GetBestSampleCurrentHoldIds()
        //    {
        //        int[] _outHoldIds = { -1, -1, -1, -1 };
        //        mMemory.GetHoldIds(bestPreSample._cSlotStateIdx, ref _outHoldIds);
        //        return _outHoldIds;
        //    }

        //    public Vector3[] GetBestSampleEndPointPoses()
        //    {
        //        List<Vector3> poses = new List<Vector3>();
        //        mMemory.GetEndPointPoses(bestPreSample._cSlotStateIdx, ref poses);
        //        return poses.ToArray();
        //    }

        //    public Vector3 GetBestSampleHipPos()
        //    {
        //        return mMemory.GetHipPos(bestPreSample._cSlotStateIdx);
        //    }

        public void SaveBestState(int _toSlot)
        {
            mMemory.SaveState(_toSlot, bestPreSample._cSlotStateIdx);
        }

        public void SaveState(int _fromContext, int _toSlot)
        {
            _controlRigs[_fromContext].SaveState(_toSlot);
        }

        //    public void LoadState(int _toContext, int _fromSlot)
        //    {
        //        _controlRigs[_toContext].loadState(_fromSlot);
        //    }

        //    public int[] GetCurrentHoldIDs(int _curContext)
        //    {
        //        return _controlRigs[_curContext].getCurrentHoldIds();
        //    }

        public int GetCurrentOptItr()
        {
            return _iter;
        }

        public void CopyMasterContextToOtherContexts()
        {
            ContextManager master_context = _controlRigs[GetMasterTrajectoryIdx()].GetContextManager();
            for (int i = 0; i < nOptimizationSamples; i++)
            {
                for (int h = 0; h < master_context.NumHolds(); h++)
                {
                    _controlRigs[i].GetContextManager().SetHoldPosition(h, master_context.GetHoldPosition(h));
                    _controlRigs[i].GetContextManager().SetHoldRotation(h, master_context.GetHoldRotation(h));
                }
                _controlRigs[i].GetContextManager().targetHoldType = master_context.targetHoldType;
                _controlRigs[i].GetContextManager().ConnectionThreshold = master_context.ConnectionThreshold;
            }
        }

        //    public Vector3 GetCOMTrajectory(int trajectoryIdx)
        //    {
        //        return _controlRigs[trajectoryIdx].COM();
        //    }

        //    int[] GetBoneIndexFrom(HumanBodyBones[] fIndex)
        //    {
        //        List<int> idx = new List<int>();
        //        for (int i = 0; i < fIndex.Length; i++)
        //        {
        //            idx.Add(_controlRigs[masterContextIdx].boneIdx(fIndex[i]));
        //        }
        //        return idx.ToArray();
        //    }

        public bool IsSampleAddable()
        {
            return (bestPreSample.objectiveFuncVal >= neuralNetSampleVal) && bestPreSample.isReached;
        }

        public void ResetOptimization()
        {
            if (_iter > 0)
            {
                EndOptimization();
            }

            for (int t = 0; t < nOptimizationSamples; t++)
            {
                _trajectorySamples[t].ResetFlag = true;
                _trajectorySamples[t]._cSimulationStep = 0;
                _trajectorySamples[t].isActionDone = false;
                _trajectorySamples[t].isReached = false;
            }
        }

        //public float[] GetStateToHolds(int stateIdx, int[] tHoldIDs, int contextIdx)
        //{
        //    List<float> outState = new List<float>();

        //    int cIndex = outState.Count + 1;

        //    MyTools.PushStateFeature(ref outState, 0);
        //    mMemory.GetState(stateIdx, ref outState);
        //    outState[cIndex - 1] = outState.Count - cIndex;

        //    cIndex = outState.Count + 1;
        //    MyTools.PushStateFeature(ref outState, 0);

        //    int[] cHoldIds = new int[4];
        //    mMemory.GetHoldIds(stateIdx, ref cHoldIds);
        //    for (int i = 0; i < 4; i++)
        //    {
        //        MyTools.PushStateFeature(ref outState, cHoldIds[i]);
        //    }
        //    for (int i = 0; i < 4; i++)
        //    {
        //        MyTools.PushStateFeature(ref outState, _controlRigs[contextIdx].GetContextManager().GetHoldPosition(cHoldIds[i]));
        //    }

        //    int count_diff = 0;
        //    for (int i = 0; i < 4; i++)
        //    {
        //        if (cHoldIds[i] != tHoldIDs[i])
        //            count_diff++;
        //        MyTools.PushStateFeature(ref outState, tHoldIDs[i]);
        //    }

        //    for (int i = 0; i < 4; i++)
        //    {
        //        MyTools.PushStateFeature(ref outState, _controlRigs[contextIdx].GetContextManager().GetHoldPosition(tHoldIDs[i]));
        //    }
        //    outState[cIndex - 1] = outState.Count - cIndex;

        //    return outState.ToArray();
        //}

        //public float[] GetFeatureState(int stateIdx, int[] tHoldIDs, int contextIdx)
        //{
        //    //bool use_endpoint_as_target = false;
        //    List<float> outState = new List<float>();
        //    outState.Add(0);
        //    Vector3[] tHoldPoses = { new Vector3(), new Vector3(), new Vector3(), new Vector3() };
        //    for (int i = 0; i < 4; i++)
        //        tHoldPoses[i] = _controlRigs[contextIdx].GetContextManager().GetHoldPosition(tHoldIDs[i]);

        //    int[] boneIndices = { 1, 2, 3, 4, 6, 7, 9, 10, 12, 13 };

        //    if (contextIdx == masterContextIdx && contextIdx > _trajectorySamples.Count) // this is used for opt
        //    {
        //        outState.Add(0);
        //        mMemory.GetFeatureState(stateIdx, ref outState, ref _trajectorySamples[0].targetHoldIds, ref tHoldPoses, boneIndices);
        //    }
        //    else
        //    {
        //        outState.Add(_trajectorySamples[contextIdx]._cSimulationStep);
        //        mMemory.GetFeatureState(stateIdx, ref outState, ref _trajectorySamples[contextIdx].targetHoldIds, ref tHoldPoses, boneIndices);
        //    }
        //    outState[0] = outState.Count - 1;
        //    return outState.ToArray();
        //}
        
    }
    
}