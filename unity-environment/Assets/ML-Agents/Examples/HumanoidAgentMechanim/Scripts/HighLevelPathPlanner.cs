using Assets.ML_Agents.Examples.SharedAssets.Scripts;

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEditor;
using ICM;
using System.IO;
using System.Text;

using System.Diagnostics;

namespace Assets.ML_Agents.Examples.ClimberScripts
{
    public class SamplingHighLevelPlan
    {
        //public List<ControlledPoses> sourcePos = new List<ControlledPoses>();
        //public List<Vector3> targetPos = new List<Vector3>();

        public int[] _sampledFromStanceID = { -1, -1, -1, -1 };
        public int[] _sampledTargetStanceID = { -1, -1, -1, -1 };
        public int _sampledInitialStateSlotIdx = -1;// s
        public int startingNodeID = -1;
    };

    public class HighLevelPlanner
    {
        public class Node
        {
            public Node(int iStateIdx, int iFatherNodeId)
            {
                stateIdx = iStateIdx;
                fatherNodeIdx = iFatherNodeId;
            }

            public int stateIdx = -1;
            public int fatherNodeIdx = -1;
            public List<int> mChildrenTreeNodes = new List<int>();
            public int graph_index_node = -1;
        };

        public class StanceNode
        {
            public int[] stanceHoldIDs = { -1, -1, -1, -1 };

            public List<int> neighbourStanceNodeIDs = new List<int>();
            public List<float> neighbourStanceNodeValues = new List<float>();
            public List<float> neighbourStanceNodeFailedTransitionVal = new List<float>();

            public int stanceIndex = -1;

            public bool dijkstraVisited = false;
            public float g_AStar = 0.0f;
            public int bFatherStanceIndex = -1;
        };

        public class StanceGraph
        {
            const float _MAXFailedTransitionCost = 1e6f;
            ContextManager mContext;
            Dictionary<uint, int> stanceToNode = new Dictionary<uint, int>();
            List<List<int>> handClusterStances = new List<List<int>>();
            List<StanceNode> mStanceNodes = new List<StanceNode>();
            List<int> cPath = new List<int>();

            public StanceGraph(ContextManager iContext)
            {
                mContext = iContext;

                for (int i = 0; i <= mContext.NumHolds(); i++)
                    handClusterStances.Add(new List<int>());

                AddStanceNode(new int[] { -1, -1, -1, -1 });

                //float[] l = { -5, 2, -5, 7, -10, 0, 1, -20 };

                //SortedList<float, int> _minmax = new SortedList<float, int>();
                //for (int i = 0; i < l.Length; i++)
                //{
                //    _minmax.Add(l[i], i);
                //    int f = _minmax.Values[0];
                //}
                //_minmax.RemoveAt(0);
                //int _f = _minmax.Values[0];
                //return;
            }

            public void ClearGraph()
            {
                stanceToNode.Clear();
                handClusterStances.Clear();
                mStanceNodes.Clear();
                cPath.Clear();

                for (int i = 0; i <= mContext.NumHolds(); i++)
                    handClusterStances.Add(new List<int>());

                AddStanceNode(new int[] { -1, -1, -1, -1 });
            }

            public int GetStanceNodeIndex(int[] stance)
            {
                uint key = StanceToKey(stance);

                if (stanceToNode.ContainsKey(key))
                    return stanceToNode[key];

                int retNodeIndex = UpdateGraph(stance);

                return retNodeIndex;
            }

            public void GetHoldIds(int _graphNodeIndex, ref int[] out_holds)
            {
                for (int i = 0; i < 4; i++)
                {
                    out_holds[i] = mStanceNodes[_graphNodeIndex].stanceHoldIDs[i];
                }
            }

            public void UpdateFailedTransitions(int[] _fStance, int[] _tStance)
            {
                int fStanceIdx = GetStanceNodeIndex(_fStance);
                int tStanceIdx = GetStanceNodeIndex(_tStance);

                for (int n = 0; n < mStanceNodes[fStanceIdx].neighbourStanceNodeIDs.Count; n++)
                {
                    if (mStanceNodes[fStanceIdx].neighbourStanceNodeIDs[n] == tStanceIdx)
                    {
                        mStanceNodes[fStanceIdx].neighbourStanceNodeFailedTransitionVal[n] += _MAXFailedTransitionCost;
                    }
                }
            }

            uint StanceToKey(int[] stance)
            {
                uint result = 0;
                for (int i = 0; i < 4; i++)
                {
                    uint uNode = (uint)(stance[i] + 1);  //+1 to handle the -1
                    result = (result + uNode);
                    if (i < 3)
                        result = result << 8; //shift, assuming max 256 holds
                }
                return result;
            }

            int GetClusterIndex(int[] stance)
            {
                int connectedHand = -1;
                if (stance[3] != -1)
                {
                    connectedHand = stance[3];
                }
                else
                {
                    connectedHand = stance[2];
                }

                if (connectedHand >= 0)
                    return connectedHand + 1;
                return 0;
            }

            public int[] ApplyDijkstra()
            {
                //Run Dijkstra backwards, initializing all goal nodes to zero cost and others to infinity.
                //This results in each node having the cost as the minimum cost to go towards any of the goals.
                //This will then be used as the heuristic in A* prune. Note that if the climber is able to make
                //all the moves corresponding to all edges, the heuristic equals the actual cost, and A* will be optimal.
                //In case the climber fails, this function will be called again, i.e., the heuristic will be updated
                //after each failure.
                //Note: minmax is a DynamicMinMax instance - almost like std::priority_queue, but supports updating 
                //the priorities of elements at a logN cost without removal & insertion.
                DynamicMinMax _minmax = new DynamicMinMax(mStanceNodes.Count);

                for (int i = 0; i < mStanceNodes.Count; i++)
                {
                    mStanceNodes[i].g_AStar = float.MaxValue;
                    //  mStanceNodes[i].h_AStar = FLT_MAX;  //this is the output value, i.e., the "heuristic" used by A* prune
                    mStanceNodes[i].dijkstraVisited = false;
                    mStanceNodes[i].bFatherStanceIndex = -1;
                    _minmax.SetValue(i, float.MaxValue);
                }

                // starting from root, it can be anywhere else!
                mStanceNodes[0].g_AStar = 0;
                _minmax.SetValue(0, mStanceNodes[0].g_AStar);

                int nVisited = 0;
                int goal_node_index = -1;
                float goal_dis = float.MaxValue;
                while (nVisited < mStanceNodes.Count)
                {
                    //get the node with least cost (initially, all goal nodes have zero cost)
                    StanceNode pNode = mStanceNodes[_minmax.GetMinIdx()];//minmax.getMinIdx()

                    //loop over neighbors (childStanceIds really contains all the neighbors)
                    //and update their costs
                    for (int c = 0; c < pNode.neighbourStanceNodeIDs.Count; c++)
                    {
                        StanceNode cNode = mStanceNodes[pNode.neighbourStanceNodeIDs[c]];

                        float nF = pNode.g_AStar + pNode.neighbourStanceNodeValues[c] + pNode.neighbourStanceNodeFailedTransitionVal[c];

                        if (!cNode.dijkstraVisited)
                        {
                            if (nF < cNode.g_AStar)
                            {
                                cNode.g_AStar = nF;
                                cNode.bFatherStanceIndex = pNode.stanceIndex;

                                _minmax.SetValue(cNode.stanceIndex, nF);

                                // update goal on an incomplete graph
                                int handHoldIdx = GetClusterIndex(cNode.stanceHoldIDs) - 1;
                                if (handHoldIdx > -1)
                                {
                                    float cDis = (mContext.GetHoldPosition(handHoldIdx) - mContext.GetGoalHoldPosition()).magnitude;
                                    if (cDis < goal_dis)
                                    {
                                        goal_node_index = cNode.stanceIndex;
                                        goal_dis = cDis;
                                    }
                                }
                            }
                        }
                    } //all neighbors
                    //Mark the node as visited. Also set it's priority to FLT_MAX so that it will not be returned by minmax.getMinIdx().
                    //Note that since dijkstraVisited is now true, the priority will not be updated again and will stay at FLT_MAX for the remainder of the algorithm.
                    pNode.dijkstraVisited = true;
                    _minmax.SetValue(pNode.stanceIndex, float.MaxValue);

                    nVisited++;
                }

                ReturnPath(goal_node_index);

                _minmax.DestroyMinMax();

                return cPath.ToArray();
            }

            void ReturnPath(int goal_node_index)
            {
                int cNode = goal_node_index;
                cPath.Clear();
                int counter = 0;
                while (cNode != -1)
                {
                    cPath.Insert(0, cNode);
                    cNode = mStanceNodes[cNode].bFatherStanceIndex;

                    counter++;
                    if (counter > mStanceNodes.Count)
                    {
                        break;
                    }
                }
                return;
            }

            public void CompleteGraph()
            {
                int rndHoldIndex = Random.Range(0, mContext.NumHolds());

                int[] sampleStance = mContext.GetRandomStanceAroundHandHold(rndHoldIndex);

                UpdateGraph(sampleStance);
            }

            public void CompleteGraphAround(int[] curHoldIds)
            {
                int rndHoldIndex = Random.Range(0, mContext.NumHolds());

                int[] sampleStance = mContext.GetRandomStanceAroundHandHold(rndHoldIndex, curHoldIds, 1);

                UpdateGraph(sampleStance);
            }

            int UpdateGraph(int[] sampleStance)
            {
                // add stances to graph and cluster
                int nStanceIdx = AddStanceNode(sampleStance);

                StanceNode nStance = mStanceNodes[nStanceIdx];
                int nClusterHandIdx = GetClusterIndex(nStance.stanceHoldIDs) - 1;
                Vector3 nP = mContext.GetHoldPosition(nClusterHandIdx);
                // update stance connections based on the clustering
                for (int c = 0; c < handClusterStances.Count; c++)
                {
                    int clusterHandHoldIdx = c - 1;
                    Vector3 p = mContext.GetMidHoldStancePos(mStanceNodes[0].stanceHoldIDs);
                    if (clusterHandHoldIdx >= 0)
                    {
                        p = mContext.GetHoldPosition(clusterHandHoldIdx);
                    }

                    if ((nP - p).magnitude < 1.0f)
                    {
                        for (int s = 0; s < handClusterStances[c].Count; s++)
                        {
                            int cStanceIdx = handClusterStances[c][s];
                            StanceNode cStance = mStanceNodes[cStanceIdx];
                            
                            if (MyTools.GetDiffBtwSetASetB(cStance.stanceHoldIDs, nStance.stanceHoldIDs) == 1)
                            {
                                UpdateStanceConnection(cStanceIdx, nStanceIdx);
                            }
                        }
                    }
                }

                return nStanceIdx;
            }

            void UpdateStanceConnection(int si, int sj)
            {
                StanceNode stance_i = mStanceNodes[si];
                StanceNode stance_j = mStanceNodes[sj];

                if (!stance_i.neighbourStanceNodeIDs.Contains(sj))
                {
                    stance_i.neighbourStanceNodeIDs.Add(sj);
                    stance_i.neighbourStanceNodeValues.Add(getCostAtNode(stance_j.stanceHoldIDs) + getCostMovementLimbs(stance_i.stanceHoldIDs, stance_j.stanceHoldIDs));
                    stance_i.neighbourStanceNodeFailedTransitionVal.Add(1.0f);
                }

                if (!stance_j.neighbourStanceNodeIDs.Contains(si))
                {
                    stance_j.neighbourStanceNodeIDs.Add(si);
                    stance_j.neighbourStanceNodeValues.Add(getCostAtNode(stance_i.stanceHoldIDs) + getCostMovementLimbs(stance_j.stanceHoldIDs, stance_i.stanceHoldIDs));
                    stance_j.neighbourStanceNodeFailedTransitionVal.Add(1.0f);
                }
            }

            int AddStanceNode(int[] stance)
            {
                uint key = StanceToKey(stance);

                if (stanceToNode.ContainsKey(key))
                    return stanceToNode[key];

                StanceNode nNode = new StanceNode();

                for (int i = 0; i < 4; i++)
                {
                    nNode.stanceHoldIDs[i] = stance[i];
                }
                nNode.stanceIndex = mStanceNodes.Count;
                mStanceNodes.Add(nNode);

                stanceToNode.Add(key, mStanceNodes.Count - 1);

                int clusterIndex = GetClusterIndex(stance);
                handClusterStances[clusterIndex].Add(mStanceNodes.Count - 1);

                return mStanceNodes.Count - 1;
            }

            // Siggraph 2017 Heuristics
            float getCostAtNode(int[] iCStance)
            {
                float k_crossing = 100;
                float k_hanging_hand = 200 + 50; // 200
                float k_hanging_leg = 10 + 10; // 10
                                               //		float k_hanging_more_than2 = 0;//100;
                float k_matching = 100;
                float k_dis = 1000;

                float _cost = 0.0f;

                // punish for hanging more than one limb
                int counter_hanging = 0;
                for (int i = 0; i < iCStance.Length; i++)
                {
                    if (iCStance[i] == -1)
                    {
                        counter_hanging++;

                        if (i >= 2)
                        {
                            // punish for having hanging hand
                            _cost += k_hanging_hand;

                        }
                        else
                        {
                            // punish for having hanging hand
                            _cost += k_hanging_leg;
                        }
                    }
                }

                // crossing hands
                if (iCStance[2] != -1 && iCStance[3] != -1)
                {
                    Vector3 rHand = mContext.GetHoldPosition(iCStance[3]);
                    Vector3 lHand = mContext.GetHoldPosition(iCStance[2]);

                    if (rHand.x < lHand.x)
                    {
                        _cost += k_crossing;
                    }
                }

                // crossing feet
                if (iCStance[0] != -1 && iCStance[1] != -1)
                {
                    Vector3 lLeg = mContext.GetHoldPosition(iCStance[0]);
                    Vector3 rLeg = mContext.GetHoldPosition(iCStance[1]);

                    if (rLeg.x < lLeg.x)
                    {
                        _cost += k_crossing;
                    }
                }

                // crossing hand and foot
                for (int i = 0; i <= 1; i++)
                {
                    if (iCStance[i] != -1)
                    {
                        Vector3 leg = mContext.GetHoldPosition(iCStance[i]);
                        for (int j = 2; j <= 3; j++)
                        {
                            if (iCStance[j] != -1)
                            {
                                Vector3 hand = mContext.GetHoldPosition(iCStance[j]);

                                if (hand.y <= leg.y)
                                {
                                    _cost += k_crossing;
                                }
                            }
                        }
                    }
                }

                //feet matching
                if (iCStance[0] == iCStance[1])
                {
                    _cost += k_matching;
                }

                //punishment for hand and leg being close
                for (int i = 0; i <= 1; i++)
                {
                    if (iCStance[i] != -1)
                    {
                        Vector3 leg = mContext.GetHoldPosition(iCStance[i]);
                        for (int j = 2; j <= 3; j++)
                        {
                            if (iCStance[j] != -1)
                            {
                                Vector3 hand = mContext.GetHoldPosition(iCStance[j]);

                                float cDis = (hand - leg).magnitude;

                                const float handAndLegDistanceThreshold = 0.5f;//mClimberSampler->climberRadius / 2.0f;
                                if (cDis < handAndLegDistanceThreshold)
                                {
                                    cDis /= handAndLegDistanceThreshold;
                                    _cost += k_dis * Mathf.Max(0.0f, 1.0f - cDis);
                                }
                            }
                        }
                    }
                }

                Vector3 midPoint1 = mContext.GetMidHoldStancePos(iCStance);
                float _dis = (midPoint1 - mContext.GetGoalHoldPosition()).magnitude;
                return _cost + 10 * _dis; // _cost
            }

            float getCostMovementLimbs(int[] si, int[] sj)
            {
                float k_dis = 1.0f;
                float k_2limbs = 120.0f;//20.0f;
                float k_pivoting_close_dis = 500.0f;

                //First get the actual distance between holds. We scale it up 
                //as other penalties are not expressed in meters
                float _cost = k_dis * getDisFromStanceToStance(si, sj);

                //penalize moving 2 limbs, except in "ladder climbing", i.e., moving opposite hand and leg
                bool flag_punish_2Limbs = true;
                //bool is2LimbsPunished = false;

                if (MyTools.GetDiffBtwSetASetB(si, sj) > 1.0f)
                {

                    if (si[0] != sj[0] && si[3] != sj[3] && firstHoldIsLower(si[0], sj[0]))
                    {
                        flag_punish_2Limbs = false;
                        if (sj[0] != -1 && sj[3] != -1 && mContext.GetHoldPosition(sj[3]).x - mContext.GetHoldPosition(sj[0]).x < 0.5f)
                            flag_punish_2Limbs = true;
                        if (sj[0] != -1 && sj[3] != -1 && mContext.GetHoldPosition(sj[3]).y - mContext.GetHoldPosition(sj[0]).y < 0.5f)
                            flag_punish_2Limbs = true;
                    }

                    if (si[1] != sj[1] && si[2] != sj[2] && firstHoldIsLower(si[1], sj[1]))
                    {
                        flag_punish_2Limbs = false;
                        if (sj[1] != -1 && sj[2] != -1 && mContext.GetHoldPosition(sj[1]).x - mContext.GetHoldPosition(sj[2]).x < 0.5f)
                            flag_punish_2Limbs = true;
                        if (sj[1] != -1 && sj[2] != -1 && mContext.GetHoldPosition(sj[2]).y - mContext.GetHoldPosition(sj[1]).y < 0.5f)
                            flag_punish_2Limbs = true;
                    }

                    if (flag_punish_2Limbs)
                        _cost += k_2limbs;
                }

                // calculating the stance during the transition
                int[] sn = { -1, -1, -1, -1 };
                int count_free_limbs = 0;
                for (int i = 0; i < si.Length; i++)
                {
                    if (si[i] != sj[i])
                    {
                        sn[i] = -1;
                        count_free_limbs++;
                    }
                    else
                    {
                        sn[i] = si[i];
                    }
                }
                // free another
                if (count_free_limbs >= 2 && MyTools.GetDiffBtwSetASetB(si, sj) == 1.0f)
                    _cost += k_2limbs;

                // punish for pivoting!!!
                float v = 0.0f;
                float max_dis = float.MinValue;
                for (int i = 0; i <= 1; i++)
                {
                    if (sn[i] != -1)
                    {
                        Vector3 leg = mContext.GetHoldPosition(sn[i]);
                        for (int j = 2; j <= 3; j++)
                        {
                            if (sn[j] != -1)
                            {
                                Vector3 hand = mContext.GetHoldPosition(sn[j]);

                                float cDis = (hand - leg).magnitude;

                                if (max_dis < cDis)
                                    max_dis = cDis;
                            }
                        }
                    }
                }
                if (max_dis >= 0 && max_dis < mContext.GetClimberRadius() / 2.0f && count_free_limbs > 1.0f)
                {
                    v += k_pivoting_close_dis;
                }
                _cost += v;

                return _cost;
            }

            float getDisFromStanceToStance(int[] si, int[] sj)
            {
                float cCount = 0.0f;
                List<Vector3> hold_points_i = new List<Vector3>();
                mContext.GetHoldStancePosFrom(si, ref hold_points_i, ref cCount);//Vector3 midPoint1 = 

                List<Vector3> hold_points_j = new List<Vector3>();
                mContext.GetHoldStancePosFrom(sj, ref hold_points_j, ref cCount);//Vector3 midPoint2 = 

                float cCost = 0.0f;
                float hangingLimbExpectedMovement = 2.0f;
                for (int i = 0; i < si.Length; i++)
                {
                    float coeff_cost = 1.0f;
                    if (si[i] != sj[i])
                    {
                        Vector3 pos_i;
                        if (si[i] != -1)
                        {
                            pos_i = hold_points_i[i];
                        }
                        else
                        {
                            //pos_i = e_hold_points_i[i];
                            cCost += 0.5f;
                            continue;
                        }
                        Vector3 pos_j;
                        if (sj[i] != -1)
                        {
                            pos_j = hold_points_j[i];
                        }
                        else
                        {
                            //pos_j = e_hold_points_j[i];
                            cCost += hangingLimbExpectedMovement;
                            continue;
                        }

                        //favor moving hands
                        if (i >= 2)
                            coeff_cost = 0.9f;

                        cCost += coeff_cost * (pos_i - hold_points_j[i]).sqrMagnitude;
                    }
                    else
                    {
                        if (sj[i] == -1)
                        {
                            cCost += hangingLimbExpectedMovement;
                        }
                    }
                }

                return Mathf.Sqrt(cCost);
            }

            bool firstHoldIsLower(int hold1, int hold2)
            {
                if (hold1 == -1 && hold2 == -1)
                    return false;
                if (hold1 != -1 && mContext.GetHoldPosition(hold1).y < mContext.GetHoldPosition(hold2).y)
                {
                    return true;
                }
                //first hold is "free" => we can't really know 
                return false;
            }

        };

        public class DataStr
        {
            public DataStr(float[] _state, float[] _input, double[] _output, double _val, float[] _nextState)
            {
                state = new float[_state.Length];
                for (int i = 0; i < _state.Length; i++)
                {
                    state[i] = _state[i];
                }
                input_featue = new float[_input.Length];
                for (int i = 0; i < _input.Length; i++)
                {
                    input_featue[i] = _input[i];
                }
                output_policy = new double[_output.Length];
                for (int i = 0; i < _output.Length; i++)
                {
                    output_policy[i] = _output[i];
                }
                val = _val;
                nextState = new float[_nextState.Length];
                for (int i = 0; i < _nextState.Length; i++)
                {
                    nextState[i] = _nextState[i];
                }
            }

            float[] state;
            float[] input_featue;
            double[] output_policy;
            double val;
            float[] nextState;

            public string ToStringFunc()
            {
                string ret_val = "1,1,";
                for (int i = 0; i < state.Length; i++)
                {
                    ret_val = ret_val + state[i].ToString() + ",";
                }

                ret_val = ret_val + (output_policy.Length).ToString() + ",";
                for (int i = 0; i < output_policy.Length; i++)
                {
                    ret_val = ret_val + output_policy[i].ToString() + ",";
                }

                ret_val = ret_val + val.ToString() + ",";

                for (int i = 0; i < input_featue.Length; i++)
                {
                    ret_val = ret_val + input_featue[i].ToString() + ",";
                }

                for (int i = 0; i < nextState.Length; i++)
                {
                    if (i == nextState.Length - 1)
                        ret_val = ret_val + nextState[i].ToString();
                    else
                        ret_val = ret_val + nextState[i].ToString() + ",";
                }
                return ret_val;
            }
        };

        private List<DataStr> dataPoints = new List<DataStr>();

        public const bool addOptSamples = false;
        public const float maxSecondPerScene = 300f;
        public const int maxSampleForDecision = 20;
        public enum SaveMode { SaveMasterContext, SaveBestState, SaveBestTrajectoryStates }
        List<Node> mNodes = new List<Node>();
        Node treeRoot = null;
        int bestNodeIndex = -1;

        LowLevelController mController;
        // the context belongs to the master context
        ContextManager mContext;
        SharedMemoryManager mMemory;
        StanceGraph mGraph;
        float sTime = float.MinValue;
        int[] nearHandHoldIndex = { -1, -1 }; // hold indices near hands to be grasped
        int startingHighlevelStates = -1;
        int masterContext = -1;

        int current_file_num = 0;

        public HighLevelPlanner(LowLevelController iController, SharedMemoryManager iMemory)
        {
            mController = iController;
            mMemory = iMemory;

            masterContext = mController.GetMasterTrajectoryIdx();
            mContext = mController.GetMasterContext();
            mGraph = new StanceGraph(mContext);

        }

        public void FinalizeHighLevelPathPlanner()
        {
            WriteDataPointsToFile();
        }

        public float[] GetStateToHolds(int fatherNodeId, SamplingHighLevelPlan _samplePlan, bool flagAddNodes)
        {
            if (flagAddNodes)
            {
                if (fatherNodeId < 0 || fatherNodeId >= mNodes.Count)
                {
                    return null;
                }

                return mController.GetStateToHolds(mNodes[fatherNodeId].stateIdx, _samplePlan._sampledTargetStanceID, masterContext);
            }
            return mController.GetStateToHolds(_samplePlan._sampledInitialStateSlotIdx, _samplePlan._sampledTargetStanceID, masterContext);
        }

        // we alwayse save from masterContext
        public int SaveNodeState(SaveMode mode, SamplingHighLevelPlan _samplePlan, bool flagAddNodes)
        {
            int lStateIdx = -1;
            int fatherNodeId = -1;
            if (flagAddNodes)
            {
                if (_samplePlan != null)
                    fatherNodeId = _samplePlan.startingNodeID;

                switch (mode)
                {
                    case SaveMode.SaveMasterContext:
                        lStateIdx = mMemory.GetNextFreeSlotIdx();
                        mController.SaveState(masterContext, lStateIdx);
                        mNodes.Add(new Node(lStateIdx, fatherNodeId));
                        break;
                    case SaveMode.SaveBestState:
                        if (mController.IsBestSampleReached())
                        {
                            lStateIdx = mMemory.GetNextFreeSlotIdx();
                            mController.SaveBestState(lStateIdx);
                            mNodes.Add(new Node(lStateIdx, fatherNodeId));
                        }
                        break;
                    case SaveMode.SaveBestTrajectoryStates:
                        break;
                }

                if (lStateIdx != -1) // node has been saved
                {
                    int[] curHolds = { -1, -1, -1, -1 };
                    mMemory.GetHoldIds(lStateIdx, ref curHolds);

                    // update children index + graph index
                    if (fatherNodeId != -1)
                    {
                        mNodes[fatherNodeId].mChildrenTreeNodes.Add(mNodes.Count - 1);
                    }
                    mNodes[mNodes.Count - 1].graph_index_node = mGraph.GetStanceNodeIndex(curHolds);

                    bestNodeIndex = mNodes.Count - 1;
                }

                if (lStateIdx != -1 && fatherNodeId == -1)
                    treeRoot = mNodes[0];
            }
            else
            {
                lStateIdx = 0;

                if (dataPoints.Count > 50)
                {
                    WriteDataPointsToFile();
                }
            }
            // Add Data for learning when planning from one state node to target stance
            if (mController.IsSampleAddable())
            {
                if (addOptSamples)
                {
                    // input feature
                    float[] s = GetStateToHolds(fatherNodeId, _samplePlan, flagAddNodes);

                    // output
                    double[] policy = mController.GetBestSampledPolicy();

                    float[] f = mController.GetFeatureState(mController.GetBestSample()._startingSlotIndex, _samplePlan._sampledTargetStanceID, masterContext);
                    float[] nf = mController.GetFeatureState(mController.GetBestSample()._cSlotStateIdx, _samplePlan._sampledTargetStanceID, masterContext);

                    if (s != null && s.Length > 0 && policy.Length > 0 && nf != null && nf.Length > 0)
                    {
                        dataPoints.Add(new DataStr(s, f, policy, mController.GetBestSampleValue(), nf));
                    }
                }
            }

            return lStateIdx;
        }

        public void CompleteGraph()
        {
            for (int _s = 0; _s < maxSampleForDecision; _s++)
            {
                mGraph.CompleteGraph();
            }
            return;
        }

        void WriteDataPointsToFile()
        {
            return;
            if (dataPoints.Count > 0)
            {
                string cPath = "Learning\\Data\\Data" + current_file_num.ToString() + ".txt";

                while (File.Exists(cPath))
                {
                    current_file_num++;
                    cPath = "Learning\\Data\\Data" + current_file_num.ToString() + ".txt";
                }

                StreamWriter mWriter = new StreamWriter(cPath, true);
                for (int i = 0; i < dataPoints.Count; i++)
                {
                    string d_i = dataPoints[i].ToStringFunc();
                    // writeLine d_i to file
                    mWriter.WriteLine(d_i);
                }

                mWriter.Flush();
                mWriter.Close();

                dataPoints.Clear();
                current_file_num++;
            }
        }

        public void RandomizeScene(float cTime)
        {
            if (cTime - sTime < maxSecondPerScene)
            {
                return;
            }

            WriteDataPointsToFile();

            //     mController.currentSD = Random.Range(0.1f, 0.5f);

            mContext.RandomizeHoldPositions(Random.Range(50.0f, 130.0f));
            for (int i = 0; i < nearHandHoldIndex.Length; i++)
                nearHandHoldIndex[i] = mContext.retNearHandIndex[i];

            for (int h = 0; h < mContext.NumHolds(); h++)
            {
                mController.SetPositionForHold(h, mContext.GetHoldPosition(h));
            }

            if (startingHighlevelStates == -1)
            {
                // Saving T-Pose
                startingHighlevelStates = SaveNodeState(SaveMode.SaveMasterContext, null, true);
            }
            else
            {
                mMemory.SetCurrentFreeIndex(startingHighlevelStates + 1);

                mNodes.Clear();
                treeRoot.mChildrenTreeNodes.Clear();
                mNodes.Add(treeRoot);
                bestNodeIndex = mNodes.Count - 1;

                mGraph.ClearGraph();

                current_index_on_graph_path = -1;
                current_index_on_tree_nodes = -1;
                stance_graph_path = new int[] { };
            }
            sTime = cTime;
        }

        public void AddToFailedTransitions(SamplingHighLevelPlan _samplePlan)
        {
            mGraph.UpdateFailedTransitions(_samplePlan._sampledFromStanceID, _samplePlan._sampledTargetStanceID);
            stance_graph_path = new int[] { };
            return;
        }

        public int SampleHierarchyTree()
        {
            int cNodeIndex = bestNodeIndex;
            List<int> path = new List<int>();
            while (cNodeIndex >= 0)
            {
                path.Add(cNodeIndex);
                cNodeIndex = mNodes[cNodeIndex].fatherNodeIdx;
            }
            int rIndex = Random.Range(0, path.Count);
            return path[rIndex];
        }

        int current_index_on_graph_path = -1;
        int current_index_on_tree_nodes = -1;
        int[] stance_graph_path = { };
        bool SetSampleUsingGraph(ref SamplingHighLevelPlan _samplePlan)
        {
            if (stance_graph_path.Length == 0)
            {
                // apply dijkstra on current graph (root to goal) and find a path
                stance_graph_path = mGraph.ApplyDijkstra();
                if (stance_graph_path.Length > 0)
                {
                    current_index_on_graph_path = 1;
                    current_index_on_tree_nodes = 0;
                }
            }
            bool flag_continue = true;
            if (current_index_on_graph_path < 0 || current_index_on_graph_path >= stance_graph_path.Length)
            {
                flag_continue = false;
            }
            while (flag_continue)
            {
                Node cNode = mNodes[current_index_on_tree_nodes];
                bool flag_found = false;
                int nTreeIndex = -1;
                for (int m = 0; m < cNode.mChildrenTreeNodes.Count && !flag_found; m++)
                {
                    Node ccNode = mNodes[cNode.mChildrenTreeNodes[m]];
                    if (ccNode.graph_index_node == stance_graph_path[current_index_on_graph_path])
                    {
                        flag_found = true;
                        nTreeIndex = cNode.mChildrenTreeNodes[m];
                    }
                }
                if (!flag_found)
                {
                    flag_continue = false;
                }
                else
                {
                    current_index_on_tree_nodes = nTreeIndex;
                    if (current_index_on_graph_path >= (int)(stance_graph_path.Length - 1))
                    {
                        flag_continue = false;
                    }
                    else
                    {
                        current_index_on_graph_path++;
                    }
                }
            }
            if (current_index_on_tree_nodes >= 0 && current_index_on_tree_nodes < mNodes.Count)
            {
                _samplePlan.startingNodeID = current_index_on_tree_nodes;
                _samplePlan._sampledInitialStateSlotIdx = mNodes[current_index_on_tree_nodes].stateIdx;

                mMemory.GetHoldIds(_samplePlan._sampledInitialStateSlotIdx, ref _samplePlan._sampledFromStanceID);
            }
            else
            {
                return false;
            }

            // reached the end path
            if (mNodes[current_index_on_tree_nodes].graph_index_node == stance_graph_path[current_index_on_graph_path])
            {
                current_index_on_graph_path = stance_graph_path.Length;
            }

            if (current_index_on_graph_path >= 0 && current_index_on_graph_path < stance_graph_path.Length)
            {
                mGraph.GetHoldIds(stance_graph_path[current_index_on_graph_path], ref _samplePlan._sampledTargetStanceID);

                return true;
            }
            else
            {
                stance_graph_path = new int[] { };
            }

            return false;
            //float r = mTools::getRandomBetween_01();
            //currentExplorationRate = max(currentExplorationRate, minExplorationRate);
            //if (r < currentExplorationRate)
            //{
            //    if (current_index_on_graph_path >= 1 && current_index_on_graph_path < (int)stance_graph_path.size())
            //    {
            //        int stance_index = stance_graph_path[current_index_on_graph_path - 1];
            //        unsigned int numChildren = mySamplingGraph->getNumOfChildrenOfStance(stance_index);
            //        std::vector<int> mRandIndex;
            //        for (unsigned int i = 0; i < numChildren; ++i)
            //        {
            //            int cChildStanceID = mySamplingGraph->getChildStanceID(stance_index, i);
            //            if (stance_graph_path[current_index_on_graph_path] != cChildStanceID || currentExplorationRate > 0.8f)
            //                mRandIndex.push_back(cChildStanceID);
            //        }
            //        if (mRandIndex.size() > 0)
            //        {
            //            std::random_shuffle(mRandIndex.begin(), mRandIndex.end());
            //            mSample.desired_hold_ids = mySamplingGraph->getStanceGraph(mRandIndex[0]);
            //            updateSampleInfo();
            //        }
            //    }
            //    else
            //    {
            //        stance_graph_path.clear();
            //    }
            //}
            //else
            //{
            //    if (current_index_on_graph_path >= 0 && current_index_on_graph_path < (int)stance_graph_path.size())
            //    {
            //        mSample.desired_hold_ids = mySamplingGraph->getStanceGraph(stance_graph_path[current_index_on_graph_path]);
            //        updateSampleInfo();
            //    }
            //    else
            //    {
            //        stance_graph_path.clear();
            //    }
            //}
        }

        public void GetRandomPlanSample(ref SamplingHighLevelPlan _samplePlan)
        {
            // empty previous sample
            //_samplePlan.sourcePos.Clear();
            //_samplePlan.targetPos.Clear();
            _samplePlan._sampledInitialStateSlotIdx = -1;
            _samplePlan.startingNodeID = -1;
            for (int _b = 0; _b < 4; _b++)
            {
                _samplePlan._sampledTargetStanceID[_b] = -1;
            }

            // sample a succeeded state including T-Pose
            if (mNodes.Count > 0)
            {
                bool isSampleSet = SetSampleUsingGraph(ref _samplePlan);

                if (!isSampleSet)
                {
                    int bestSampleNodeIdx = bestNodeIndex;
                    int[] bestSampleHolds = { -1, -1, -1, -1 };
                    PlanForOneLimb(mNodes[bestSampleNodeIdx].stateIdx, ref bestSampleHolds);

                    if (bestSampleNodeIdx != -1)
                    {
                        _samplePlan.startingNodeID = bestSampleNodeIdx;
                        _samplePlan._sampledInitialStateSlotIdx = mNodes[bestSampleNodeIdx].stateIdx;

                        for (int _b = 0; _b < 4; _b++)
                        {
                            _samplePlan._sampledTargetStanceID[_b] = bestSampleHolds[_b];
                        }
                    }
                }
            }

            //// fill high-level sample for low-level controller optimization
            //for (int _h = 0; _h < _samplePlan._sampledTargetStanceID.Length; _h++)
            //{
            //    if (_samplePlan._sampledTargetStanceID[_h] >= 0)
            //    {
            //        _samplePlan.sourcePos.Add((ControlledPoses)(ControlledPoses.LeftLeg + _h));
            //        _samplePlan.targetPos.Add(mContext.HoldPosition(_samplePlan._sampledTargetStanceID[_h]));
            //    }
            //}

        }

        void PlanForOneLimb(int initial_sample_state, ref int[] out_sample)
        {
            int[] currentHolds = { -1, -1, -1, -1 };
            mMemory.GetHoldIds(initial_sample_state, ref currentHolds);

            for (int i = 0; i < 4; i++)
                out_sample[i] = currentHolds[i];

            // choosing next hand/foot hold position assuming holds are given in the context
            if (currentHolds[2] == -1 && currentHolds[3] == -1)
            {
                int chosenHandIndex = Random.Range(2, 4); // should return 2 or 3
                if (chosenHandIndex > 3)
                    chosenHandIndex = 3;

                out_sample[chosenHandIndex] = nearHandHoldIndex[chosenHandIndex - 2];
            }
            else
            {
                //int connectedHand = -1;
                //if (currentHolds[3] != -1)
                //{
                //    connectedHand = currentHolds[3];
                //}
                //else
                //{
                //    connectedHand = currentHolds[2];
                //}
                // get a couple of stance samples and choose the best one!
                int[] _sample = mContext.GetRandomStanceAroundHandHold(Random.Range(0, mContext.NumHolds()), currentHolds, 1);


                for (int i = 0; i < 4; i++)
                {
                    out_sample[i] = _sample[i];
                }
            }

            return;
        }

        public int GetMasterContextIdx()
        {
            return masterContext;
        }

        public int GetStartingStateIdx() // this index is T-Pose saved in memory
        {
            return startingHighlevelStates;
        }
    };
}