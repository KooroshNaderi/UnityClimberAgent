using Assets.ML_Agents.Examples.SharedAssets.Scripts;
using Assets.ML_Agents.Examples.ClimberScripts;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEditor;
using ICM;
using System.IO;
using System.Text;
using System.Diagnostics;
using System;

// this code contains the high-level path planner
// In reinforcement learning manner, the initial state and targeted hands/feet can be altering for each sample
// In optimization mode (CMA-ES) the sample is the same until when opt time ended or the solution converges

public class ClimberInterface
{
    public enum UserCommandType { UserForwardSimulate = 0, UserNone = 1 };
    
    public class UserTrajectoryPoints
    {
        public int cStateID = -1;

        public List<int> _fromToStates = new List<int>();

        public int[] target_ids = { -1, -1, -1, -1 };
    }
    
    Vector3 lookAtPos = new Vector3();
    Vector3 camera_pos = new Vector3(10f, -Mathf.PI / 2.0f, 0.0f);
    Vector3 lastMouseClicked = new Vector3();
    int current_select_state = 0;
    int selected_limb = -1;
    int selected_hold = -1;
    public UserCommandType current_command_type = UserCommandType.UserNone;

    List<UserTrajectoryPoints> user_trajectory_points = new List<UserTrajectoryPoints>();
    Vector2Int cStepAnimation = new Vector2Int(0, 0);

    ClimberMechanimRig controlRig;
    SharedMemoryManager memoryManager;
    ContextManager masterContext;

    bool isTaskCompleted = true;

    int[] init_hold_ids = { -1, -1, -1, -1 };
    int[] target_hold_ids = { -1, -1, -1, -1 };
    int total_states_saved = 0;

    // variables for playing animations
    float current_scroll_value = 0.0f;
    bool flag_play_loop = true;
    float time_diff_happens = 0.0f;

    public ClimberInterface(GameObject iGameInstant, SharedMemoryManager iMemoryManager)
    {
        controlRig = iGameInstant.GetComponentsInChildren<ClimberMechanimRig>()[0];

        masterContext = iGameInstant.GetComponent<ContextManager>();

        memoryManager = iMemoryManager;

        AddTrajectoryPoint(null);
    }
    
    public bool LoadCurrentState()
    {
        return controlRig.LoadState(user_trajectory_points[user_trajectory_points.Count - 1].cStateID);
    }

    public void UpdateTrajectoryPoints(bool isOptimizationDone, LowLevelController.SamplingTrajectoryStr bestSampleTrajectory)
    {
        if (isOptimizationDone)
        {
            AddTrajectoryPoint(bestSampleTrajectory);
            isTaskCompleted = true;
            current_command_type = UserCommandType.UserNone;
        }

        return;
    }

    void CheckForUserInterrupt(LowLevelController.SamplingTrajectoryStr bestSampleTrajectory)
    {
        int limb_id = selected_limb;
        int hold_id = selected_hold;
        if (limb_id < 0)
            return;

        if (user_trajectory_points[user_trajectory_points.Count - 1].target_ids[limb_id] == hold_id)
            return;

        if (current_command_type == UserCommandType.UserForwardSimulate)
        {
            UpdateTrajectoryPoints(true, bestSampleTrajectory);
            current_command_type = UserCommandType.UserNone;
        }
        return;
    }

    public void UseInterface(ref SamplingHighLevelPlan _samplePlan,
        LowLevelController.SamplingTrajectoryStr bestSampleTrajectory,
        bool play_animation)
    {
        Vector3 nCameraPos = lookAtPos + camera_pos[0] * (new Vector3(Mathf.Cos(camera_pos[1]), Mathf.Sin(camera_pos[2]), Mathf.Sin(camera_pos[1]))).normalized;
        nCameraPos.y = Mathf.Max(0f, nCameraPos.y);

        Camera.main.transform.position = nCameraPos;
        Camera.main.transform.LookAt(lookAtPos, Vector3.up);

        Transform hips = controlRig.GetBoneTransform(HumanBodyBones.Hips);
        lookAtPos = 0.95f * lookAtPos + 0.05f * hips.position;

        if (Input.GetMouseButton(1))
        {
            Vector3 delta = Input.mousePosition - lastMouseClicked;

            camera_pos[1] += delta[0] / 100.0f;
            camera_pos[2] += delta[1] / 100.0f;

            if (camera_pos[2] < -Mathf.PI / 2.0f)
                camera_pos[2] = -Mathf.PI / 2.0f;
            if (camera_pos[2] > Mathf.PI / 2.0f)
                camera_pos[2] = Mathf.PI / 2.0f;

            if (camera_pos[1] < -Mathf.PI)
                camera_pos[1] = -Mathf.PI;
            if (camera_pos[1] > 0)
                camera_pos[1] = 0;
        }
        lastMouseClicked = Input.mousePosition;

        float mouse_scroll_value = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(mouse_scroll_value) > 0)
        {
            camera_pos[0] += -3.0f * mouse_scroll_value;
        }

        if (play_animation)
        {
            return;
        }

        RaycastHit hitInfo = new RaycastHit();
        bool hit = Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out hitInfo);
        if (hit)
        {
            if (hitInfo.transform.parent != null)
            {
                if (hitInfo.transform.parent.name != "Environment")
                {
                    if (hitInfo.transform.parent.parent != null && hitInfo.transform.parent.parent.name != "Environment")
                        return;
                }
            }

            if (current_select_state == 0)
            {
                if (hitInfo.transform.gameObject.name == "RHConnector" || hitInfo.transform.gameObject.name == "hand_R")
                {
                    selected_limb = 3;
                }
                else if (hitInfo.transform.gameObject.name == "LHConnector" || hitInfo.transform.gameObject.name == "hand_L")
                {
                    selected_limb = 2;
                }
                else if (hitInfo.transform.gameObject.name == "RLConnector" || hitInfo.transform.gameObject.name == "footR")
                {
                    selected_limb = 1;
                }
                else if (hitInfo.transform.gameObject.name == "LLConnector" || hitInfo.transform.gameObject.name == "footL")
                {
                    selected_limb = 0;
                }
                else
                {
                    selected_limb = -1;
                }
            }
            else
            {
                //UnityEngine.Debug.Log(hitInfo.transform.gameObject.tag);
                if (hitInfo.transform.gameObject.tag == "Hold")
                {
                    HoldInfo cHoldInfo = hitInfo.transform.gameObject.GetComponent<HoldInfo>();
                    selected_hold = cHoldInfo.holdId;
                }
                else
                {
                    selected_hold = -1;
                }
            }
        }

        if (Input.GetMouseButton(0))
        {
            if (selected_limb > -1)
            {
                current_select_state = 1;
            }
        }

        if (Input.GetMouseButtonUp(0))
        {
            CheckForUserInterrupt(bestSampleTrajectory);
            current_select_state = 0;
            if (selected_limb > -1)
            {
                user_trajectory_points[user_trajectory_points.Count - 1].target_ids[selected_limb] = selected_hold;
                selected_hold = -1;
            }
        }

        for (int h = 0; h < masterContext.GetNumHolds(); h++)
        {
            masterContext.SetColorToHolds(h, Color.yellow);
        }

        for (int b = 0; b < 4; b++)
        {
            if (user_trajectory_points[user_trajectory_points.Count - 1].target_ids[b] == controlRig.GetCurrentHoldBodyID(b))
            {
                controlRig.SetColorToLimb(b, Color.black);
            }
            else
            {
                controlRig.SetColorToLimb(b, Color.green);
                if (user_trajectory_points[user_trajectory_points.Count - 1].target_ids[b] > -1)
                {
                    masterContext.SetColorToHolds(user_trajectory_points[user_trajectory_points.Count - 1].target_ids[b], Color.green);
                    UnityEngine.Debug.DrawLine(controlRig.GetEndBoneGlobalPosition(b),
                        masterContext.GetHoldGlobalPosition(user_trajectory_points[user_trajectory_points.Count - 1].target_ids[b]), Color.black);
                }
            }
        }

        if (selected_limb > -1)
        {
            controlRig.SetColorToLimb(selected_limb, Color.red);
        }

        if (selected_hold > -1)
        {
            masterContext.SetColorToHolds(selected_hold, Color.red);
        }

        if (Input.GetKeyDown(KeyCode.Return))
        {
            current_command_type = UserCommandType.UserForwardSimulate;

            isTaskCompleted = false;
            for (int b = 0; b < 4; b++)
            {
                init_hold_ids[b] = controlRig.GetCurrentHoldBodyID(b);
                target_hold_ids[b] = user_trajectory_points[user_trajectory_points.Count - 1].target_ids[b];
            }

            UpdateSampleHighLevelPlan(ref _samplePlan);
        }

        if (Input.GetKeyDown(KeyCode.Backspace))
        {
            RemoveLastTrajectoryPoint();
        }

        return;
    }

    public void SaveToFile()
    {
        StreamWriter streamWriter = new StreamWriter("user_scene_trajectory.txt", false);

        // save scene
        int numHolds = masterContext.GetNumHolds();
        streamWriter.WriteLine(numHolds.ToString());
        for (int h = 0; h < numHolds; h++)
        {
            streamWriter.WriteLine(masterContext.SaveHoldInfoToString(h));
        }

        int num_trajectory_points = user_trajectory_points.Count;
        streamWriter.WriteLine(num_trajectory_points.ToString());

        // now write num_trajectory_points trajectory points
        for (int i = 0; i < num_trajectory_points; i++)
        {
            // start with head line (num from-to states, target hold ids)
            string head_line = user_trajectory_points[i]._fromToStates.Count.ToString() + ","
                             + user_trajectory_points[i].target_ids[0].ToString() + ","
                             + user_trajectory_points[i].target_ids[1].ToString() + ","
                             + user_trajectory_points[i].target_ids[2].ToString() + ","
                             + user_trajectory_points[i].target_ids[3].ToString();
            streamWriter.WriteLine(head_line);
            
            // save current state
            streamWriter.WriteLine(
                MyTools.ParseArrFloatInToString(
                    memoryManager.SaveStateToFloatArr(user_trajectory_points[i].cStateID),
                    ','));
            for (int s = 0; s < user_trajectory_points[i]._fromToStates.Count; s++)
            {
                streamWriter.WriteLine(
                MyTools.ParseArrFloatInToString(
                    memoryManager.SaveStateToFloatArr(user_trajectory_points[i]._fromToStates[s]),
                    ','));
            }
        }

        streamWriter.Flush();
        streamWriter.Close();
        return;
    }

    public void LoadFromFile()
    {
        StreamReader streamReader = new StreamReader("user_scene_trajectory.txt");

        // save scene
        int numHolds = masterContext.GetNumHolds();
        string line = streamReader.ReadLine();
        for (int h = 0; h < numHolds; h++)
        {
            masterContext.LoadHoldInfoToString(h, streamReader.ReadLine());
        }

        int num_trajectory_points = (int)(float.Parse(streamReader.ReadLine()));

        List<float> outArr = new List<float>();
        int c_index = 0;
        // now load num_trajectory_points trajectory points
        for (int i = 0; i < num_trajectory_points; i++)
        {
            if (i >= user_trajectory_points.Count)
            {
                user_trajectory_points.Add(new UserTrajectoryPoints());
            }
            // start with head line (num from-to states, target hold ids)
            string head_line = streamReader.ReadLine();

            MyTools.ParseStringIntoFloatArr(head_line, ref outArr, ','); c_index = 0;

            int from_to_state_count = (int)(outArr[c_index++]);

            user_trajectory_points[i].target_ids[0] = (int)(outArr[c_index++]);
            user_trajectory_points[i].target_ids[1] = (int)(outArr[c_index++]);
            user_trajectory_points[i].target_ids[2] = (int)(outArr[c_index++]);
            user_trajectory_points[i].target_ids[3] = (int)(outArr[c_index++]);
            
            // load current state
            if (user_trajectory_points[i].cStateID < 0)
            {
                user_trajectory_points[i].cStateID = memoryManager.GetNextFreeSlotIdx();
            }
            MyTools.ParseStringIntoFloatArr(streamReader.ReadLine(), ref outArr, ','); c_index = 0;
            
            memoryManager.LoadStateFromList(user_trajectory_points[i].cStateID, ref outArr);

            for (int s = 0; s < from_to_state_count; s++)
            {
                if (s >= user_trajectory_points[i]._fromToStates.Count)
                {
                    user_trajectory_points[i]._fromToStates.Add(memoryManager.GetNextFreeSlotIdx());
                }
                if (user_trajectory_points[i]._fromToStates[s] < 0)
                {
                    user_trajectory_points[i]._fromToStates[s] = memoryManager.GetNextFreeSlotIdx();
                }

                MyTools.ParseStringIntoFloatArr(streamReader.ReadLine(), ref outArr, ','); c_index = 0;

                memoryManager.LoadStateFromList(user_trajectory_points[i]._fromToStates[s], ref outArr);
            }
        }
        return;
    }

    void UpdateSampleHighLevelPlan(ref SamplingHighLevelPlan _samplePlan)
    {
        _samplePlan.startingNodeID = -1;
        for (int h = 0; h < 4; h++)
        {
            _samplePlan._sampledFromStanceID[h] = init_hold_ids[h];
            _samplePlan._sampledTargetStanceID[h] = target_hold_ids[h];
        }
        _samplePlan._sampledInitialStateSlotIdx = user_trajectory_points[user_trajectory_points.Count - 1].cStateID;
    }
    
    void AddTrajectoryPoint(LowLevelController.SamplingTrajectoryStr bestSampleTrajectory)
    {
        user_trajectory_points.Add(new UserTrajectoryPoints());
        user_trajectory_points[user_trajectory_points.Count - 1].cStateID = memoryManager.GetNextFreeSlotIdx();

        controlRig.SaveState(user_trajectory_points[user_trajectory_points.Count - 1].cStateID);

        for (int i = 0; i < 4; i++)
        {
            user_trajectory_points[user_trajectory_points.Count - 1].target_ids[i] = target_hold_ids[i];
        }

        if (bestSampleTrajectory != null)
        {
            for (int i = 0; i < bestSampleTrajectory._sampledMaxStep; i++)
            {
                user_trajectory_points[user_trajectory_points.Count - 1]._fromToStates.Add(memoryManager.GetNextFreeSlotIdx());

                memoryManager.SaveState(user_trajectory_points[user_trajectory_points.Count - 1]._fromToStates[i], bestSampleTrajectory._fromToStates[i]);
            }
            total_states_saved += user_trajectory_points[user_trajectory_points.Count - 1]._fromToStates.Count;
        }
        return;
    }

    void RemoveLastTrajectoryPoint()
    {
        current_command_type = UserCommandType.UserNone;
        if (isTaskCompleted)
        {
            if (user_trajectory_points.Count > 1)
            {
                total_states_saved -= user_trajectory_points[user_trajectory_points.Count - 1]._fromToStates.Count;

                user_trajectory_points.RemoveAt(user_trajectory_points.Count - 1);
            }
            else
            {
                for (int b = 0; b < 4; b++)
                    user_trajectory_points[0].target_ids[b] = -1;
            }
        }

        // restoring climber's state and target stance
        controlRig.LoadState(user_trajectory_points[user_trajectory_points.Count - 1].cStateID);
        
        for (int i = 0; i < 4; i++)
        {
            // restore to current rig's attached hold ids
            init_hold_ids[i] = controlRig.GetCurrentHoldBodyID(i);
            // restore to the last saved trajectory point
            target_hold_ids[i] = user_trajectory_points[user_trajectory_points.Count - 1].target_ids[i];
        }

        isTaskCompleted = true;
        return;
    }
    
    public float PlayAnimation(float cTime, float scroll_value, bool play_in_loop)
    {
        int index_trajectory_points = cStepAnimation[0];
        int index_state = cStepAnimation[1];
        int scroll_index_target = Mathf.CeilToInt(total_states_saved * scroll_value);
        int scroll_index_current = Mathf.CeilToInt(total_states_saved * current_scroll_value); 
        if (Mathf.Abs(scroll_index_target - scroll_index_current) > 0)
        {
            int index_counter = 0;
            for (int p = 0; p < user_trajectory_points.Count; p++)
            {
                if (index_counter + user_trajectory_points[p]._fromToStates.Count < scroll_index_target)
                {
                    index_counter += user_trajectory_points[p]._fromToStates.Count;
                }
                else
                {
                    index_trajectory_points = p;
                    index_state = scroll_index_target - index_counter;
                    break;
                }
            }
            current_scroll_value = scroll_value;
            flag_play_loop = false;
            time_diff_happens = cTime;
        }

        if (cTime - time_diff_happens > 2.0f)
            flag_play_loop = true;

        if (flag_play_loop)
        {
            flag_play_loop = play_in_loop;
        }

        if (index_trajectory_points < user_trajectory_points.Count)
        {
            if (index_state < user_trajectory_points[index_trajectory_points]._fromToStates.Count)
            {
                controlRig.LoadState(user_trajectory_points[index_trajectory_points]._fromToStates[index_state]);
            }
            else
            {
                if (flag_play_loop)
                {
                    index_state = 0;
                    index_trajectory_points++;

                    if (index_trajectory_points >= user_trajectory_points.Count)
                    {
                        index_trajectory_points = 0;
                        scroll_index_target = 0;
                    }
                }
            }
            if (flag_play_loop)
            {
                index_state++;
                scroll_index_target++;
            }
        }

        cStepAnimation[0] = index_trajectory_points;
        cStepAnimation[1] = index_state;
        current_scroll_value = (scroll_index_target / (total_states_saved + 1e-6f));
        return current_scroll_value;
    }
}

public class TrajectoryManager : MonoBehaviour
{
    public const bool useLowLevelRandomSamples = false;
    public const bool tryNetworkConnection = false;
    private const bool _useAutoPhysicsUpdate = false;

    [Header("Interface")]
    public bool useInterface = true;
    public bool playAnimation = false;
    //public InputField FileName;
    public Scrollbar AnimationSlider;
    public Button SaveButton;
    public Button LoadButton;
    public Button PlayButton;
    public Button PauseButton;

    [Header("Context")]
    public Text textbox;
    public HoldInfo.HoldType targetHoldType = HoldInfo.HoldType.Sphere;
    public bool ToggleRandomizeScene = false;
    public GameObject _instantObj;

    [Header("Low Level Controller")]
    //public float MaxSampleDis = 0.25f;
    public float ConnectionThreshold = 0.25f;
    //public bool UsePPOReward = true;
    //public bool ResetInitState = false;
    //public float PercentFollowRef = 0.8f;
    //public int MaxSampleSize = 100;
    //public int nTrajectories;// number of trajectories to simulate forward each time
    
    [HideInInspector]
    ContextManager mContext;
    SharedMemoryManager mMemory;
    ClimberInterface mClimberInterface = null;
    LowLevelController mController;
    
    bool isTrajectoryManagerInitialized = false;

    HighLevelPlanner mHighLevelPlanner = null;
    SamplingHighLevelPlan _samplePlan = new SamplingHighLevelPlan();
    NetworkManager mNetworkManager = new NetworkManager();

    Text playButtonText;
    Text pauseButtonText;
    bool play_in_loop = true;
    // Use this for initialization
    void Start()
    {
        IntializeTrajectoryManager();
    }
    
    void Update()
    {
        float v = AnimationSlider.value;

        if (mNetworkManager != null && tryNetworkConnection)
        {
            mNetworkManager.ConnectToLocalHost();
        }

        mContext.ConnectionThreshold = this.ConnectionThreshold;

        if (mContext.targetHoldType != targetHoldType || ToggleRandomizeScene)
        {
            if (ToggleRandomizeScene)
            {
                RandomizeSence();
                ToggleRandomizeScene = false;
            }
            mContext.targetHoldType = targetHoldType;
            // update other contexts
            mController.CopyMasterContextToOtherContexts();
        }

        // setting controller properties
        //mController.MaxSampleDis = MaxSampleDis;
        //mController.MaxSampleSize = MaxSampleSize;
        //mController.PercentFollowRef = PercentFollowRef;
        //mController.UsePPOReward = UsePPOReward;
        //mController.PlayAnimation = playAnimation;

        if (mClimberInterface != null)
        {
            mClimberInterface.UseInterface(ref _samplePlan, mController.GetBestSample(), playAnimation);
        }
        if (!playAnimation)
        {
            if (mClimberInterface != null)
            {
                if (mClimberInterface.current_command_type == ClimberInterface.UserCommandType.UserNone)
                {
                    mController.ResetOptimization();
                    mClimberInterface.LoadCurrentState();
                }
                else if (mClimberInterface.current_command_type == ClimberInterface.UserCommandType.UserForwardSimulate)
                {
                    bool isDone = mController.OptimizeCost(ref _samplePlan);
                    mClimberInterface.UpdateTrajectoryPoints(isDone, mController.GetBestSample());
                }
            }
        }
        else
        {
            v = mClimberInterface.PlayAnimation(Time.time, v, play_in_loop);
            AnimationSlider.value = v;
        }
        // this should be moved to the low-level controller
        //if (UseAutoPhysicsUpdate)
        //{
        //    Physics.autoSimulation = true;
        //    return;
        //}

        //      if (!useLowLevelRandomSamples)
        //      {
        //          if (mController.GetCurrentOptItr() == 0)
        //          {
        //              mHighLevelPlanner.RandomizeScene(Time.time);
        //          }

        //          mHighLevelPlanner.CompleteGraph();

        //          if (mController.GetCurrentOptItr() == 0)
        //          {
        //              mHighLevelPlanner.GetRandomPlanSample(ref _samplePlan);
        //              playAnimation = false;
        //          }
        //      }

        ////      if (ResetInitState)
        ////      {
        ////          ResetInitState = false;
        ////          mController.ResetInitialStateSample();
        ////      }

        //if (!playAnimation)
        //{
        //    if (isDone)
        //    {
        //        int saved_id = mHighLevelPlanner.SaveNodeState(HighLevelPlanner.SaveMode.SaveBestState, _samplePlan, !useLowLevelRandomSamples);
        //        if (!useLowLevelRandomSamples)
        //        {
        //            if (saved_id == -1) // failed
        //            {
        //                mHighLevelPlanner.AddToFailedTransitions(_samplePlan);
        //            }
        //        }
        //    }
        //}
        //else
        //{
        //    LowLevelController.SamplingTrajectoryStr bestSample = mController.GetBestSample();

        //    if (cStepAnimation >= bestSample._sampledMaxStep || cStepAnimation < 0)
        //        cStepAnimation = 0;
        //    mController.LoadState(mController.GetMasterTrajectoryIdx(), bestSample._fromToStates[cStepAnimation]);
        //    cStepAnimation += 3;
        //}

        //drawStar(_controlRigs[GetMasterTrajectoryIdx()].getEndBonePosition(0) + _controlRigs[GetMasterTrajectoryIdx()].initialBiasPosition);
        //drawStar(_controlRigs[GetMasterTrajectoryIdx()].getEndBonePosition(1) + _controlRigs[GetMasterTrajectoryIdx()].initialBiasPosition);
        //drawStar(_controlRigs[GetMasterTrajectoryIdx()].getEndBonePosition(2) + _controlRigs[GetMasterTrajectoryIdx()].initialBiasPosition);
        //drawStar(_controlRigs[GetMasterTrajectoryIdx()].getEndBonePosition(3) + _controlRigs[GetMasterTrajectoryIdx()].initialBiasPosition);

        textbox.text = "Time: " + Time.time.ToString() + "\n"
            + "State Cost: " + (-mController.GetBestSampleValue()).ToString("f3") + "\n"
            + "Slider Value: " + v.ToString("f3");
    }
    
    void drawStar(Vector3 p)
    {
        float s = 0.5f;
        UnityEngine.Debug.DrawLine(p - new Vector3(s, 0, 0), p + new Vector3(s, 0, 0), Color.red);
        UnityEngine.Debug.DrawLine(p - new Vector3(0, s, 0), p + new Vector3(0, s, 0), Color.red);
        UnityEngine.Debug.DrawLine(p - new Vector3(0, 0, s), p + new Vector3(0, 0, s), Color.red);
        return;
    }

    void RandomizeSence()
    {
        if (useInterface)
            mContext.RandomizeHoldPositions();
        else
            mHighLevelPlanner.RandomizeScene(Time.time);

        return;
    }

    public void IntializeTrajectoryManager()
    {
        if (isTrajectoryManagerInitialized)
            return;

        Time.fixedDeltaTime = 1 / 100.0f;

        mMemory = new SharedMemoryManager();
        mContext = _instantObj.GetComponent<ContextManager>();
        mContext.DeactiveRandomizeFromInterface();
        mContext.RandomizeHoldPositions();

        mController = new LowLevelController(_instantObj, mMemory, mNetworkManager); //mContext

        if (useInterface)
            mClimberInterface = new ClimberInterface(_instantObj, mMemory);
        else
            mHighLevelPlanner = new HighLevelPlanner(mController, mMemory);

        //    _samplePlan._sampledInitialStateSlotIdx = -1;

        //    //if (useLowLevelRandomSamples && _samplePlan._sampledInitialStateSlotIdx < 0)
        //    //{
        //    //    _samplePlan._sampledInitialStateSlotIdx = mMemory.GetNextFreeSlotIdx();
        //    //}

        //if (flag_test)
        //    TestRun();

        //mHighLevelPlanner.RandomizeScene(Time.time);

        PlayButton.onClick.AddListener(PlayButtonOnClick);
        PauseButton.onClick.AddListener(PauseButtonOnClick);
        SaveButton.onClick.AddListener(SaveButtonOnClick);
        LoadButton.onClick.AddListener(LoadButtonOnClick);

        playButtonText = PlayButton.GetComponentInChildren<Text>();
        pauseButtonText = PauseButton.GetComponentInChildren<Text>();
        isTrajectoryManagerInitialized = true;
    }

    private void PlayButtonOnClick()
    {
        if (playButtonText.text == "Play")
        {
            playButtonText.text = "Stop";
            playAnimation = true;
        }
        else if (playButtonText.text == "Stop")
        {
            playButtonText.text = "Play";
            playAnimation = false;
        }
    }

    private void PauseButtonOnClick()
    {
        if (pauseButtonText.text == "Pause")
        {
            pauseButtonText.text = "UnPause";
            play_in_loop = false;
        }
        else if (pauseButtonText.text == "UnPause")
        {
            pauseButtonText.text = "Pause";
            play_in_loop = true;
        }
    }

    void SaveButtonOnClick()
    {
        if (mClimberInterface != null)
        {
            mClimberInterface.SaveToFile();
        }
        return;
    }

    void LoadButtonOnClick()
    {
        if (mClimberInterface != null)
        {
            mClimberInterface.LoadFromFile();
        }
        return;
    }

    public bool UseAutoPhysicsUpdate
    {
        get { return _useAutoPhysicsUpdate; }
    }

    //public int GetSampledInitialStateIdx(int _trajectoryIdx)
    //{
    //    return _samplePlan._sampledInitialStateSlotIdx;
    //}

    //public int[] GetSampledTargetStanceID(int _trajectoryIdx)
    //{
    //    return _samplePlan._sampledTargetStanceID;
    //}

    //void OnApplicationQuit()
    //{
    //    mNetworkManager.Close();

    //    mController.FinilizeLowLevelController();

    //    mHighLevelPlanner.FinalizeHighLevelPathPlanner();
    //}

    /// <summary>
    /// testing MAES
    /// </summary>
    double frosen(double[] x, int N)
    {
        double Fit = 0;
        double tmp1, tmp2;
        double Fit1 = 0;
        double Fit2 = 0;
        //for (int i=0; i<N-1; i++)
        //	Fit += 100 * pow( x[i]*x[i] - x[i+1], 2.0  ) + pow(x[i] - 1.0, 2.0); // function 'pow' is very slow
        for (int i = 0; i < N - 1; i++)
        {
            tmp1 = x[i] * x[i] - x[i + 1];
            tmp2 = x[i] - 1.0f;
            Fit1 += tmp1 * tmp1;
            Fit2 += tmp2 * tmp2;
        }
        Fit = 100 * Fit1 + Fit2;
        return Fit;
    }

    void TestRun()
    {
        int dim = 100, gen = 1000;
        MAES lmm = new MAES();
        int popSize = 64;
        lmm.init(dim, popSize, new double[dim], 0.25f, OptimizationModes.minimize);

        //VectorXf & fitnessArr = lmm.GetFitnessArray();
        double bestFitnessLMM = double.MaxValue;//FLT_MAX;
        for (int g = 0; g < gen; ++g)
        {
            OptimizationSample[] samples = new OptimizationSample[popSize];
            for (int i = 0; i < popSize; i++)
                samples[i] = new OptimizationSample(dim);
            lmm.generateSamples(samples);
            for (int s = 0; s < popSize; ++s)
            {
                samples[s].objectiveFuncVal = frosen(samples[s].x, dim);
                if (samples[s].objectiveFuncVal < bestFitnessLMM)
                    bestFitnessLMM = samples[s].objectiveFuncVal;
            }
            lmm.update(samples);
            UnityEngine.Debug.Log("Generation " + (g + 1).ToString() + " => " + bestFitnessLMM.ToString());
        }
    }
}
