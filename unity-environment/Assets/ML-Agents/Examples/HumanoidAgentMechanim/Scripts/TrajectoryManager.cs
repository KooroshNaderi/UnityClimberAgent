using Assets.ML_Agents.Examples.SharedAssets.Scripts;
using Assets.ML_Agents.Examples.ClimberScripts;
using UnityEngine;
using UnityEngine.UI;
using ICM;
using System.Text;
using System.Diagnostics;
using System;

// this code contains the high-level path planner
// In reinforcement learning manner, the initial state and targeted hands/feet can be altering for each sample
// In optimization mode (CMA-ES) the sample is the same until when opt time ended or the solution converges

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
