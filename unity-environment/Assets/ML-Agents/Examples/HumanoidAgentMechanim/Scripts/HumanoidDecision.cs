using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// this class decides on low-level controller actions given (s,sigma_i+1)
public class HumanoidDecision : MonoBehaviour, Decision {
    [HideInInspector]
    TrajectoryManager mTrajectoryManager;
    const int sample_size = 31;

    // Use this for initialization
    void Start ()
    {
        Random.InitState((int)System.DateTime.Now.Ticks);
//        List<double> initialMean = new List<double>(sample_size);
//        for (int i = 0; i < sample_size; i++) initialMean[i] = 0.0f;
//        opt.init(sample_size, 1, initialMean.ToArray(), 1, OptimizationModes.maximize);

        _decisionAction = new List<float>();

        mTrajectoryManager = transform.parent.GetComponent<TrajectoryManager>();
    }
	
	// Update is called once per frame
	void Update () {
		
	}

    List<float> _decisionAction;
//    OptimizationSample[] sample;
    public float[] Decide(List<float> vectorObs, List<Texture2D> visualObs, float reward, bool done, List<float> memory)
    {
        // initialize vector action
        if (_decisionAction == null)
        {
            _decisionAction = new List<float>();
        }
        
 //       opt.generateSamples(sample);

 //       mTrajectoryManager.SampleHighLevelPlan(BrainType.Heuristic);

        while (_decisionAction.Count < sample_size)
        {
            _decisionAction.Add(0f);
        }

        _decisionAction[0] = Random.Range(0.1f, 0.75f);


        return _decisionAction.ToArray();
    }

    public List<float> MakeMemory(
        List<float> vectorObs,
        List<Texture2D> visualObs,
        float reward,
        bool done,
        List<float> memory)
    {
        return new List<float>();
    }
}
