using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using AaltoGames;
using System.Threading;

public class LocomotionSamplingTest : MonoBehaviour {
    MecanimODERig rig;
    public float targetInterpolationTime = 0.2f;
    public int nFwdSteps = 20;
    const int nRollouts = 64;
    public bool useThreading = false;
    ManualResetEvent[] threadEvents=new ManualResetEvent[nRollouts];
    class FwdSimulationData
    {
        public float [] targetPose;
        public float reward;
        public Vector3 endPos;
        public System.Random rng = new System.Random(Random.Range(0,int.MaxValue));  //each thread has to have a different random generator seed
    };
    FwdSimulationData[] simdata = new FwdSimulationData[nRollouts];
    float timeStep;
	// Use this for initialization
	void Start () {
        if (ODEManager.instance == null)
            Debug.LogException(new System.Exception("ODE Manager missing, cannot create ODE objects!"));

        //find rig
        rig = GetComponent<MecanimODERig>();
        if (rig == null)
            Debug.LogException(new System.Exception("This script needs a MecanimODERig component on the same game object"));
        rig.initialize();

        //int threading
        for (int i = 0; i < nRollouts; i++)
        {
            threadEvents[i] = new ManualResetEvent(false);
            simdata[i] = new FwdSimulationData();
            simdata[i].targetPose = new float[rig.numControlDOFs()];

        }
        timeStep = Time.fixedDeltaTime; //cache Time.fixedDeltaTime, as accessing it from a background thread causes a freeze
	}

    void FixedUpdate()
    {
        // Reset events 
        for (int i = 0; i < threadEvents.Length; i++) 
        { 
            threadEvents[i].Reset(); 
        }

        //Prepare for forward simulation
        UnityOde.setCurrentOdeContext(0);
        UnityOde.saveOdeState(0);


        //Launch threads
        for (int i = 0; i < nRollouts; i++)
        {
            if (useThreading)
                ThreadPool.QueueUserWorkItem(threadProc, i);
            else
                threadProc(i);
        }

        //Wait for threads to finish
        if (useThreading)
            WaitHandle.WaitAll(threadEvents);

        //Restore master context
        UnityOde.setCurrentOdeContext(0);
        UnityOde.restoreOdeState(0);

        //Find thread with best cost, also debug draw
        float bestReward=simdata[0].reward;
        int bestIdx=0;
        Vector3 com=rig.COM();
        for (int i = 1; i < nRollouts; i++)
        {
            Debug.DrawLine(rig.COM(),simdata[i].endPos,Color.green);
            if (simdata[i].reward>bestReward)
            {
                bestIdx=i;
                bestReward=simdata[i].reward;
            }
        }

        //Repeat the action of the best thread
        rig.driveToPose(simdata[bestIdx].targetPose,targetInterpolationTime);
        UnityOde.stepOde(Time.fixedDeltaTime,false);

    }

    void threadProc(object o)
    {
        int contextIdx=(int)o;
        //activate simulation context for this thread and load simulation state
        UnityOde.setCurrentOdeContext(contextIdx);
        UnityOde.restoreOdeState(0);

        //randomize action (we do it here, as Random is not thread-safe
        for (int i = 0; i < simdata[contextIdx].targetPose.Length; i++)
        {
            //randomize target angles between limits
            //NOTE: Unity's Random is not thread-safe, so we have to use System.Random
            simdata[contextIdx].targetPose[i] = rig.angleMinLimits()[i] + ((float)simdata[contextIdx].rng.NextDouble()) * (rig.angleMaxLimits()[i] - rig.angleMinLimits()[i]);
        }


        //take action, simulate
        rig.driveToPose(simdata[contextIdx].targetPose, targetInterpolationTime);
        Vector3 startPos = rig.COM();
        for (int i = 0; i < nFwdSteps; i++)
        {
            UnityOde.stepOde(timeStep, false);
        }

        ////compute reward, here simply the distance traveled in positive z axis direction
        simdata[contextIdx].endPos = rig.COM();
        simdata[contextIdx].reward = Mathf.Max(0, simdata[contextIdx].endPos.z - startPos.z);

        //signal that this thread is done
        threadEvents[contextIdx].Set();
    }

}
