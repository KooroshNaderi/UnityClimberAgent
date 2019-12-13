using UnityEngine;
using System.Collections;

[AddComponentMenu("UnityODE/ODE Manager")]
public class ODEManager : MonoBehaviour {
    public static ODEManager instance;
    public int nContexts = 256; //number of thread contexts that can be simulated in parallel and independent of each other
    public bool createGroundPlane = true;
    public bool autoSimulate = true;
    public bool createDebugVisualizers = false;
    public int masterContext = 0;
    [HideInInspector]
    int groundPlaneGeomId;
    void Awake()
    {
        instance = this;
        //Init ODE
        UnityOde.initOde(nContexts);
        UnityOde.odeRandSetSeed((uint)System.DateTime.Now.Ticks);
        UnityOde.setCurrentOdeContext(UnityOde.ALLTHREADS);
        UnityOde.odeSetContactSoftCFM(0);
        UnityOde.odeWorldSetGravity(0, -9.81f, 0);
        //create ground plane
        if (createGroundPlane)
        {
            groundPlaneGeomId = UnityOde.odeCreatePlane(0, 0, 1, 0, 0);
            Debug.Log("Ground plane: " + groundPlaneGeomId);
            UnityOde.odeGeomSetCategoryBits(groundPlaneGeomId, 0xf0000000);
            UnityOde.odeGeomSetCollideBits(groundPlaneGeomId, 0xffffffff);
        }
        ////create other scene physics objects
        //AddOdeGeom[] aog = GameObject.FindObjectsOfType<AddOdeGeom>();
        //foreach (AddOdeGeom a in aog)
        //{
        //    a.initialize();
        //}
        OdeBody.debug = createDebugVisualizers;
        OdeGeomCapsule.debug = createDebugVisualizers;
        OdeGeomBox.debug = createDebugVisualizers;
    }
    //Note: script execution order for ODEManager is delayed => ODE simulation is stepped after other scripts
    //execute their FixedUpdates.
    //Thus, one can add ODE torques and set other simulation params in fixedupdate of other scripts.
    void FixedUpdate()
    {
        if (autoSimulate)
        {
            UnityOde.setCurrentOdeContext(masterContext);
            UnityOde.stepOde(Time.fixedDeltaTime, false);
        }
    }
    void OnApplicationQuit()
    {
        UnityOde.uninitOde();
    }
}
