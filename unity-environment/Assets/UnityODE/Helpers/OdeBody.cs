#define DEBUG

using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using System;
using AaltoGames;

public class OdeBody : MonoBehaviour {
    public int debugId;
    public Quaternion unityToOdeRotation;
    public Vector3 unityToOdeTranslation;
    public Quaternion bindRotation;
    private int bodyId = -1;
    static Dictionary<Transform, OdeBody> transformToBody = new Dictionary<Transform, OdeBody>();
    protected float lastFixedUpdateTime = -1;
    public float debugValue = 0;

    public int BodyId
    {
        get { return bodyId; }
        protected set { bodyId = debugId = value; }
    }

    
    public float mass
    {
        get
        {
            return UnityOde.odeBodyGetMass(bodyId);
        }
        set
        {
            UnityOde.odeBodySetMass(bodyId,mass);
        }
    }
   
    public bool isKinematic
    {
        get
        {
            return (bool)UnityOde.odeBodyIsKinematic(bodyId);
        }
        set
        {
            if (value == true)
                UnityOde.odeBodySetKinematic(bodyId);
            else
                UnityOde.odeBodySetDynamic(bodyId);
        }
    }

    public Vector3 position
    {
        get
        {
			return UnityOde.odeBodyGetPosition(bodyId);
        }
        set
        {
            UnityOde.odeBodySetPosition(bodyId, value.x, value.y, value.z);
        }
    }

    public string rotationFrom;

    public Quaternion rotation
    {
        get
        {
            return UnityOde.odeBodyGetQuaternion(bodyId);
        }
        set
        {
            if (!UnityOde.odeBodySetQuaternion(bodyId, value, false))
            {
                Debug.LogWarning("Can't set rotation " + rotationFrom);
            }
        }
    }
    public void updatePhysicsFromControlledObject()
    {
        updatePhysicsFromControlledRotationAndPosition(transform.position, transform.rotation);
        /*
        Quaternion q = transform.rotation;
        UnityOde.odeBodySetQuaternion(bodyId, q);

        Vector3 p = transform.position;
        UnityOde.odeBodySetPosition(bodyId, p.x, p.y, p.z);
         * */
    }
    public Vector3 velocity
    {
        get
        {
            return UnityOde.odeBodyGetLinearVel(bodyId);
        }
        set
        {
            UnityOde.odeBodySetLinearVel(bodyId, value.x, value.y, value.z);
        }
    }

    public Vector3 angularVelocity
    {
        get
        {
			return UnityOde.odeBodyGetAngularVel(bodyId);
        }
        set
        {
            UnityOde.odeBodySetAngularVel(bodyId, value.x, value.y, value.z);
        }
    }

    private Vector3 localOffset;
    public Vector3 LocalOffset
    {
        get { return localOffset; }
        set { localOffset = value; }
    }

    private OdeGeom geometry = null;
    public OdeGeom Geometry
    {
        get { return geometry; }
        set
        {
            // Check for changes
            if (geometry != value)
            {
                // Detach old
                if (geometry != null)
                {
                    UnityOde.odeGeomSetBody(geometry.GeomId, 0);
                }
                geometry = value;
                // Attach new
                if (geometry != null)
                {
                    //Debug.Log("Combining body " + bodyId + " and geom " + geometry.GeomId);
                    UnityOde.odeGeomSetBody(geometry.GeomId, BodyId);
                }
            }
        }
    }
    protected int myContext = 0;
	void Awake()
    {
        // TODO: these are currently not used
        myContext = UnityOde.getCurrentOdeContext();
        BodyId = UnityOde.odeBodyCreate();
        UnityOde.odeBodySetDynamic(bodyId);
        //UnityOde.odeBodySetMass(bodyId, this.mass, 0.1f, 0.1f);

        //Debug.Log("Created OdeBody: " + bodyId);
        transformToBody.Add(transform, this);
        OdeGeom geom = null;
        if ((geom = GetComponent<OdeGeom>()) != null)
        {
            if (geom.GeomId != -1)
            {
                Geometry = geom;
                Geometry.Body = this;
            }
        }
	}

    /*public void addForce(ref Vector3 force)
    {
        UnityOde.odeBodyAddForce(bodyId, force.oconv());
    }

    public void addForceAtPosition(ref Vector3 force, ref Vector3 at)
    {
        UnityOde.odeBodyAddForceAtPos(bodyId, force.oconv(), at.oconv());
    }

    public void addForceAtRelPosition(ref Vector3 force, ref Vector3 at)
    {
        UnityOde.odeBodyAddForceAtRelPos(bodyId, force.oconv(), at.oconv());
    }*/

    void Start()
    {
    }

    void OnDestroy()
    {
        if (UnityOde.initialized())
        {
            UnityOde.setCurrentOdeContext(myContext);
            UnityOde.odeBodyDestroy(BodyId);
        }
    }

    public bool disableUpdate { get; set; }
    static int lastUpdateBodyId = -1;
    
    void FixedUpdate()
    {
        if (disableUpdate)
        {
            return;
        }
        UnityOde.setCurrentOdeContext(myContext);

        //The calling order of body fixed updates is undetermined, hence we must use the
        //recursive rotation and position update (physics body transforms are in global coordinates, and
        //assigning them to a hierarchical character model must be done in parents first -order)

        // hack: let's only do this once per frame
        if (transform.parent == null)
        {
            updateTransformFromPhysicsRecursive(transform.root);
        }
        else if (lastUpdateBodyId == -1 || bodyId == lastUpdateBodyId)
        {
            updateTransformFromPhysicsRecursive(transform.root);
            lastUpdateBodyId = bodyId;
        }
	}
    
    public static void updateTransformFromPhysicsRecursive(Transform transform)
    {
        if (transformToBody.ContainsKey(transform))
        {
            OdeBody body = transformToBody[transform];
            if (body.lastFixedUpdateTime != Time.fixedTime)
            {
                body.updateTransfromFromPhysics();
                body.lastFixedUpdateTime = Time.fixedTime;
            }
        }
        for (int i=0; i<transform.childCount; i++)
        {
            updateTransformFromPhysicsRecursive(transform.GetChild(i));
        }
    }
    public void physicsToControlledTransform(Vector3 physPos, Quaternion physRot, out Vector3 transformPos, out Quaternion transformRot)
    {
        transformRot = physRot * Quaternion.Inverse(unityToOdeRotation);
        transformPos = physPos - (physRot * Quaternion.Inverse(bindRotation)) * unityToOdeTranslation;
    }
    //Note: in a hierarchical character, this must always be called parents first
    public void updateTransfromFromPhysics()
    {
        updateTransfromFromPhysics(position, rotation);
    }
    public void updateTransfromFromPhysics(Vector3 physPos,Quaternion physRot)
    {
        Vector3 transformPos;
        Quaternion transformRot;
        physicsToControlledTransform(physPos, physRot, out transformPos, out transformRot);
        transform.rotation = transformRot;
        transform.position = transformPos;
    }
    public Quaternion physicsToUnityRotation(Quaternion physRot)
    {
        return physRot * Quaternion.Inverse(unityToOdeRotation);
    }
    public void controlledToPhysics(Vector3 controlledPos, Quaternion controlledRot, out Vector3 odePos, out Quaternion odeRot)
    {
        odeRot = controlledRot * unityToOdeRotation;
        odePos = controlledPos + (odeRot * Quaternion.Inverse(bindRotation)) * unityToOdeTranslation;
    }
    public Quaternion unityToPhysicsRotation(Quaternion unityRotation)
    {
        return unityRotation * unityToOdeRotation;
    }
    public void updatePhysicsFromControlledRotationAndPosition(Vector3 pos,Quaternion q)
    {
        Quaternion physRot = q * unityToOdeRotation;
        rotation = physRot;
        position = pos + (physRot * Quaternion.Inverse(bindRotation)) * unityToOdeTranslation;
    }
    //sets the current translation and rotation offsets between the Ode body and its Unity transform as the defaults
    public void updateUnityToOdeTransform()
    {
        unityToOdeRotation = Quaternion.Inverse(transform.rotation) * rotation;
        bindRotation = rotation;
        unityToOdeTranslation = position-transform.position; 
    }

    public static bool debug = true;

    void OnDrawGizmos()
    {
        UnityOde.setCurrentOdeContext(myContext);
        if (ODEManager.instance!=null)
        {
            Gizmos.color = Color.blue;

            // Get ODE body position instead of offset one
			Vector3 v = UnityOde.odeBodyGetPosition(BodyId);
            Gizmos.DrawCube(v, new Vector3(0.05f, 0.05f, 0.05f));
        }

        if (debug)
        {
            UnityOde.setCurrentOdeContext(0);
            int c = UnityOde.odeGetContactCount();
            for(int i = 0; i < c; i++)
            {
                int body1, body2;
                Vector3 tmp = Vector3.zero;
                Vector3 v;
                UnityOde.odeGetContactInfo(i, out body1, out body2, out v, out tmp,out tmp);

                if (body1 == BodyId || body2 == bodyId)
                {
                    Gizmos.color = Color.green;
                    Gizmos.DrawCube(v, Vector3.one * 0.1f);
                    break;
                }
            }

            Vector3 start = transform.position;
            Vector3 end = start + velocity;

            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(start, end);

            Vector3 end2 = start + angularVelocity;
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(start, end2);
        }

        if (debugValue != 0)
        {
            Vector3 start = transform.position;
            Vector3 end = start + debugValue * Vector3.left;

            //Debug.Log("Value: " + debugValue + " this: " + name);

            if (debugValue < 1)
            {
                Gizmos.color = Color.Lerp(Color.red, Color.yellow, debugValue);
            }
            else if (debugValue < 1.5f)
            {
                Gizmos.color = Color.Lerp(Color.yellow, Color.green, (debugValue - 1) * 2);
            }
            else if (debugValue < 2)
            {
                Gizmos.color = Color.Lerp(Color.green, Color.blue, (debugValue - 1.5f) * 2);
            }
            else if (debugValue < 3)
            {
                Gizmos.color = Color.Lerp(Color.blue, Color.white, debugValue - 2);
            }
            else
            {
                Gizmos.color = Color.white;
            }

            Gizmos.DrawLine(start, end);
        }
    }
    void OnApplicationQuit()
    {
        transformToBody = null;  //prevent multiple restarts from Unity editor from leaking memory
    }
}
