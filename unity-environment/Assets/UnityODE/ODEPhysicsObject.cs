using UnityEngine;
using System.Collections;


//Finds a sphere or box collider and creates the corresponding ODE geom
[AddComponentMenu("UnityODE/ODE Physics Object")]
public class ODEPhysicsObject : MonoBehaviour {
    public int geomId = -1;
    public Vector3 posShift = Vector3.zero;
    public bool debugVisualize = false;
    public bool dynamic = false;
    public float density = 1.0f;
    // Use this for initialization
	void Start () {
        if (ODEManager.instance == null)
            Debug.LogException(new System.Exception("ODE Manager missing, cannot create ODE objects!"));
        else
            initialize();
	}
	
    //The object managing physics will call this at the correct time
    public void initialize()
    {
        if (dynamic)
        {
            if (GetComponent<SphereCollider>() != null)
            {
                SphereCollider collider = GetComponent<SphereCollider>();
                float radius = collider.radius * Mathf.Max(collider.transform.lossyScale.x, Mathf.Max(collider.transform.lossyScale.y, collider.transform.lossyScale.z));
                var geom = gameObject.AddComponent<OdeGeomSphere>();
                var body = gameObject.AddComponent<OdeBody>();
                geom.debug = debugVisualize;
                geomId = geom.GeomId;// = UnityOde.odeCreateBox(size.x, size.y, size.z);
                UnityOde.odeGeomSphereSetRadius(geom.GeomId, radius);
                GameObject.Destroy(geom.debugSphere);//.transform.localScale = new Vector3(radius, radius, radius)*2.0f/transform.localScale.x;
                UnityOde.odeMassSetSphere(body.BodyId, density, radius);
                body.position = transform.position;
                body.rotation = transform.rotation;
                body.updateUnityToOdeTransform(); //body remembers its rotation and translation offsets relative to the controlled Unity transform
            }
            else if (GetComponent<BoxCollider>() != null)
            {
                BoxCollider collider = GetComponent<BoxCollider>();
                Vector3 size = new Vector3(collider.size.x * transform.lossyScale.x, collider.size.y * transform.lossyScale.y, collider.size.z * transform.lossyScale.z);
                Vector3 pos = transform.TransformPoint(collider.center);
                var geom = gameObject.AddComponent<OdeGeomBox>();
                var body = gameObject.AddComponent<OdeBody>();
                geomId = geom.GeomId;// = UnityOde.odeCreateBox(size.x, size.y, size.z);
                geom.lengths=size;
                body.position = pos;
                body.rotation = transform.rotation;
                body.updateUnityToOdeTransform(); //body remembers its rotation and translation offsets relative to the controlled Unity transform
            }
        }
        else
        {
            if (GetComponent<SphereCollider>() != null)
            {
                SphereCollider collider = GetComponent<SphereCollider>();
                float radius = collider.radius * Mathf.Max(collider.transform.lossyScale.x, Mathf.Max(collider.transform.lossyScale.y, collider.transform.lossyScale.z));
                Vector3 pos = transform.TransformPoint(collider.center);

                geomId = UnityOde.odeCreateSphere(radius);
                UnityOde.odeGeomSetPosition(geomId, pos.x, pos.y, pos.z);
                if (debugVisualize)
                {
                    GameObject debugSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    debugSphere.transform.localScale = new Vector3(radius, radius, radius) * 2.0f;
                    debugSphere.transform.position = pos;
                }
            }
            else if (GetComponent<BoxCollider>() != null)
            {
                BoxCollider collider = GetComponent<BoxCollider>();
                Vector3 size = new Vector3(collider.size.x * transform.lossyScale.x, collider.size.y * transform.lossyScale.y, collider.size.z * transform.lossyScale.z);
                Vector3 pos = transform.TransformPoint(collider.center);
                geomId = UnityOde.odeCreateBox(size.x, size.y, size.z);
                UnityOde.odeGeomSetPosition(geomId, pos.x, pos.y, pos.z);
                UnityOde.odeGeomSetQuaternion(geomId, transform.rotation);
                if (debugVisualize)
                {
                    GameObject debugBox = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    debugBox.name = "dbgBox_" + gameObject.name;
                    debugBox.transform.position = pos;
                    debugBox.transform.rotation = transform.rotation;
                    debugBox.transform.localScale = size;
                }

            }
        }
    }

}
