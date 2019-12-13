using System;
using UnityEngine;

class OdeGeomCapsule : OdeGeom
{
    private float m_radius = 0.5f;
    public float radius
    {
        get
        {
            float radius, length;
            UnityOde.odeGeomCapsuleGetParams(GeomId, out radius, out length);
            return radius;
        }
        set
        { 
            m_radius = value;
            UnityOde.odeGeomCapsuleSetParams(GeomId, m_radius, height);
        }
    }

    private float m_height = 2.0f;
    public float height
    {
        get
        {
            float radius, length;
            UnityOde.odeGeomCapsuleGetParams(GeomId, out radius, out length);
            return length;
        }
        set
        {
            m_height = value;
            UnityOde.odeGeomCapsuleSetParams(GeomId, radius, m_height);
        }
    }

    public static bool debug = false;
    private GameObject debugCapsule = null;

    protected override void Awake()
    {
        // TODO: these are currently not used
        int spaceId = -1;

        base.GeomId = UnityOde.odeCreateCapsule(spaceId, this.m_radius, this.m_height);
        //Debug.Log("Created OdeCapsuleGeometry: " + GeomId);

        if (debug)
        {
            debugCapsule = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            debugCapsule.name = "dbgCapsule_" + gameObject.name;
            //debugCapsule.transform.parent = transform;
            debugCapsule.transform.position = Vector3.zero;
            debugCapsule.transform.rotation = Quaternion.Euler(90.0f, 0.0f, 0.0f); 
        }

        base.Awake();
    }

    protected override void FixedUpdate()
    {
        if (debugCapsule != null)
        {
            if (debug)
            {
                UnityOde.setCurrentOdeContext(myContext);
                // default size of Unity capsule primitive is r=0.5 h=2.0
                float radiusF = m_radius / 0.5f;
                float heightF = m_height / 2.0f;
                Vector3 localScale = new Vector3(radiusF, heightF, radiusF);
                debugCapsule.transform.position = Body.position;
                debugCapsule.transform.rotation = Body.rotation * Quaternion.Euler(90.0f, 0.0f, 0.0f); //unity's capsule is aligned along y, whereas Ode's capsule is aligned along z

                debugCapsule.transform.localScale = localScale;
                debugCapsule.SetActive(true);
            }
            else
            {
                debugCapsule.SetActive(false);
            }
        }
    }

    public override void SetMass(float mass)
    {
        if (Body != null)
            UnityOde.odeMassSetCapsuleTotal(Body.BodyId, mass, radius, height);
        else
            Debug.LogError("Could not set mass to " + gameObject.name + " because there is no body attached");
    }

    public override void SetDensity(float density)
    {
        if (Body != null)
            UnityOde.odeMassSetCapsule(Body.BodyId, density, radius, height);
        else
            Debug.LogError("Could not set mass to " + gameObject.name + " because there is no body attached");
    }

    float Distance(Ray ray, out Vector3 c1, out Vector3 c2)
    {
        // Capsule axis is local Z-axis (from ODE docs)
        Vector3 dir = rotation * Vector3.forward;//transform.TransformDirection(Vector3.forward);
        // Calculate cylinder end points (note that height is cylinder height, without caps)
        float h = 0.5f * height;
        Vector3 a = position - h * dir;
        Vector3 b = position + h * dir;

        float s, t; //  parameters of closest points on segment and ray, respectively. s = [0,1], t = [0,inf)
        //Vector3 c1, c2; // closest points on segment and ray, respectively
        float sqDist = ClosestPtSegmentRay(a, b, ray, out s, out t, out c1, out c2);

        return Mathf.Sqrt(sqDist);
    }

    public float DistanceToSurface(Ray ray, out Vector3 point)
    {
        Vector3 c1, c2;
        float d = Distance(ray, out c1, out c2);

        // Use either distance to closest point or surface as "surface point"
        point = c2 + Mathf.Min(d, radius) * (c1 - c2);

        return Mathf.Max(0, d - radius);
    }

    public float DistanceToCenter(Ray ray, out Vector3 point)
    {
        Vector3 c1, c2;
        float d = Distance(ray, out c1, out c2);

        // Use point on center segment
        point = c1;

        return d;
    }

    // Computes closest points c1 and c2 on segment (p1, q1) and ray so that
    // Segment(s): c1 = p1 + s * (q1 - p1), s = [0,1]
    // Ray(t):     c2 = ray.origin + t * ray.direction, t = [0,inf)
    // Function result is squared distance between c1 and c2
    // Modified version of ClosestPtSegmentSegment of Real-Time Collision Detection by Christopher Ericson
    float ClosestPtSegmentRay(Vector3 p1, Vector3 q1, Ray ray, out float s, out float t, out Vector3 c1, out Vector3 c2)
    {
        Vector3 p2 = ray.origin;

        Vector3 d1 = q1 - p1; // Direction of vector of segment S1
        Vector3 d2 = ray.direction; // Direction of ray S2

        Vector3 r = p1 - p2;

        float a = Vector3.Dot(d1, d1); // Squared length of S1
        float f = Vector3.Dot(d2, r);

        float epsilon = 1e-6f;
        if (a <= epsilon)
        {
            // First segment degenerates into a point
            s = 0.0f;
            t = f;
        }
        else
        {
            float c = Vector3.Dot(d1, r);
            // The general nondegenerate case starts here
            float b = Vector3.Dot(d1, d2);
            float denom = a - b * b;

            // If segments not parallel, compute closest point on L1 to L2 and clamp to segment S1. Else pick arbitrary s (here 0)
            if (denom != 0.0f)
            {
                s = Mathf.Clamp((b * f - c) / denom, 0.0f, 1.0f);
            }
            else
            {
                s = 0.0f;
            }
            // Compute point on L2 closest to S1(s) using
            t = (b * s + f);

            // If t in [0,1] done. Else clamp t, recompute s for the new value of t using and clamp s to [0,1]
            if (t < 0.0f)
            {
                t = 0.0f;
                s = Mathf.Clamp(-c / a, 0.0f, 1.0f);
            }
        }

        c1 = p1 + d1 * s;
        c2 = p2 + d2 * t;
        return Vector3.Dot(c1 - c2, c1 - c2);
    }
}
