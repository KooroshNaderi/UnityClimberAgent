using UnityEngine;
using System.Collections;
using AaltoGames;
using System;
using System.Runtime.InteropServices;
#if UNITY_EDITOR
using UnityEditorInternal;
#endif

public class Poser : MonoBehaviour
{
    public Transform targetCharacter;  //if null, the script will control the object it's attached to
    public bool constrainTo2d=true;
    public bool initPhysics = true;
    public bool createPhysicsRig = true;
    [HideInInspector]
    public Rig rig = new Rig(); 

    private bool changed;    //signals that the pose has changed and there's unsaved changes
    public bool Changed
    {
        get { return changed; }
    }
    private bool dragging;   // signals that handles are being dragged
    public bool Dragging
    {
        get { return dragging; }
    }
    private bool released;   // signals that handles have been released between this and the previous frame
    public bool Released
    {
        get { return released; }
    }
    public AnimHandle[] activeHandles = new AnimHandle[2] { null, null };
    public AnimHandle focusedHandle = null;
    Vector3 dragStartMouse3dPos = Vector3.zero;
    float dragStartTwist = 0;

    float twist = 0;
    AnimHandle[] closestHandles = new AnimHandle[2] { null, null };
    Vector3 mouse3dPos = Vector3.zero;
    Vector2 winapiCursorShift = Vector2.zero;
    float mouse3DSensitivity = 0.05f;
    public struct POINT
    {
        public int X;
        public int Y;

        public POINT(int x, int y)
        {
            this.X = x;
            this.Y = y;
        }

    }
    [DllImport("user32.dll")]
    private static extern IntPtr GetForegroundWindow();
    [DllImport("user32.dll")]
    static extern bool ClientToScreen(IntPtr hwnd, ref POINT lpPoint);
    [DllImport("user32.dll")]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool GetCursorPos(out POINT lpPoint);
    [DllImport("user32.dll")]
    static extern bool SetCursorPos(int X, int Y);
    // Or use System.Drawing.Point (Forms only)
    public Font fontTemplate;
    Font font = null, shadowFont = null;
    GUIStyle labelStyle = new GUIStyle();
    GUIStyle labelShadowStyle = new GUIStyle();
    [HideInInspector]
    public bool initialized = false;

    private Camera currentTransformationCamera = null;

    public Camera[] cameras;
    private float closestHandleDist = float.MaxValue;

    void Start()
    {
        if (cameras == null || cameras.Length == 0)
        {
            cameras = new Camera[] { Camera.main };
        }

        //no code here, initialize() may be called by some other script in its start to ensure proper initialization order
        initialize();
    }
    int myPhysicsContext = 0;
    public void initialize()
    {
        if (initialized)    //check if Start
            return;
        if (targetCharacter == null)
            targetCharacter = transform;
        rig.targetCharacter = targetCharacter;
        if (initPhysics)
        {
            UnityOde.initOde(1);  //+2 because of one master context and one extra save slot for reseting 
            UnityOde.setCurrentOdeContext(UnityOde.ALLTHREADS);
            UnityOde.odeSetContactSoftCFM(0);
            UnityOde.odeWorldSetGravity(0, -9.81f, 0);
        }
        rig.createPhysicsObjects = createPhysicsRig;
        if (targetCharacter.GetComponent<Animator>() != null)
        {
            Debug.Log("Poser: The target character rig type appears to be a Mecanim Biped, building...");
            myPhysicsContext = UnityOde.getCurrentOdeContext();
            rig.buildMecanimBipedRig(constrainTo2d);
        }
        else
        {
            throw new System.Exception("Poser.Start(): Target character missing the Mecanim Animator component!");
        }
        initialized = true;
    }
    void OnApplicationQuit()
    {
        if (initPhysics)
            UnityOde.uninitOde();
    }
    //call from OnRenderObject
    public void OnRenderObject()
    {
        GraphUtils.setLineMaterialPass(0, false);
        foreach (AnimHandle handle in rig.handles)
        {
            if (activeHandles[0] != handle && activeHandles[1] != handle && closestHandles[0] != handle && closestHandles[1] != handle)
            {
                if (handle.highlighted)
                {
                    GL.Color(Color.blue);
                }
                else if (focusedHandle == handle)
                {
                    GL.Color(Color.yellow);
                }
                else
                {
                    GL.Color(Color.gray);
                }
            }
            else
            {
                GL.Color(Color.white);
            }

            handle.render();
        }
        GraphUtils.setLineMaterialPass(0, true);
        for (int i = 0; i < 2; i++)
        {
            GL.Color(new Color(0, 0.5f, 0, 1.0f));
            if (activeHandles[i] != null)
            {
                GraphUtils.drawCrosshair(activeHandles[i].getPos(), 5.0f);
            }
        }
    }
    //returns true if scene changed
    public void Update()
    {
        UnityOde.setCurrentOdeContext(myPhysicsContext);
        bool result = false;
        released = false;
        int ctrlIdx = 0;
        Vector3 mousePosition = Input.mousePosition;

        if (activeHandles[ctrlIdx] == null)
        {
            POINT point = new POINT(0, 0);
            GetCursorPos(out point);
            winapiCursorShift = new Vector2((float)point.X - mousePosition.x, (float)point.Y - (Screen.height - mousePosition.y));
        }

        //twist
        twist += Input.GetAxis("Mouse ScrollWheel")*50;

        closestHandles[ctrlIdx] = null;
        if (activeHandles[ctrlIdx] == null)
        {
            float maxDist = 50;
            AnimHandle closest = null;
            Camera closestViewCamera = null;

            foreach (Camera camera in cameras)
            {
                foreach (AnimHandle handle in rig.handles)
                {
                    float dist = handle.distanceToScreenPoint(camera, mousePosition);

                    if (dist < maxDist)
                    {
                        maxDist = dist;
                        closest = handle;
                        closestViewCamera = camera;
                    }
                }
            }

            closestHandleDist = (closest == null) ? float.MaxValue : maxDist;

            if (otherPosers != null)
            {
                foreach (Poser other in otherPosers)
                {
                    if (checkOtherMouseCaptureCloserThan(other.closestHandleDist))
                    {
                        return;
                    }

                    other.checkOtherMouseCaptureCloserThan(closestHandleDist);
                }
            }

            closestHandles[ctrlIdx] = closest;
            //if no object selected, show plane cursor when howering over objects and select when lmb pressed
            if (closest != null)
            {
                if (Input.GetMouseButtonDown(0))
                {
                    focusedHandle = activeHandles[ctrlIdx] = closest;
                    mouse3dPos = closest.getPos();
                    closest.startDrag(mouse3dPos, Quaternion.identity, twist);
                    dragStartMouse3dPos = mouse3dPos;
                    dragStartTwist = twist;
                    currentTransformationCamera = closestViewCamera;
#if UNITY_EDITOR
                    Animator anim = targetCharacter.GetComponent<Animator>();
                    if (anim != null)
                    {
                        anim.StopPlayback();
                        UnityEditor.Animations.AnimatorController.SetAnimatorController(anim, null);
                    }
#endif
                }
            }
        }
        else //if we are dragging
        {
            Transform camTransform = currentTransformationCamera.transform;

            //if (Input.GetMouseButton(1))
            //	twist+=Input.GetAxis("Mouse X");
            //else
            mouse3dPos += camTransform.right * mouse3DSensitivity * Input.GetAxis("Mouse X");

            //move the 3d position
            if (Input.GetMouseButton(1))
            {
                mouse3dPos += camTransform.forward * mouse3DSensitivity * Input.GetAxis("Mouse Y");
            }
            else
            {
                mouse3dPos += camTransform.up * mouse3DSensitivity * Input.GetAxis("Mouse Y");
            }

            dragging = true;

            activeHandles[ctrlIdx].update(mouse3dPos, Quaternion.identity, twist);
            //detect changes to scene, add keyframe if needed
            if (mouse3dPos != dragStartMouse3dPos || twist != dragStartTwist)
            {
                result = true;
            }


            //update cursors
            mousePosition = currentTransformationCamera.WorldToScreenPoint(mouse3dPos);
            //end drag
            if (Input.GetMouseButtonUp(0))
            {
                for (int i = 0; i < 2; i++)
                {
                    activeHandles[i] = null;
                }

                released = true;
                dragging = false;
            }
        }
        changed=result;

        //If there are ODE bodies, make them follow the posing, allowing use to, e.g., query ODE motors for the Euler angles they think the posed character is in
        /*if (changed)
        {
            foreach (Rig.BoneData data in rig.bones)
            {
                if (data.rb != null)
                {
                    data.rb.updatePositionAndRotationFromTransform();
                }
            }
        }*/
        if (createPhysicsRig)
            rig.updatePhysicsFromUnityTransforms();
    }
    public void OnGUI()
    {
        if (activeHandles[0] != null)
        {
            Vector3 cursorPos = currentTransformationCamera.WorldToScreenPoint(mouse3dPos);

            POINT point = new POINT((int)cursorPos.x, (int)(Screen.height - cursorPos.y));
            //ClientToScreen(GetForegroundWindow(),ref point);
            SetCursorPos(point.X + (int)winapiCursorShift.x, point.Y + (int)winapiCursorShift.y);
        }

        //GUI.DrawTexture(new Rect(cursorPos.x,Screen.height-cursorPos.y,mouseCursorTexture.width,mouseCursorTexture.height),mouseCursorTexture,ScaleMode.ScaleAndCrop,true);
        for (int i = 0; i < 2; i++)
        {
            if (closestHandles[i] != null && fontTemplate != null)
            {
                //Vector3 pos=currentTransformationCamera.WorldToScreenPoint(move3dPos[i]);
                //if (!psMoveWrapper.enabled)
                Vector3 pos = Input.mousePosition + new Vector3(10, 0, 0);
                if (font == null)
                {
                    font = GameObject.Instantiate(fontTemplate) as Font;
                    labelStyle.normal.textColor = Color.white;
                    labelStyle.font = font;
                    shadowFont = GameObject.Instantiate(fontTemplate) as Font;
                    labelShadowStyle.font = shadowFont;
                    labelShadowStyle.normal.textColor = Color.gray;
                }
                GUI.Label(new Rect(pos.x + 1, Screen.height - pos.y - 10 + 1, 200, 20), closestHandles[i].getName(), labelShadowStyle);
                GUI.Label(new Rect(pos.x, Screen.height - pos.y - 10, 200, 20), closestHandles[i].getName(), labelStyle);
            }
        }
    }

    public bool isActiveHandle
    {
        get
        {
            return activeHandles[0] != null;
        }
    }

    public bool checkOtherMouseCaptureCloserThan(float dist)
    {
        if (dist > closestHandleDist)
        {
            return false;
        }

        closestHandles[0] = null;
        return true;
    }

    public Poser[] otherPosers;
};
