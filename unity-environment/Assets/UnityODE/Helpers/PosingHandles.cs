
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;

namespace AaltoGames
{
    public abstract class AnimHandle
    {
        //methods for finding the closest handle to a 2d or 3d cursor
        public abstract float distanceToScreenPoint(Camera camera, Vector2 pt);
        public abstract float distanceToWorldPoint(Vector3 pt);
        public abstract Vector3 getPos();
        public abstract void startDrag(Vector3 cursorPos, Quaternion q, float twist);
        public abstract void update(Vector3 cursorPos, Quaternion q, float twist);
        public abstract void render();
        public abstract string getName();
        public abstract Transform getFocusTarget();

        public bool highlighted = false;
    };
    public class TranslationHandle : AnimHandle
    {
        Transform transform;

        Vector3 cursorOffset;
        //		bool dragging=false;

        string name;
        bool constrainTo2d;
        Vector3 startPosition;
        public TranslationHandle(string name, Transform transform, bool constrainTo2d = false)
        {
            constrainTo2d = false;
            this.name = name;
            this.transform = transform;
            startPosition = transform.position;
            this.constrainTo2d = constrainTo2d;
            //lineMaterial = new Material(Shader.Find("Custom/GizmoShader"));
            //lineMaterial = new Material("Shader \"Lines/Colored Blended\" {" + "SubShader { Pass { " + "    Blend SrcAlpha OneMinusSrcAlpha " + "    ZWrite Off Cull Off Fog { Mode Off } " + "    BindChannels {" + "      Bind \"vertex\", vertex Bind \"color\", color }" + "} } }");
            //lineMaterialZTestOff.hideFlags = HideFlags.HideAndDontSave;
            //lineMaterialZTestOff.shader.hideFlags = HideFlags.HideAndDontSave;

            //lineMaterial = new Material("Shader \"Lines/Colored Blended\" {" +
            //    "SubShader { Pass {" +
            //    "   BindChannels { Bind \"Color\",color }" +
            //    "   Blend SrcAlpha OneMinusSrcAlpha" +
            //    "   ZTest Off ZWrite Off Cull Off Fog { Mode Off }" +
            //    "} } }");
            //lineMaterial.hideFlags = HideFlags.HideAndDontSave;
            //lineMaterial.shader.hideFlags = HideFlags.HideAndDontSave;
        }
        //methods for finding the closest handle to a 2d or 3d cursor
        public override float distanceToScreenPoint(Camera camera, Vector2 pt)
        {
            return (camera.WorldToScreenPoint(transform.position) - (Vector3)pt).magnitude;
        }
        public override float distanceToWorldPoint(Vector3 pt)
        {
            return (transform.position - pt).magnitude;
        }
        public override Vector3 getPos()
        {
            return transform.position;
        }
        public override void startDrag(Vector3 cursorPos, Quaternion q, float twist)
        {
            cursorOffset = cursorPos - transform.position;
            //			dragging=true;
        }
        public override void update(Vector3 cursorPos, Quaternion q, float twist)
        {
            if (constrainTo2d)
            {
                Vector3 pos = cursorPos - cursorOffset;
                pos.z = startPosition.z;
                transform.position = pos;

            }
            else
            {
                transform.position = cursorPos - cursorOffset;
            }
        }
        public override void render()
        {
            GraphUtils.setLineMaterialPass(0, true);
            GraphUtils.drawCube(transform.position, 0.2f, transform.up, transform.right, transform.forward);
        }
        public override string getName()
        {
            return name;
        }
        public override Transform getFocusTarget()
        {
            return transform;
        }
    };

    public class RotationHandle : AnimHandle
    {
        Transform transform;
        Vector3 cursorOffset;
        float dragStartTwist;
        Quaternion dragStartRotation;
        //		bool dragging=false;
        Material lineMaterial;
        Vector3 localHandleEnd;
        string name;
        bool constrainTo2d;
        Vector3 startPosition;
        public RotationHandle(string name, Transform transform, Vector3 handleEnd, bool constrainTo2d = false)
        {
            constrainTo2d = false;

            this.name = name;
            this.constrainTo2d = constrainTo2d;
            startPosition = transform.position;
            if (constrainTo2d)
                handleEnd.z = startPosition.z;
            localHandleEnd = transform.InverseTransformPoint(handleEnd);
            if (localHandleEnd.magnitude < 0.07f)
                localHandleEnd *= 0.07f / localHandleEnd.magnitude;
            this.transform = transform;
            lineMaterial = new Material("Shader \"Lines/Colored Blended\" {" +
                "SubShader { Pass {" +
                "   BindChannels { Bind \"Color\",color }" +
                "   Blend SrcAlpha OneMinusSrcAlpha" +
                "   ZTest Off ZWrite Off Cull Off Fog { Mode Off }" +
                "} } }");
            lineMaterial.hideFlags = HideFlags.HideAndDontSave;
            lineMaterial.shader.hideFlags = HideFlags.HideAndDontSave;
        }
        //methods for finding the closest handle to a 2d or 3d cursor
        public override float distanceToScreenPoint(Camera camera, Vector2 pt)
        {
            return (camera.WorldToScreenPoint(getPos()) - (Vector3)pt).magnitude;
        }
        public override float distanceToWorldPoint(Vector3 pt)
        {
            return (getPos() - pt).magnitude;
        }
        public override Vector3 getPos()
        {
            return transform.TransformPoint(localHandleEnd);
        }
        public override void startDrag(Vector3 cursorPos, Quaternion q, float twist)
        {
            cursorOffset = cursorPos - getPos();
            //			dragging=true;
            dragStartTwist = twist;
            dragStartRotation = transform.rotation;
        }
        public override void update(Vector3 cursorPos, Quaternion q, float twist)
        {
            transform.rotation = dragStartRotation;
            Vector3 rootToMouse = cursorPos - cursorOffset - transform.position;
            if (constrainTo2d)
                rootToMouse.z = 0;
            Vector3 currentDirection = rootToMouse.normalized;
            Vector3 dragStartDirection = (getPos() - transform.position).normalized;

            transform.rotation = Quaternion.FromToRotation(dragStartDirection, currentDirection) * transform.rotation;
            transform.Rotate((getPos() - transform.position).normalized, twist - dragStartTwist);

        }
        public override void render()
        {
            GraphUtils.setLineMaterialPass(0, true);
            GraphUtils.drawCube(getPos(), 0.05f, transform.up, transform.right, transform.forward);
            GL.Begin(GL.LINES);
            GL.Vertex(transform.position);
            GL.Vertex(getPos());
            GL.End();
        }
        public override string getName()
        {
            return name;
        }
        public override Transform getFocusTarget()
        {
            return transform;
        }
    };

    public class ArmLegIKHandle : AnimHandle
    {
        Transform shoulder, elbow;
        Vector3 cursorOffset;
        float dragStartTwist;
        //		bool dragging=false;
        Material lineMaterial;
        Vector3 localWristPos;
        //Vector3 localElbowEulers;
        //Quaternion localElbowRotation;
        float anglea, angleb;
        string name;
        Vector3 elbowLocalAxis;
        bool constrainTo2d;
        Vector3 startPosition;
        //for max biped: anglea=Mathf.PI, angleb=-1.0f
        public ArmLegIKHandle(string name, Transform shoulder, Transform elbow, Transform wrist, float anglea, float angleb, Vector3 elbowLocalAxis, bool constrainTo2d = false)
        {
            constrainTo2d = false;
            this.constrainTo2d = constrainTo2d;
            if (constrainTo2d)
            {
                float meanz = (shoulder.position.z + elbow.position.z + wrist.position.z) / 3.0f;
                shoulder.position = new Vector3(shoulder.position.x, shoulder.position.y, meanz);
                elbow.position = new Vector3(elbow.position.x, elbow.position.y, meanz);
                wrist.position = new Vector3(wrist.position.x, wrist.position.y, meanz);
            }
            startPosition = wrist.position;
            this.name = name;
            this.shoulder = shoulder;
            this.elbow = elbow;
            this.anglea = anglea;
            this.angleb = angleb;
            this.elbowLocalAxis = elbowLocalAxis;
//            Debug.Log("ArmLegIKHandle elbow bone " + elbow.name + ", local rotation axis " + elbowLocalAxis);
            //this.localElbowEulers = elbow.localEulerAngles;
         //   initialLocalElbowEulers = elbow.localEulerAngles;
            //this.localElbowRotation = elbow.localRotation;
            localWristPos = elbow.InverseTransformPoint(wrist.position); // wrist position in elbow local space
            lineMaterial = new Material("Shader \"Lines/Colored Blended\" {" +
                "SubShader { Pass {" +
                "   BindChannels { Bind \"Color\",color }" +
                "   Blend SrcAlpha OneMinusSrcAlpha" +
                "   ZTest Off ZWrite Off Cull Off Fog { Mode Off }" +
                "} } }");
            lineMaterial.hideFlags = HideFlags.HideAndDontSave;
            lineMaterial.shader.hideFlags = HideFlags.HideAndDontSave;
        }
        //methods for finding the closest handle to a 2d or 3d cursor
        public override float distanceToScreenPoint(Camera camera, Vector2 pt)
        {
            return (camera.WorldToScreenPoint(getPos()) - (Vector3)pt).magnitude;
        }
        public override float distanceToWorldPoint(Vector3 pt)
        {
            return (getPos() - pt).magnitude;
        }
        public override Vector3 getPos()
        {
            return elbow.TransformPoint(localWristPos);
        }
        public override void startDrag(Vector3 cursorPos, Quaternion q, float twist)
        {
            if (constrainTo2d)
                cursorPos.z = startPosition.z;
            cursorOffset = cursorPos - getPos();
            //			dragging=true;
            dragStartTwist = twist;

        }
        public override void update(Vector3 cursorPos, Quaternion q, float twist)
        {
            //2 bone ik
            //shoulder.rotation=dragStartShoulderRotation;
            //elbow.rotation=dragStartElbowRotation;
            if (constrainTo2d)
                cursorPos.z = startPosition.z;
            //elbow/knee angle based on bone lengths and distance using basic trig
            Vector3 target = cursorPos - cursorOffset;
            Vector3 hand = getPos();
            Vector3 shoulderToElbow = shoulder.position - elbow.position;
            Vector3 shoulderToTarget = target - shoulder.position;
            Vector3 elbowToHand = hand - elbow.position;

            //cosine rule to find elbow angle: c²=a²+b²-2abcos(C), where C is the angle opposite C (between sides a and b)
            float a = shoulderToElbow.magnitude;
            float b = elbowToHand.magnitude;
            float c = shoulderToTarget.magnitude;
            float elbowAngleTry = Mathf.Acos((a * a + b * b - c * c) / (2.0f * a * b));
            if (float.IsNaN(elbowAngleTry) || float.IsInfinity(elbowAngleTry))
                elbowAngleTry = Mathf.PI;
            //float plainElbowAngle = elbowAngleTry;
            elbowAngleTry = anglea + angleb * elbowAngleTry;

            elbow.localRotation = Quaternion.AngleAxis(Mathf.Rad2Deg * elbowAngleTry, elbowLocalAxis);
            //rotate so that end effector meets target
            Vector3 rootToMouse = (cursorPos - shoulder.position);
            Vector3 currentDirection = rootToMouse.normalized;
            Vector3 origDirection = (getPos() - shoulder.position).normalized;
            shoulder.rotation = Quaternion.FromToRotation(origDirection, currentDirection) * shoulder.rotation;

            //Twist
            if (!constrainTo2d)
            {
                //Debug.Log("Twist: " + twist);
                shoulder.Rotate(currentDirection, twist - dragStartTwist);
                dragStartTwist = twist;
            }
        }
        public override void render()
        {
            GraphUtils.setLineMaterialPass(0, true);
            GraphUtils.drawCube(getPos(), 0.05f, elbow.up, elbow.right, elbow.forward);
            GL.Begin(GL.LINES);
            GL.Vertex(shoulder.position);
            GL.Vertex(elbow.position);
            GL.Vertex(elbow.position);
            GL.Vertex(getPos());
            GL.End();
        }
        public override string getName()
        {
            return name;
        }
        public override Transform getFocusTarget()
        {
            return elbow;
        }
    };
}
