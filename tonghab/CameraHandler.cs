using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System;


public class CameraHandler : MonoBehaviour
{
    //pjh
    [Serializable]
    struct CameraOffset
    {
        public string pierName;
        public Vector2[] angles;
        public float[] distances;
    }
    [SerializeField]
    CameraOffset[] cameraOffsets;
    //
    public CraneInfo craneInfo;
    public SelectionManager selectionManager;
    public Transform target;

    public float distance = 300;
    public Vector2 cameraAngle = new Vector2(114, 15);
    private Vector3 cameraOffset = new Vector3();

    public float yMinLimit = -20f;
    public float yMaxLimit = 80f;

    public float rotationSpeed = 5.0f;
    public float translationSpeed = 5.0f;
    public float zoomSpeed = 100.0f;
    
    private bool bUpdateCrane = true;
    private int pierSelected = 0;
    private int craneSelected = 0;
    private int currentPier = 0;
    void Start()
    {
        string currentStringPier = CsCore.Configuration.ReadConfigIni("Initial Pier", "Value");
        currentPier = PierUtility.PierNum(currentStringPier);
       // pierSelected = selectionManager.currentPier;
       pierSelected = currentPier;
        craneSelected = selectionManager.currentCrane;
    }

    void Update()
    {
        UpdateTarget();
        if (selectionManager.IsIgnoreKey() == false)
        {
            Zoom();
            Rotate();
        }
        UpdateCamera();
    }
    private void Zoom()
    {
        if (!EventSystem.current.IsPointerOverGameObject())
        {
            float d = Input.GetAxis("Mouse ScrollWheel") * -1 * zoomSpeed;
            if (d != 0)
            {
                distance += d;
            }
        }

        if (Input.GetKey(KeyCode.LeftBracket) ||
            Input.GetKey(KeyCode.Minus) ||
            Input.GetKey(KeyCode.KeypadMinus))
        {
            distance += zoomSpeed * 0.1f;
        }

        if (Input.GetKey(KeyCode.RightBracket)
            || Input.GetKey(KeyCode.Plus) 
            || Input.GetKey(KeyCode.KeypadPlus))
        {
            distance -= zoomSpeed * 0.1f;
        }
    }
    public void UpdateTargetCrane(int pier, int crane)
    {
        pierSelected = pier;
        craneSelected = crane;
        bUpdateCrane = true;
    }
    private void UpdateTarget()
    {
        // 카메라 리셋
        if (bUpdateCrane)
        {
            if(pierSelected > cameraOffsets.Length) Debug.LogError("cannot find cameraOffset "+ pierSelected);
            if (craneInfo.Contains(pierSelected, craneSelected))
            {
                //pjh
                cameraOffset = new Vector2();
                distance = cameraOffsets[pierSelected-1].distances[0];
                cameraAngle = cameraOffsets[pierSelected-1].angles[0];
                bUpdateCrane = false;
                //
            }
            else
            {
                //pjh
                cameraOffset = new Vector2();
                distance = cameraOffsets[pierSelected-1].distances.Length > 1 ? cameraOffsets[pierSelected-1].distances[1] : cameraOffsets[pierSelected-1].distances[0]; 
                cameraAngle = cameraOffsets[pierSelected-1].angles.Length > 1 ? cameraOffsets[pierSelected-1].angles[1] : cameraOffsets[pierSelected-1].angles[0];
                bUpdateCrane = false;
                //
            }
        }

        // 대상 위치 갱신
        Transform tf = target;
        if (craneInfo.Contains(pierSelected, craneSelected))
        {
            int key = craneInfo.GetCraneKeycode(pierSelected, craneSelected);
            GameObject craneObject = craneInfo.dictCraneGameObject[key];
            if (craneObject.activeSelf)
            {
                tf = craneObject.transform;
            }
        }
        else
        {
            if(0 < craneInfo.pierCenter.Count && pierSelected < craneInfo.pierCenter.Count)
            {
                GameObject gameObject = craneInfo.pierCenter[pierSelected];
                if(gameObject != null)
                {
                    tf = gameObject.transform;
                }
            }
        }

        target = tf;
    }

    private void Rotate()
    {
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            cameraAngle.x += rotationSpeed * 0.5f;
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
            cameraAngle.x -= rotationSpeed * 0.5f;
        }
        if (Input.GetKey(KeyCode.UpArrow))
        {
            cameraAngle.y += rotationSpeed * 0.5f;
            cameraAngle.y = ClampAngle(cameraAngle.y, yMinLimit, yMaxLimit);
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            cameraAngle.y -= rotationSpeed * 0.5f;
            cameraAngle.y = ClampAngle(cameraAngle.y, yMinLimit, yMaxLimit);
        }
        if (Input.GetKey(KeyCode.W))
        {
            float dx = 0;
            float dy = 0;
            float dz = translationSpeed;

            Vector3 vecMove = new Vector3(dx, dy, dz);
            Quaternion rot = Quaternion.Euler(cameraAngle.y, cameraAngle.x, 0);
            vecMove = rot * vecMove;

            cameraOffset += vecMove;
        }
        if (Input.GetKey(KeyCode.S))
        {
            float dx = 0;
            float dy = 0;
            float dz = -translationSpeed;

            Vector3 vecMove = new Vector3(dx, dy, dz);
            Quaternion rot = Quaternion.Euler(cameraAngle.y, cameraAngle.x, 0);
            vecMove = rot * vecMove;

            cameraOffset += vecMove;
        }
        if (Input.GetKey(KeyCode.A))
        {
            float dx = -translationSpeed;
            float dy = 0;
            float dz = 0;

            Vector3 vecMove = new Vector3(dx, dy, dz);
            Quaternion rot = Quaternion.Euler(cameraAngle.y, cameraAngle.x, 0);
            vecMove = rot * vecMove;

            cameraOffset += vecMove;
        }
        if (Input.GetKey(KeyCode.D))
        {
            float dx = translationSpeed;
            float dy = 0;
            float dz = 0;

            Vector3 vecMove = new Vector3(dx, dy, dz);
            Quaternion rot = Quaternion.Euler(cameraAngle.y, cameraAngle.x, 0);
            vecMove = rot * vecMove;

            cameraOffset += vecMove;
        }

        if (Input.GetKey(KeyCode.Q))
        {
            float dx = 0;
            float dy = -translationSpeed;
            float dz = 0;

            Vector3 vecMove = new Vector3(dx, dy, dz);
            Quaternion rot = Quaternion.Euler(cameraAngle.y, cameraAngle.x, 0);
            vecMove = rot * vecMove;

            cameraOffset += vecMove;
        }

        if (Input.GetKey(KeyCode.E))
        {
            float dx = 0;
            float dy = translationSpeed;
            float dz = 0;

            Vector3 vecMove = new Vector3(dx, dy, dz);
            Quaternion rot = Quaternion.Euler(cameraAngle.y, cameraAngle.x, 0);
            vecMove = rot * vecMove;

            cameraOffset += vecMove;
        }

        if (!EventSystem.current.IsPointerOverGameObject())
        {
            if (Input.GetMouseButton(1))
            {
                cameraAngle.x += Input.GetAxis("Mouse X") * rotationSpeed;
                cameraAngle.y -= Input.GetAxis("Mouse Y") * rotationSpeed;
                cameraAngle.y = ClampAngle(cameraAngle.y, yMinLimit, yMaxLimit);
            }
            if (Input.GetMouseButton(2))
            {
                float dx = -Input.GetAxis("Mouse X") * translationSpeed;
                float dy = -Input.GetAxis("Mouse Y") * translationSpeed;

                Vector3 vecMove = new Vector3(dx, dy, 0);
                Quaternion rot = Quaternion.Euler(cameraAngle.y, cameraAngle.x, 0);
                vecMove = rot * vecMove;

                cameraOffset += vecMove;
            }
        }
    }
    float ClampAngle(float angle, float min, float max)
    {
        while (angle < -360) angle += 360;
        while (angle > 360) angle -= 360;
        return Mathf.Clamp(angle, min, max);
    }
    private void UpdateCamera()
    {
        Vector3 vecFront = Vector3.forward;
        Quaternion rot = Quaternion.Euler(cameraAngle.y, cameraAngle.x, 0);
        vecFront = rot * vecFront;
        transform.position = target.position - distance * vecFront + cameraOffset;

        Quaternion q = Quaternion.LookRotation(vecFront);
        transform.rotation = q;
    }
}
