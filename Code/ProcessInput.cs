using System;
using System.Collections;
using System.Collections.Generic;
using CsCore;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class ProcessInput : MonoBehaviour
{
    public ProcessCamera cameraControl;
    public ProcessAlarm processAlarm;
    public LogControlMenuBar logControlMenuBar;
    public ProcessInterface clientCollisionProcessor;
    public ProcessCraneMonitoring processCraneMonitoring;
    public bool blogControlMenuBar = false;
    private Vector2 clickPoint;
    private float dragSpeed = 25.0f;

    private void Awake()
    {
    }
    private void Update()
    {
        UpdateMouseDrag();
        UpdateTouch();
        //UpdateMouseLeftClick();
        UpdateMouseMiddleClick();
        UpdateMouseWheel();

        if (Input.GetKeyUp(KeyCode.L))
        {
            logControlMenuBar.gameObject.SetActive(!logControlMenuBar.gameObject.activeSelf);
            clientCollisionProcessor.bIgnoreCommunication = !clientCollisionProcessor.bIgnoreCommunication;
            blogControlMenuBar = !blogControlMenuBar;
            logControlMenuBar.SetLogStatus(blogControlMenuBar);
            processCraneMonitoring.SetStartBit(!blogControlMenuBar);
        }
    }

    private void UpdateMouseRightClick()
    {
        if(Input.GetMouseButton(1))
        {
            if (!EventSystem.current.IsPointerOverGameObject())
            {
                float mouseX = Input.GetAxis("Mouse X");
                float mouseY = Input.GetAxis("Mouse Y");
                int screenNum = cameraControl.GetScreenNum(Input.mousePosition);
                cameraControl.Rotate(mouseX, mouseY, screenNum);
            }
        }
    }

    private void UpdateTouch()
    {
        if (Input.touchSupported)
        {
            if (Input.touchCount == 1)
            {
                Touch touch = Input.GetTouch(0);
                if (touch.phase == TouchPhase.Moved)
                {
                    int screenNum = cameraControl.GetScreenNum(Input.mousePosition);
                    cameraControl.RotateAround(touch.deltaPosition.x/* * 0.1f*/, touch.deltaPosition.y/* * 0.1f*/, screenNum);
                }
            }
            else if (Input.touchCount > 1)
            {
                Touch touchA = Input.GetTouch(0);
                Touch touchB = Input.GetTouch(1);
                if (touchA.phase == TouchPhase.Moved || touchB.phase == TouchPhase.Moved)
                {
                    Vector2 dA = touchA.deltaPosition;
                    Vector2 pA = touchA.position;
                    Vector2 pA0 = touchA.position - dA;

                    Vector2 dB = touchB.deltaPosition;
                    Vector2 pB = touchB.position;
                    Vector2 pB0 = touchB.position - dB;

                    float length0 = (pB0 - pA0).magnitude;
                    float length = (pB - pA).magnitude;

                    float magnitude = length - length0;
                    if(Mathf.Abs(magnitude) > 5)
                    {
                        cameraControl.Zoom(magnitude * 0.03f);
                    }
                    else
                    {
                        cameraControl.Move(dA.x * 0.03f, dA.y * 0.03f);
                    }
                }
                else
                {
                }
            }
        }
    }

    private void UpdateMouseLeftClick()
    {
        if (Input.GetMouseButton(0))
        {
            if (!EventSystem.current.IsPointerOverGameObject())
            {

                Debug.Log("Left Mouse x = " + Input.GetAxis("Mouse X") + " y = " + Input.GetAxis("Mouse Y"));
                {
                    float mouseX = Input.GetAxis("Mouse X");
                    float mouseY = Input.GetAxis("Mouse Y");
                    int screenNum = cameraControl.GetScreenNum(Input.mousePosition);
                    cameraControl.RotateAround(mouseX, mouseY, screenNum);
                }
            }
        }
    }

    private void UpdateMouseMiddleClick()
    {
        if (Input.GetMouseButton(2))
        {
            if (!EventSystem.current.IsPointerOverGameObject())
            {
                float mouseX = Input.GetAxis("Mouse X");
                float mouseY = Input.GetAxis("Mouse Y");
                int screenNum = cameraControl.GetScreenNum(Input.mousePosition);
                cameraControl.Move(mouseX, mouseY);
            }
        }
    }

    private void UpdateMouseWheel()
    {
        float MouseWheel = Input.GetAxis("Mouse ScrollWheel");
        if(MouseWheel != 0)cameraControl.Zoom(MouseWheel);
    }

    private void UpdateMouseDrag()
    {
        if (Camera.main == null)
            return;
        if(Input.GetMouseButtonDown(0))
        {
            clickPoint = Input.mousePosition;
        }

        if (!EventSystem.current.IsPointerOverGameObject())
        {
            if (Input.GetMouseButton(0))
            {
                Vector3 position = Camera.main.ScreenToViewportPoint((Vector2)Input.mousePosition - clickPoint);
                Vector3 move = position * (0.01f * dragSpeed);

                int screenNum = cameraControl.GetScreenNum(Input.mousePosition);
                cameraControl.RotateAround(move.x * dragSpeed, move.y * dragSpeed, screenNum);
            }
        }
    }

    private bool IsMouseOverGUIElement()
    {
        Ray ray = cameraControl.mainCamera.ScreenPointToRay(Input.mousePosition);
        RaycastHit hitInfo;

        if (Physics.Raycast(ray, out hitInfo))
        {
            GameObject hitObject = hitInfo.collider.gameObject;
            //EventTrigger trigger = hitInfo.collider.GetComponent<EventTrigger>();

            //if (trigger != null)
            //{
            //    return true;
            //}
        }

        return false;
    }

    private bool IsMouseOverGUI2D()
    {
        if (EventSystem.current.IsPointerOverGameObject())
        {
            return true;
        }
        return false;
    }
}

