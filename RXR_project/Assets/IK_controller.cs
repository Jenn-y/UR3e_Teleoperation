using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using TMPro;
using System.IO;
using System.Linq;
using System;

public class IK_Controller : MonoBehaviour
{
    public InputDevice _rightController;
    public GameObject RobotBase;
    private GameObject[] jointList = new GameObject[6];
    private UR3_Solver Robot = new();
    private APIManager aPIManager = new();
    public Transform Target;
    public TMP_Text text;
    public float repeatRate = 4f;
    public bool waspressed = false;
    public Vector3 origPosition;
    public Vector3 currOrig = new Vector3(0.1362f, 0.3557f, 0.0738f);
    
    // Start is called before the first frame update
    void Start()
    {
        InitializeJoints();
        if(!_rightController.isValid)
        {
            InitializeInputDevice(InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Right, ref _rightController);
        }
        InvokeRepeating("UpdatePosition", 0.5f, repeatRate);
    }

    void InitializeJoints()
    {
        var RobotChildren = RobotBase.GetComponentsInChildren<Transform>();
        for (int i = 1; i < RobotChildren.Length; i++)
        {   
            jointList[i-1] = RobotChildren[i].gameObject;
        }
    }

    private void InitializeInputDevice(InputDeviceCharacteristics inputCharacteristics, ref InputDevice inputDevice)
    {
        List<InputDevice> devices = new();
        //Call InputDevices to see if it can find any devices with the characteristics we're looking for
        InputDevices.GetDevicesWithCharacteristics(inputCharacteristics, devices);

        //Our hands might not be active and so they will not be generated from the search.
        //We check if any devices are found here to avoid errors.
        if (devices.Count > 0)
        {
            inputDevice = devices[0];
        }
    }

    void UpdatePosition()
    {
        bool isIndexFingerPressed;
        if (_rightController.TryGetFeatureValue(CommonUsages.triggerButton, out isIndexFingerPressed) && isIndexFingerPressed)
        {
            if (waspressed == false)
            {
                waspressed = true;
                origPosition = Target.position;
            }
            ResolveIK();

        }
        else
        {
            if (waspressed == true)
            {
                currOrig = currOrig + Target.position - origPosition;
            }
            waspressed = false;
        }
    }

    private void ResolveIK()
    {
        if (Target == null)
            return;

        var targetPosition = currOrig + Target.position - origPosition;
        var targetRotation = Target.rotation;
        Vector3 euler = targetRotation.eulerAngles;

        Robot.SolvePhysicalRobot(targetPosition.z, -targetPosition.y, -targetPosition.x, euler.x * Mathf.Deg2Rad, euler.y * Mathf.Deg2Rad, euler.z * Mathf.Deg2Rad, text);

        string coords = targetPosition.x + " " + targetPosition.y + " " + targetPosition.z;

        string angles = string.Join(" ", Robot.solutionArray);
        aPIManager.SendControllerCoordinates(angles + ' ' + coords, text);

        Robot.SolveModelRobot(targetPosition.x, targetPosition.y, targetPosition.z, euler.x * Mathf.Deg2Rad, euler.y * Mathf.Deg2Rad, euler.z * Mathf.Deg2Rad, text);


        if (this.IsSafe(Robot.solutionArray) && Robot.solutionArray.Count(float.IsNaN) < 4)
        {
            for (int j = 0; j < 6; j++)
            {
                Vector3 currentRotation = jointList[j].transform.localEulerAngles;

                if (j == 0 || j == 4)
                {
                    currentRotation.y = Robot.solutionArray[j] * Mathf.Rad2Deg;
                }
                else
                {
                    currentRotation.z = Robot.solutionArray[j] * Mathf.Rad2Deg;
                }

                jointList[j].transform.localEulerAngles = currentRotation;
            }
        }
    }

    public bool IsSafe(float[] command)
    {
        bool safe = true;
        
        if (command[0] < -6.1 || command[0] > 6.1)
        {
            safe = false;
            Console.WriteLine("unsafe 0");
        }

        if (command[1] < -3.6 || command[1] > 0.5)
        {
            safe = false;
            Console.WriteLine("unsafe 1");
        }

        if (command[2] < -2.5 || command[2] > 2.5)
        {
            safe = false;
            Console.WriteLine("unsafe 2");
        }

        // Uncomment the following block if needed
        // if (command[3] < -3.1)
        // {
        //     if (command[4] < -0.87 || command[4] > 2.6)
        //     {
        //         safe = false;
        //         Console.WriteLine("unsafe 34 c1");
        //     }
        // }

        // Uncomment the following block if needed
        // if (command[3] > 0)
        // {
        //     if (command[4] < -2.6 || command[4] > 0.87)
        //     {
        //         safe = false;
        //         Console.WriteLine("unsafe 34 c2");
        //     }
        // }

        return safe;
    }
}