using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using CsCore;
public class SelectionManager : MonoBehaviour
{
    public CraneInfo craneInfo;
    public DetailedMenuControl detailedMenuControl;
    public CraneColorControl colorControl;
    public Dropdown dropdownCrane;
    public Dropdown dropdownPier;
    public CameraHandler cameraHandler;
    public Dictionary<int, List<GameObject>> gameObjectList = new Dictionary<int, List<GameObject>>();
    public List<GameObject> ignoreIfActivated;
    public List<int> dropdownPierNum;

    public int currentPier = 0;
    public int currentCrane = 0;

    private bool selectionChanged = false;

    
    // Start is called before the first frame update
    void Start()
    {
        string currentStringPier = CsCore.Configuration.ReadConfigIni("Initial Pier", "Value");

        switch(currentStringPier)
        {
            case "J":
                currentPier = 1;
                break;

            case "K":
                currentPier = 2;
                break;

            case "HAN":
                currentPier = 3;
                break;
            case "6":
                currentPier = 4;
                break;
            case "G2":
                currentPier = 5;
                break;
            case "G3":
                currentPier = 6;
                break;
            case "G4":
                currentPier = 7;
                break;
            default:   
                break;
        }

        UpdateDropdownList();

        switch (currentPier)
        {
            case 1:
                dropdownPier.value = 3;
                break;
            case 2:
                dropdownPier.value = 0;
                break;
            case 3:
                dropdownPier.value = 1;
                break;
            case 4:
                dropdownPier.value = 2;
                break;
            case 5:
                dropdownPier.value = 4;
                break;
            case 6:
                dropdownPier.value = 5;
                break;
            case 7:
                dropdownPier.value = 6;
                break;
            case 8:
                dropdownPier.value = 7;
                break;
            default:
                break;
        }

        dropdownPier.onValueChanged.AddListener(delegate
        {
            DropdownPierChanged(dropdownPier);
        });

        dropdownCrane.onValueChanged.AddListener(delegate
        {
            DropdownCraneChanged(dropdownCrane);
        });

        // Initialize objects
        foreach (int key in craneInfo.keys)
        {
            GameObject craneObject = craneInfo.dictCraneGameObject[key];
            gameObjectList.Add(key, new List<GameObject>());
            gameObjectList[key].Add(craneObject);

            for (int iChild = 0; iChild < craneObject.transform.childCount; iChild++)
            {
                GameObject child = craneObject.transform.GetChild(iChild).gameObject;
                gameObjectList[key].Add(child);
            }
        }
    }

    public bool IsIgnoreKey()
    {
        bool bIgnoreKey = false;
        for (int i = 0; i < ignoreIfActivated.Count; i++)
        {
            if (ignoreIfActivated[i].activeSelf)
            {
                bIgnoreKey = true;
            }
        }
        if (SimpleFileBrowser.FileBrowser.IsOpen) bIgnoreKey = true;
        return bIgnoreKey;
    }
    
    // Update is called once per frame
    void Update()
    {
        // 입력 상태 갱신
        bool bIgnoreKey = IsIgnoreKey();

        if (!bIgnoreKey) UpdateKeyInput();
        UpdateClickGamerObject();

        // 선택 크레인 변경에 대한 처리
        if(selectionChanged)
        {
            cameraHandler.UpdateTargetCrane(currentPier, currentCrane);
            detailedMenuControl.UpdateCurrentCrane(currentPier, currentCrane);
            colorControl.SetSelectCrane(currentPier, currentCrane);

            // 드랍다운 값 갱신
            if (craneInfo.Contains(currentPier, currentCrane))
            {
                switch(currentPier)
                {
                    case 1:
                        dropdownPier.value = 3;
                        break;
                    case 2:
                        dropdownPier.value = 0;
                        break;
                    case 3:
                        dropdownPier.value = 1;
                        break;
                    case 4:
                        dropdownPier.value = 2;
                        break;
                    case 5:
                        dropdownPier.value = 4;
                        break;
                    case 6:
                        dropdownPier.value = 5;
                        break;
                    case 7:
                        dropdownPier.value = 6;
                        break;
                    case 8:
                        dropdownPier.value = 7;
                        break;
                    default:
                        break;
                }

            }
            if (craneInfo.Contains(currentPier, currentCrane))
            {

                dropdownCrane.value = currentCrane + 1;
            }
            else
            {
                dropdownCrane.value = 0;
            }
            selectionChanged = false;
        }
    }

    void UpdateDropdownList()
    {
        dropdownPier.ClearOptions();
        List<string> pierOptions = new List<string>();
        foreach (int key in craneInfo.keys)
        {
            string pierName = craneInfo.dictPierName[key];
            if (!pierOptions.Contains(pierName))
            {
                pierOptions.Add(pierName);
            }
        }
        dropdownPier.AddOptions(pierOptions);

        dropdownCrane.ClearOptions();
        List<string> craneOptions = new List<string>();
        craneOptions.Add("-");
        foreach (int key in craneInfo.keys)
        {
            int pier = craneInfo.dictPier[key];
            if (pier == currentPier)
            {
                craneOptions.Add(craneInfo.dictCraneName[key]);
            }
        }
        dropdownCrane.AddOptions(craneOptions);
    }

    public void ChangeCraneSelection(int craneIndex)
    {
        if(craneInfo.Contains(currentPier, craneIndex))
        {
            selectionChanged = true;
            currentCrane = craneIndex;
            UpdateDropdownList();
        }
    }

    public void ChangeCraneSelection(int pier, int crane)
    {
        if(currentPier != pier || currentCrane != crane)
        {
            selectionChanged = true;
            currentPier = pier;
            currentCrane = crane;
            UpdateDropdownList();
        }
    }

    // 드랍다운으로 크레인 선택
    void DropdownCraneChanged(Dropdown change)
    {
        selectionChanged = true;

        int crane = change.value - 1;
        int key = craneInfo.GetCraneKeycode(currentPier, crane);
        if(craneInfo.keys.Contains(key))
        {
            ChangeCraneSelection(currentPier, crane);
        }
        else
        {
            ChangeCraneSelection(currentPier, -1);
        }
    }

    // 드랍다운으로 야드 선택
    void DropdownPierChanged(Dropdown change)
    {
        selectionChanged = true;
        int pier = -1;
        if (0<dropdownPierNum.Count && change.value < dropdownPierNum.Count)
        {
            pier = dropdownPierNum[change.value];
        }

        if(craneInfo.cranePierCode.Contains(pier))
        {
            ChangeCraneSelection(pier, -1);
        }
    }

    // 키 입력으로 선택
    void UpdateKeyInput()
    {
        int number = 0;
        string input = Input.inputString;
        if (int.TryParse(input, out number))
        {
            if (craneInfo.Contains(currentPier, number - 1))
            {
                ChangeCraneSelection(currentPier, number - 1);
            }
        }
        else if(input == "`")
        {
            ChangeCraneSelection(currentPier, -1);
        }
        else
        {
        }
    }

    // 게임 오브젝트 클릭으로 선택
    void UpdateClickGamerObject()
    {
        if (Input.GetMouseButton(0))
        {
            if (!EventSystem.current.IsPointerOverGameObject())
            {
                Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
                RaycastHit hit;
                if (Physics.Raycast(ray, out hit))
                {
                    GameObject o = hit.transform.gameObject;

                    foreach (int key in craneInfo.keys)
                    {
                        int pier = craneInfo.dictPier[key];
                        int crane = craneInfo.dictCrane[key];
                        if (gameObjectList[key].Contains(o))
                        {
                            ChangeCraneSelection(pier, crane);
                            break;
                        }
                    }
                }
            }
        }
    }
}
