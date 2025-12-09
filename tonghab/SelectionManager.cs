using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
public class SelectionManager : MonoBehaviour
{
    CraneInfo craneInfo;
    public DetailedMenuControl detailedMenuControl;
    public CraneColorControl colorControl;
    public Dropdown dropdownCrane;
    public Dropdown dropdownPier;
    public CameraHandler cameraHandler;
    public Dictionary<int, List<GameObject>> gameObjectList = new Dictionary<int, List<GameObject>>();
    public List<GameObject> ignoreIfActivated;
    public List<int> dropdownPierNum;
    public List<int> dropdownCraneNum=new List<int>();

    public int currentPier = 0;
    public int currentCrane = 0;

    private bool selectionChanged = false;

    private void Awake()
    {
        int[] checkPierType = new int[(int)PierUtility.PierType.MAX]; 
        for (int i = 0; i < checkPierType.Length; i++)
            checkPierType[i] = 1;

        for (int i = 0; i < dropdownPierNum.Count; i++)
        {
            if(checkPierType.Length > (int)dropdownPierNum[i])
            {
                checkPierType[(int)dropdownPierNum[i]] = 0;
            }
        }
        for (int i=0;i< (int)PierUtility.PierType.MAX; i++)
        {
            // 현재 7안벽은 통합관제에서 표시되고 있지 않음.
            if (i == (int)PierUtility.PierType.Pier_7)
                continue;
            if (checkPierType[i] == 0)
                continue;
            dropdownPierNum.Add(i);
        }
    }
    // Start is called before the first frame update
    void Start()
    {
        craneInfo = GetComponent<CraneInfo>();

        string currentStringPier = CsCore.Configuration.ReadConfigIni("Initial Pier", "Value");

        currentPier = PierUtility.PierNum(currentStringPier);

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
            case 9:
                dropdownPier.value = 8;
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

            if (craneInfo.Contains(currentPier, currentCrane))
            {
                int dropDownIndex = dropdownCraneNum.FindIndex(e => e == currentCrane);
                dropdownCrane.value = dropDownIndex + 1;
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
        dropdownCraneNum.Clear();
        List<string> craneOptions = new List<string>();
        craneOptions.Add("-");
        foreach (int key in craneInfo.keys)
        {
            int pier = craneInfo.dictPier[key];
            if (pier == currentPier)
            {
                craneOptions.Add(craneInfo.dictCraneName[key]);
                dropdownCraneNum.Add(craneInfo.dictCrane[key]);
            }
        }
        dropdownCrane.AddOptions(craneOptions);
    }

    public void ChangeCraneSelection(int craneIndex)
    {
        if (dropdownCraneNum.Count <= craneIndex)
            return;
        if(craneInfo.Contains(currentPier, dropdownCraneNum[craneIndex]))
        {
            selectionChanged = true;
            currentCrane = dropdownCraneNum[craneIndex];
            UpdateDropdownList();
        }
    }

//pjh
    public void ChangeCraneSelection(int pier, int crane, bool change=true)
    {
        if(currentPier != pier || currentCrane != crane)
        {
            selectionChanged = true;
            currentPier = pier;
            currentCrane = crane;
            if(change)UpdateDropdownList();
        }
    }
    //

    // 드랍다운으로 크레인 선택
    void DropdownCraneChanged(Dropdown change)
    {
        var value = change.value;//pjh
        selectionChanged = true;
        int crane = -1;
        if (change.value - 1 >= 0 && change.value - 1 < dropdownCraneNum.Count)
            crane = dropdownCraneNum[change.value-1];

        int key = craneInfo.GetCraneKeycode(currentPier, crane);
        if(craneInfo.keys.Contains(key))
        {
            ChangeCraneSelection(currentPier, crane, false); //pjh
        }
        else
        {
            ChangeCraneSelection(currentPier, -1, false); //pjh
        }
        var pierValue = dropdownPierNum.IndexOf(currentPier);
        dropdownPier.value = pierValue;//pjh
        dropdownPier.captionText.text = dropdownPier.options[pierValue].text;//pjh
        dropdownCrane.value = value;//pjh
        dropdownCrane.captionText.text = dropdownCrane.options[value].text;//pjh
    }

    // 드랍다운으로 야드 선택
    void DropdownPierChanged(Dropdown change)
    {
        var value = change.value;//pjh

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
        dropdownPier.value = value;//pjh
        
        dropdownPier.captionText.text = dropdownPier.options[value].text;//pjh
    }

    // 키 입력으로 선택
    void UpdateKeyInput()
    {
        int number = 0;
        string input = Input.inputString;
        if (int.TryParse(input, out number))
        {
            int index = number - 1;
            if (dropdownCraneNum.Count <= index)
                ChangeCraneSelection(currentPier, -1);

            if (craneInfo.Contains(currentPier, dropdownCraneNum[index]))
            {
                ChangeCraneSelection(currentPier, dropdownCraneNum[index]);
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
