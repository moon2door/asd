using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LoginMenuControl : MonoBehaviour
{
    [Header("[스크립트 참조]")]
    public MySocket socketObject;
    public MenuController menuController;
    
    [Header("[게임오브젝트 참조]")]
    public UnityEngine.UI.InputField InputID;
    public UnityEngine.UI.InputField InputPassword;
    public UnityEngine.UI.Button btnOK;
    public UnityEngine.UI.Button btnCalcel;

    [Space(10)]
    public string adminId;
    public string adminPassword;

    private bool bUpdateView = false;

    void Awake()
    {
        btnOK.onClick.AddListener(delegate
        {
            OnClickButtonOk();
        });

        btnCalcel.onClick.AddListener(delegate
        {
            OnClickButtonClose();
        });
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (bUpdateView)
        {
            InputID.text = "";
            InputPassword.text = "";
            bUpdateView = false;
        }

        if (Input.GetKeyUp(KeyCode.Tab))
        {
            if (InputID.isFocused == true)
            {
                InputPassword.Select();
            }
            else if (InputPassword.isFocused == true)
            {
                InputID.Select();
            }
            else
            {
                InputID.Select();
            }
        }

        if (Input.GetKeyUp(KeyCode.KeypadEnter) ||
            Input.GetKeyUp(KeyCode.Return))
        {
            OnClickButtonOk();
        }
    }

    public void ResetInput()
    {
        bUpdateView = true;
    }

    private void OnClickButtonOk()
    {
        if (InputID.text.Length == 0 ||
            InputPassword.text.Length == 0)
        {
            // empty
        }
        else
        {
            if(adminId == InputID.text && adminPassword == InputPassword.text)
            {
                menuController.OnReplyLoginAdmin();
            }
            else
            {
                StructDefines.StRequestLogin message = new StructDefines.StRequestLogin();
                message.id = InputID.text;
                message.password = InputPassword.text;
                socketObject.SendRequestLogin(message);

                menuController.OnButtonLogin();
            }
            ResetInput();
        }
    }

    private void OnClickButtonClose()
    {
        menuController.OnButtonLogin();
    }
}
