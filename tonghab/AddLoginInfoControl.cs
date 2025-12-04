using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AddLoginInfoControl : MonoBehaviour
{
    [Header("[스크립트 참조]")]
    public MySocket socketObject;
    public MenuController menuController;

    [Header("[게임오브젝트 참조]")]
    public UnityEngine.UI.InputField InputID;
    public UnityEngine.UI.InputField InputPassword;
    public UnityEngine.UI.InputField InputInfo;
    public UnityEngine.UI.Toggle toggleAdmin;
    public UnityEngine.UI.Button btnOK;
    public UnityEngine.UI.Button btnCalcel;

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
            InputInfo.text = "";
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
                InputInfo.Select();
            }
            else if (InputInfo.isFocused == true)
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
        if( InputID.text.Length == 0 ||
            InputPassword.text.Length == 0 ||
            InputInfo.text.Length == 0)
        {
            // empty
        }
        else
        {
            StructDefines.StRequestAddUser message = new StructDefines.StRequestAddUser((uint)StructDefines.StRequestAddUser.Code.COOPLIST_ADD);
            message.id = InputID.text;
            message.password = InputPassword.text;
            message.info = InputInfo.text;
            message.admin = toggleAdmin.isOn;

            socketObject.SendRequestAddUser(message);

            menuController.OnButtonAddLoginInfo();

            ResetInput();
        }
    }

    private void OnClickButtonClose()
    {
        menuController.OnButtonAddLoginInfo();
    }


}
