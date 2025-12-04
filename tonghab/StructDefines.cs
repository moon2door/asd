using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Runtime.InteropServices;
using System;
using UnityEngine.UI;

namespace StructDefines
{
    public struct StCooperationMessage
    {
        public static bool operator >(StCooperationMessage op1, StCooperationMessage op2)
        {
            DateTime s1 = new DateTime(op1.startYear, op1.startMonth, op1.startDate, op1.startHour, op1.startMin, 0);
            DateTime e1 = new DateTime(op1.endYear, op1.endMonth, op1.endDate, op1.endHour, op1.endMin, 0);
            DateTime s2 = new DateTime(op2.startYear, op2.startMonth, op2.startDate, op2.startHour, op2.startMin, 0);
            DateTime e2 = new DateTime(op2.endYear, op2.endMonth, op2.endDate, op2.endHour, op2.endMin, 0);

            return (s1 > s2) || ((s1 == s2) && (e1 < e2));
        }
        public static bool operator <(StCooperationMessage op1, StCooperationMessage op2)
        {
            DateTime s1 = new DateTime(op1.startYear, op1.startMonth, op1.startDate, op1.startHour, op1.startMin, 0);
            DateTime e1 = new DateTime(op1.endYear, op1.endMonth, op1.endDate, op1.endHour, op1.endMin, 0);
            DateTime s2 = new DateTime(op2.startYear, op2.startMonth, op2.startDate, op2.startHour, op2.startMin, 0);
            DateTime e2 = new DateTime(op2.endYear, op2.endMonth, op2.endDate, op2.endHour, op2.endMin, 0);

            return (s1 < s2) || ((s1 == s2) && (e1 > e2));
        }

        public static bool operator >=(StCooperationMessage op1, StCooperationMessage op2)
        {
            DateTime s1 = new DateTime(op1.startYear, op1.startMonth, op1.startDate, op1.startHour, op1.startMin, 0);
            DateTime e1 = new DateTime(op1.endYear, op1.endMonth, op1.endDate, op1.endHour, op1.endMin, 0);
            DateTime s2 = new DateTime(op2.startYear, op2.startMonth, op2.startDate, op2.startHour, op2.startMin, 0);
            DateTime e2 = new DateTime(op2.endYear, op2.endMonth, op2.endDate, op2.endHour, op2.endMin, 0);

            return (s1 > s2) || ((s1 == s2) && (e1 <= e2));
        }
        public static bool operator <=(StCooperationMessage op1, StCooperationMessage op2)
        {
            DateTime s1 = new DateTime(op1.startYear, op1.startMonth, op1.startDate, op1.startHour, op1.startMin, 0);
            DateTime e1 = new DateTime(op1.endYear, op1.endMonth, op1.endDate, op1.endHour, op1.endMin, 0);
            DateTime s2 = new DateTime(op2.startYear, op2.startMonth, op2.startDate, op2.startHour, op2.startMin, 0);
            DateTime e2 = new DateTime(op2.endYear, op2.endMonth, op2.endDate, op2.endHour, op2.endMin, 0);

            return (s1 < s2) || ((s1 == s2) && (e1 >= e2));
        }

        public enum Cooperation
        {
            COOP_NONE = 0,
            COOP_REQUEST = 1,
            COOP_ON = 2,
        };

        public StCooperationMessage(uint pier, uint crane)
        {
            this.pier = pier;
            this.crane = crane;
            cooperationMode = 0;
            startYear = 0;
            startMonth = 0;
            startDate = 0;
            startHour = 0;
            startMin = 0;
            startSec = 0;
            endYear = 0;
            endMonth = 0;
            endDate = 0;
            endHour = 0;
            endMin = 0;
            endSec = 0;
            authority = 0;
            id = "";
            name = "";
            reserved = "";
        }

        public const uint messageId = 9;
        public uint pier;
        public uint crane;

        // 현재 협업 모드
        // 0 : 협업 미수행(제어중)
        // 1 : 협업 요청(제어중)
        // 2 : 협업중(제어 해제)
        public int cooperationMode;
        public int startYear;
        public int startMonth;
        public int startDate;
        public int startHour;
        public int startMin;
        public int startSec;
        public int endYear;
        public int endMonth;
        public int endDate;
        public int endHour;
        public int endMin;
        public int endSec;

        public uint authority;
        public string id;
        public string name;
        public string reserved;

        public const int byteSize = 164;

        public bool FromBytes(byte[] byteArray)
        {
            bool ret = false;
            if(byteArray.Length >= byteSize)
            {
                int pos = 4;
                pier            = BitConverter.ToUInt32(byteArray, pos); pos += 4;
                crane           = BitConverter.ToUInt32(byteArray, pos); pos += 4;
                cooperationMode = BitConverter.ToInt32(byteArray, pos); pos += 4;
                startYear       = BitConverter.ToInt32(byteArray, pos); pos += 4;
                startMonth      = BitConverter.ToInt32(byteArray, pos); pos += 4;
                startDate       = BitConverter.ToInt32(byteArray, pos); pos += 4;
                startHour       = BitConverter.ToInt32(byteArray, pos); pos += 4;
                startMin        = BitConverter.ToInt32(byteArray, pos); pos += 4;
                startSec        = BitConverter.ToInt32(byteArray, pos); pos += 4;
                endYear         = BitConverter.ToInt32(byteArray, pos); pos += 4;
                endMonth        = BitConverter.ToInt32(byteArray, pos); pos += 4;
                endDate         = BitConverter.ToInt32(byteArray, pos); pos += 4;
                endHour         = BitConverter.ToInt32(byteArray, pos); pos += 4;
                endMin          = BitConverter.ToInt32(byteArray, pos); pos += 4;
                endSec          = BitConverter.ToInt32(byteArray, pos); pos += 4;
                authority       = BitConverter.ToUInt32(byteArray, pos); pos += 4;
                id              = Encoding.UTF8.GetString(byteArray, pos, 32); pos += 32;
                name            = Encoding.UTF8.GetString(byteArray, pos, 32); pos += 32;
                reserved        = Encoding.UTF8.GetString(byteArray, pos, 32); pos += 32;
                ret = true;
            }
            return ret;
        }

        public int ToBytes(ref byte[] byteArray)
        {
            byte[] _messageId = BitConverter.GetBytes(messageId);
            byte[] _pier = BitConverter.GetBytes(pier);
            byte[] _crane = BitConverter.GetBytes(crane);

            byte[] _cooperationMode = BitConverter.GetBytes(cooperationMode);
            byte[] _startYear = BitConverter.GetBytes(startYear);
            byte[] _startMonth = BitConverter.GetBytes(startMonth);
            byte[] _startDate = BitConverter.GetBytes(startDate);
            byte[] _startHour = BitConverter.GetBytes(startHour);
            byte[] _startMin = BitConverter.GetBytes(startMin);
            byte[] _startSec = BitConverter.GetBytes(startSec);
            byte[] _endYear = BitConverter.GetBytes(endYear);
            byte[] _endMonth = BitConverter.GetBytes(endMonth);
            byte[] _endDate = BitConverter.GetBytes(endDate);
            byte[] _endHour = BitConverter.GetBytes(endHour);
            byte[] _endMin = BitConverter.GetBytes(endMin);
            byte[] _endSec = BitConverter.GetBytes(endSec);

            byte[] _id = Encoding.UTF8.GetBytes(id);
            byte[] _authority = BitConverter.GetBytes(authority);
            byte[] _name = Encoding.UTF8.GetBytes(name);
            byte[] _reserved = Encoding.UTF8.GetBytes(reserved);
            
            int pos = 0;
            Array.Clear(byteArray, 0, byteArray.Length);
            Array.Copy(_messageId, 0, byteArray, pos, _messageId.Length); pos += _messageId.Length;
            Array.Copy(_pier, 0, byteArray, pos, _pier.Length); pos += _pier.Length;
            Array.Copy(_crane, 0, byteArray, pos, _crane.Length); pos += _crane.Length;

            Array.Copy(_cooperationMode, 0, byteArray, pos, _cooperationMode.Length); pos += _cooperationMode.Length;
            Array.Copy(_startYear, 0, byteArray, pos, _startYear.Length); pos += _startYear.Length;
            Array.Copy(_startMonth, 0, byteArray, pos, _startMonth.Length); pos += _startMonth.Length;
            Array.Copy(_startDate, 0, byteArray, pos, _startDate.Length); pos += _startDate.Length;
            Array.Copy(_startHour, 0, byteArray, pos, _startHour.Length); pos += _startHour.Length;
            Array.Copy(_startMin, 0, byteArray, pos, _startMin.Length); pos += _startMin.Length;
            Array.Copy(_startSec, 0, byteArray, pos, _startSec.Length); pos += _startSec.Length;
            Array.Copy(_endYear, 0, byteArray, pos, _endYear.Length); pos += _endYear.Length;
            Array.Copy(_endMonth, 0, byteArray, pos, _endMonth.Length); pos += _endMonth.Length;
            Array.Copy(_endDate, 0, byteArray, pos, _endDate.Length); pos += _endDate.Length;
            Array.Copy(_endHour, 0, byteArray, pos, _endHour.Length); pos += _endHour.Length;
            Array.Copy(_endMin, 0, byteArray, pos, _endMin.Length); pos += _endMin.Length;
            Array.Copy(_endSec, 0, byteArray, pos, _endSec.Length); pos += _endSec.Length;

            Array.Copy(_authority, 0, byteArray, pos, _authority.Length); pos += _authority.Length;

            Array.Clear(byteArray, pos, 32);
            Array.Copy(_id, 0, byteArray, pos, _id.Length); pos += 32;

            Array.Clear(byteArray, pos, 32);
            Array.Copy(_name, 0, byteArray, pos, _name.Length); pos += 32;

            Array.Clear(byteArray, pos, 32);
            Array.Copy(_reserved, 0, byteArray, pos, _reserved.Length); pos += 32;

            return pos;
        }
    }

    public struct StRequestCooperationList
    {
        public enum ListCode
        {
            COOPLIST_REQUEST = 0,
            COOPLIST_ACCEPTED = 1,
        };

        public StRequestCooperationList(uint pier, uint crane, uint listCode, int year, int month, int date)
        {
            this.pier = pier;
            this.crane = crane;
            this.listCode = listCode;
            this.year = year;
            this.month = month;
            this.date = date;
        }

        public const uint messageId = 10;
        public uint pier;
        public uint crane;
        public uint listCode;
        public int year;
        public int month;
        public int date;

        public int ToBytes(ref byte[] byteArray)
        {
            byte[] _messageId = BitConverter.GetBytes(messageId);
            byte[] _pier = BitConverter.GetBytes(pier);
            byte[] _crane = BitConverter.GetBytes(crane);
            byte[] _listCode = BitConverter.GetBytes(listCode);
            byte[] _year = BitConverter.GetBytes(year);
            byte[] _month = BitConverter.GetBytes(month);
            byte[] _date = BitConverter.GetBytes(date);

            int pos = 0;
            Array.Clear(byteArray, 0, byteArray.Length);
            Array.Copy(_messageId, 0, byteArray, pos, _messageId.Length); pos += _messageId.Length;
            Array.Copy(_pier, 0, byteArray, pos, _pier.Length); pos += _pier.Length;
            Array.Copy(_crane, 0, byteArray, pos, _crane.Length); pos += _crane.Length;
            Array.Copy(_listCode, 0, byteArray, pos, _listCode.Length); pos += _listCode.Length;
            Array.Copy(_year, 0, byteArray, pos, _year.Length); pos += _year.Length;
            Array.Copy(_month, 0, byteArray, pos, _month.Length); pos += _month.Length;
            Array.Copy(_date, 0, byteArray, pos, _date.Length); pos += _date.Length;
            return pos;
        }
    }

    public struct StCooperationAccept
    {
        public enum AcceptCode
        {
            COOP_ACCEPT = 0,
            COOP_REFUSE = 1
        };

        public StCooperationAccept(uint pier, uint crane, uint index, uint listCode, string name)
        {
            this.pier = pier;
            this.crane = crane;
            this.index = index;
            this.listCode = listCode;
            this.name = name;
        }

        public const uint messageId = 11;
        public uint pier;
        public uint crane;
        public uint index;
        public uint listCode;
        public string name;

        public int ToBytes(ref byte[] byteArray)
        {
            byte[] _messageId = BitConverter.GetBytes(messageId);
            byte[] _pier = BitConverter.GetBytes(pier);
            byte[] _crane = BitConverter.GetBytes(crane);
            byte[] _index = BitConverter.GetBytes(index);
            byte[] _listCode = BitConverter.GetBytes(listCode);
            byte[] _name = Encoding.UTF8.GetBytes(name);

            int pos = 0;
            Array.Clear(byteArray, 0, byteArray.Length);
            Array.Copy(_messageId, 0, byteArray, pos, _messageId.Length); pos += _messageId.Length;
            Array.Copy(_pier, 0, byteArray, pos, _pier.Length); pos += _pier.Length;
            Array.Copy(_crane, 0, byteArray, pos, _crane.Length); pos += _crane.Length;
            Array.Copy(_index, 0, byteArray, pos, _index.Length); pos += _index.Length;
            Array.Copy(_listCode, 0, byteArray, pos, _listCode.Length); pos += _listCode.Length;
            Array.Copy(_name, 0, byteArray, pos, _name.Length); pos += 32;
            return pos;
        }
    }

    public struct StCooperationRemove
    {

        public StCooperationRemove(uint pier, uint crane, uint year, uint month, uint date, uint index, string name)
        {
            this.pier = pier;
            this.crane = crane;
            this.index = index;
            this.year = year;
            this.month = month;
            this.date = date;
            this.name = name;
        }

        public const uint messageId = 16;
        public uint pier;
        public uint crane;
        public uint year;
        public uint month;
        public uint date;
        public uint index;
        public string name;

        public int ToBytes(ref byte[] byteArray)
        {
            byte[] _messageId = BitConverter.GetBytes(messageId);
            byte[] _pier = BitConverter.GetBytes(pier);
            byte[] _crane = BitConverter.GetBytes(crane);
            byte[] _index = BitConverter.GetBytes(index);
            byte[] _year = BitConverter.GetBytes(year);
            byte[] _month = BitConverter.GetBytes(month);
            byte[] _date = BitConverter.GetBytes(date);
            byte[] _name = Encoding.UTF8.GetBytes(name);

            int pos = 0;
            Array.Clear(byteArray, 0, byteArray.Length);
            Array.Copy(_messageId, 0, byteArray, pos, _messageId.Length); pos += _messageId.Length;
            Array.Copy(_pier, 0, byteArray, pos, _pier.Length); pos += _pier.Length;
            Array.Copy(_crane, 0, byteArray, pos, _crane.Length); pos += _crane.Length;
            Array.Copy(_index, 0, byteArray, pos, _index.Length); pos += _index.Length;
            Array.Copy(_year, 0, byteArray, pos, _year.Length); pos += _year.Length;
            Array.Copy(_month, 0, byteArray, pos, _month.Length); pos += _month.Length;
            Array.Copy(_date, 0, byteArray, pos, _date.Length); pos += _date.Length;
            Array.Copy(_name, 0, byteArray, pos, _name.Length); pos += 32;
            return pos;
        }
    }

    public struct StPlcInfoMessage
    {
        public StPlcInfoMessage(uint pier, uint crane)
        {
            this.pier = pier;
            this.crane = crane;
            Hoist1DecelControl = 0;
            Hoist2DecelControl = 0;
            Hoist3DecelControl = 0;
            Trolley1DecelControl = 0;
            Trolley2DecelControl = 0;
            GantryDecelControl = 0;
            AuxHoistDecelControl = 0;
            SlewingDecelControl = 0;
            bUserEnableSwich = false;
            bGantryCommandFoward = false;
            bGantryCommandReverse = false;
            bTrolley1CommandRight = false;
            bTrolley1CommandLeft = false;
            bTrolley2CommandRight = false;
            bTrolley2CommandLeft = false;
            bHoist1CommandUp = false;
            bHoist1CommandDown = false;
            bHoist2CommandUp = false;
            bHoist2CommandDown = false;
            bHoist3CommandUp = false;
            bHoist3CommandDown = false;
            bAuxHoistCommandUp = false;
            bAuxHoistCommandDown = false;
            Trolley1Position = 0;
            Trolley2Position = 0;
            GoliathPosition1 = 0;
            GoliathPosition2 = 0;
            Hoist1Position = 0;
            Hoist2Position = 0;
            Hoist3Position = 0;
            Hoist1Weight = 0;
            Hoist2Weight = 0;
            Hoist3Weight = 0;
            AuxHoistWeight = 0;
            plcConnected = false;
            reserved = new int[20];
        }

        public const uint messageId = 13;
        public uint pier;
        public uint crane;

        public int Hoist1DecelControl;
        public int Hoist2DecelControl;
        public int Hoist3DecelControl;
        public int Trolley1DecelControl;
        public int Trolley2DecelControl;
        public int GantryDecelControl;
        public int AuxHoistDecelControl;
        public int SlewingDecelControl;

        // 조작 정보
        public bool bUserEnableSwich;
        public bool bGantryCommandFoward;
        public bool bGantryCommandReverse;
        public bool bTrolley1CommandRight;
        public bool bTrolley1CommandLeft;
        public bool bTrolley2CommandRight;
        public bool bTrolley2CommandLeft;
        public bool bHoist1CommandUp;
        public bool bHoist1CommandDown;
        public bool bHoist2CommandUp;
        public bool bHoist2CommandDown;
        public bool bHoist3CommandUp;
        public bool bHoist3CommandDown;
        public bool bAuxHoistCommandUp;
        public bool bAuxHoistCommandDown;

        // 상태 정보
        public int Trolley1Position; // mm
        public int Trolley2Position; // mm
        public int GoliathPosition1; // mm
        public int GoliathPosition2; // mm
        public int Hoist1Position; // mm
        public int Hoist2Position; // mm
        public int Hoist3Position; // mm
        public int Hoist1Weight; // 0.1ton
        public int Hoist2Weight; // 0.1ton
        public int Hoist3Weight; // 0.1ton
        public int AuxHoistWeight;
        public bool plcConnected;

        public int[] reserved;

        public const int byteSize = 172;

        public bool FromBytes(byte[] byteArray)
        {
            bool ret = false;
            if (byteArray.Length >= byteSize)
            {
                int pos = 0;
                Hoist1DecelControl = BitConverter.ToInt32(byteArray, pos); pos += 4;
                Hoist2DecelControl = BitConverter.ToInt32(byteArray, pos); pos += 4;
                Hoist3DecelControl = BitConverter.ToInt32(byteArray, pos); pos += 4;
                Trolley1DecelControl = BitConverter.ToInt32(byteArray, pos); pos += 4;
                Trolley2DecelControl = BitConverter.ToInt32(byteArray, pos); pos += 4;
                GantryDecelControl = BitConverter.ToInt32(byteArray, pos); pos += 4;
                AuxHoistDecelControl = BitConverter.ToInt32(byteArray, pos); pos += 4;
                SlewingDecelControl = BitConverter.ToInt32(byteArray, pos); pos += 4;
                
                bUserEnableSwich = (byteArray[pos] == 1); pos += 1;
                bGantryCommandFoward = (byteArray[pos] == 1); pos += 1;
                bGantryCommandReverse = (byteArray[pos] == 1); pos += 1;
                bTrolley1CommandRight = (byteArray[pos] == 1); pos += 1;
                bTrolley1CommandLeft = (byteArray[pos] == 1); pos += 1;
                bTrolley2CommandRight = (byteArray[pos] == 1); pos += 1;
                bTrolley2CommandLeft = (byteArray[pos] == 1); pos += 1;
                bHoist1CommandUp = (byteArray[pos] == 1); pos += 1;
                bHoist1CommandDown = (byteArray[pos] == 1); pos += 1;
                bHoist2CommandUp = (byteArray[pos] == 1); pos += 1;
                bHoist2CommandDown = (byteArray[pos] == 1); pos += 1;
                bHoist3CommandUp = (byteArray[pos] == 1); pos += 1;
                bHoist3CommandDown = (byteArray[pos] == 1); pos += 1;
                bAuxHoistCommandUp = (byteArray[pos] == 1); pos += 1;
                bAuxHoistCommandDown = (byteArray[pos] == 1); pos += 1;
                Trolley1Position = BitConverter.ToInt32(byteArray, pos); pos += 4;
                Trolley2Position = BitConverter.ToInt32(byteArray, pos); pos += 4;
                GoliathPosition1 = BitConverter.ToInt32(byteArray, pos); pos += 4;
                GoliathPosition2 = BitConverter.ToInt32(byteArray, pos); pos += 4;
                Hoist1Position = BitConverter.ToInt32(byteArray, pos); pos += 4;
                Hoist2Position = BitConverter.ToInt32(byteArray, pos); pos += 4;
                Hoist3Position = BitConverter.ToInt32(byteArray, pos); pos += 4;
                Hoist1Weight = BitConverter.ToInt32(byteArray, pos); pos += 4;
                Hoist2Weight = BitConverter.ToInt32(byteArray, pos); pos += 4;
                Hoist3Weight = BitConverter.ToInt32(byteArray, pos); pos += 4;
                AuxHoistWeight = BitConverter.ToInt32(byteArray, pos); pos += 4;
                plcConnected = (byteArray[pos] == 1); pos += 1;
                reserved = new int[20];
                Array.Copy(byteArray, pos, reserved, 0, 20);
                ret = true;
            }
            return ret;
        }
    }

    public struct StRequestAddUser
    {
        public enum Code
        {
            COOPLIST_ADD = 0,
            COOPLIST_REMOVE = 1,
        };

        public StRequestAddUser(uint code)
        {
            this.code = code;
            id = "";
            password = "";
            info = "";
            admin = false;
        }

        public const uint messageId = 14;
        public uint code;
        public string id;
        public string password;
        public string info;
        public bool admin;

        public int ToBytes(ref byte[] byteArray)
        {
            byte[] _messageId = BitConverter.GetBytes(messageId);
            byte[] _code = BitConverter.GetBytes(code);
            byte[] _id = Encoding.UTF8.GetBytes(id);
            byte[] _password = Encoding.UTF8.GetBytes(password);
            byte[] _info = Encoding.UTF8.GetBytes(info);
            
            int pos = 0;
            Array.Clear(byteArray, 0, byteArray.Length);
            Array.Copy(_messageId, 0, byteArray, pos, _messageId.Length); pos += _messageId.Length;
            Array.Copy(_code, 0, byteArray, pos, _code.Length); pos += _code.Length;
            Array.Copy(_id, 0, byteArray, pos, Math.Min(_id.Length, 32)); pos += 32;
            Array.Copy(_password, 0, byteArray, pos, Math.Min(_password.Length, 32)); pos += 32;
            Array.Copy(_info, 0, byteArray, pos, Math.Min(_info.Length, 31)); pos += 31;
            if (admin == true) byteArray[pos] = 1; pos += 1;

            return pos;
        }
    }

    public struct StRequestLogin
    {
        public const uint messageId = 15;
        public string id;
        public string password;

        public int ToBytes(ref byte[] byteArray)
        {
            byte[] _messageId = BitConverter.GetBytes(messageId);
            byte[] _id = Encoding.UTF8.GetBytes(id);
            byte[] _password = Encoding.UTF8.GetBytes(password);
            
            int pos = 0;
            Array.Clear(byteArray, 0, byteArray.Length);
            Array.Copy(_messageId, 0, byteArray, pos, _messageId.Length); pos += _messageId.Length;
            Array.Copy(_id, 0, byteArray, pos, Math.Min(_id.Length, 32)); pos += 32;
            Array.Copy(_password, 0, byteArray, pos, Math.Min(_password.Length, 32)); pos += 32;

            return pos;
        }
    }

    public struct StReplyLogin
    {
        public enum AcceptCode
        {
            LOGIN_ACCEPTED = 0,
            COOP_REFUSED = 1
        };

        public const uint messageId = 15;
        public int code;
        public const int byteSize = 36;
        public string id;

        public bool FromBytes(byte[] byteArray)
        {
            bool ret = false;
            if (byteArray.Length >= byteSize)
            {
                int pos = 0;
                code = BitConverter.ToInt32(byteArray, pos); pos += 4;
                id = Encoding.UTF8.GetString(byteArray, pos, 32); pos += 32;
                ret = true;
            }
            return ret;
        }
    }

    public struct StRequestLogPlay
    {
        public enum AcceptCode
        {
            LOGPLAY_OPEN = 0,
            LOGPLAY_START = 1,
            LOGPLAY_PAUSE = 2,
            LOGPLAY_READY = 3,
            LOGPLAY_CLOSE = 4
        };

        public const uint messageId = 17;
        public uint pier;
        public int code;
        public DateTime playDay;

        public int ToBytes(ref byte[] byteArray)
        {
            byte[] _messageId = BitConverter.GetBytes(messageId);
            byte[] _pier = BitConverter.GetBytes(pier);
            byte[] _code = BitConverter.GetBytes(code);
            byte[] _year = BitConverter.GetBytes(playDay.Year);
            byte[] _month = BitConverter.GetBytes(playDay.Month);
            byte[] _date = BitConverter.GetBytes(playDay.Day);
            byte[] _hour = BitConverter.GetBytes(playDay.Hour);
            byte[] _min = BitConverter.GetBytes(playDay.Minute);
            byte[] _sec = BitConverter.GetBytes(playDay.Second);

            int pos = 0;
            Array.Clear(byteArray, 0, byteArray.Length);
            Array.Copy(_messageId, 0, byteArray, pos, _messageId.Length); pos += _messageId.Length;
            Array.Copy(_pier, 0, byteArray, pos, _pier.Length); pos += _pier.Length;
            Array.Copy(_code, 0, byteArray, pos, _code.Length); pos += _code.Length;
            Array.Copy(_year, 0, byteArray, pos, _year.Length); pos += _year.Length;
            Array.Copy(_month, 0, byteArray, pos, _month.Length); pos += _month.Length;
            Array.Copy(_date, 0, byteArray, pos, _date.Length); pos += _date.Length;
            Array.Copy(_hour, 0, byteArray, pos, _hour.Length); pos += _hour.Length;
            Array.Copy(_min, 0, byteArray, pos, _min.Length); pos += _min.Length;
            Array.Copy(_sec, 0, byteArray, pos, _sec.Length); pos += _sec.Length;
            return pos;
        }
    }

    public struct StIndicateLogPlay
    {
        public const uint messageId = 18;
        public const int byteSize = 72;
        public DateTime startTime;
        public DateTime endTime;
        public DateTime curTime;

        public bool FromBytes(byte[] byteArray)
        {
            bool ret = false;
            if (byteArray.Length >= byteSize)
            {
                int pos = 0;
                int startYear = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int startMonth = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int startDay = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int startHour = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int startMin = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int startSec = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int endYear = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int endMonth = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int endDay = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int endHour = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int endMin = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int endSec = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int curYear = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int curMonth = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int curDay = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int curHour = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int curMin = BitConverter.ToInt32(byteArray, pos); pos += 4;
                int curSec = BitConverter.ToInt32(byteArray, pos); pos += 4;

                Debug.Log(string.Format("startTime {0}-{1}-{2} {3}:{4} {5}", startYear, startMonth, startDay, startHour, startMin, startSec));
                Debug.Log(string.Format("endTime {0}-{1}-{2} {3}:{4} {5}", endYear, endMonth, endDay, endHour, endMin, endSec));
                Debug.Log(string.Format("curTime {0}-{1}-{2} {3}:{4} {5}", curYear, curMonth, curDay, curHour, curMin, curSec));
                startTime = new DateTime(startYear, startMonth, startDay, startHour, startMin, startSec);
                endTime = new DateTime(endYear, endMonth, endDay, endHour, endMin, endSec);
                curTime = new DateTime(curYear, curMonth, curDay, curHour, curMin, curSec);

                ret = true;
            }
            return ret;
        }
    }
}
