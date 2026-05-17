#SingleInstance Force  ; 仍然有效，确保脚本只运行一个实例

SendMode "Event" 
SetKeyDelay -1, -1

; 设置工作目录为脚本所在目录（v2 语法）
SetWorkingDir A_ScriptDir

; Alt + T 切换当前窗口的置顶状态
!t:: {
    hwnd := WinExist("A")  ; 获取当前活动窗口句柄
    WinSetAlwaysOnTop(-1, hwnd)
}

; 音量控制 Alt 是功能键
!c::Send "{Volume_Up}"    ; Alt + C -> 音量增加
!x::Send "{Volume_Down}"  ; Alt + X -> 音量减少
!z::Send "{Volume_Mute}"  ; Alt + Z -> 静音

SetCapsLockState "AlwaysOff"

global VisualMode := false

; 单击 Caps -> Esc
CapsLock::
{
    global VisualMode

    VisualMode := false

    KeyWait "CapsLock"

    ; 如果没有组合键触发，则发送 Esc
    if (A_PriorKey = "CapsLock")
        Send "{Esc}"
}

; 松开 Caps 自动退出 Visual
CapsLock Up::
{
    global VisualMode
    VisualMode := false
}

#HotIf GetKeyState("CapsLock", "P")

; movement
*h::Send VisualMode ? "+{Left}"  : "{Left}"
*j::Send VisualMode ? "+{Down}"  : "{Down}"
*k::Send VisualMode ? "+{Up}"    : "{Up}"
*l::Send VisualMode ? "+{Right}" : "{Right}"

; word movement
*w::Send VisualMode ? "^+{Right}" : "^{Right}"
*b::Send VisualMode ? "^+{Left}"  : "^{Left}"

; line
*u::Send VisualMode ? "+{Home}" : "{Home}"
*i::Send VisualMode ? "+{End}"  : "{End}"

; page
*n::Send VisualMode ? "+{PgUp}" : "{PgUp}"
*m::Send VisualMode ? "+{PgDn}" : "{PgDn}"

; visual mode
*v::
{
    global VisualMode
    VisualMode := true
}

; copy / cut / paste
*y::
{
    global VisualMode
    Send "^c"
    VisualMode := false
}

*x::
{
    global VisualMode
    Send "^x"
    VisualMode := false
}

*p::Send "^v"

#HotIf

; 是否使用中文标点
; Global chinesePunctuationActive := false
; 
; !.:: {
;     Global chinesePunctuationActive
;     chinesePunctuationActive := !chinesePunctuationActive ;切换状态
; 
;     If chinesePunctuationActive {
;         ToolTip("中文标点已启用")
;     } Else {
;         ToolTip("中文标点已禁用")
;     }
;     ; 2秒后自动隐藏提示信息
;     SetTimer () => ToolTip(), -2000
; }
; 
; ; -------- 中文标点符号热键定义 -----------
; #HotIf chinesePunctuationActive
; 
; ,::SendInput "，"         
; .::SendInput "。"        
; ;::SendInput "；"         
; '::SendInput "‘’"         
; [::SendInput "【"          ; 左方括号
; ]::SendInput "】"          ; 右方括号
; \::SendInput "、"          ; 顿号 
; !::SendInput "！"         
; ?::SendInput "？"        
; :::SendInput "："       
; `;::SendInput "；"
; "::SendInput "“”"       
; (::SendInput "（"          
; )::SendInput "）"         
; ~::SendInput "～"        
; ^::SendInput "……"       
; _::SendInput "——"      
; $::SendInput "￥"         
; <::SendInput "《"       
; >::SendInput "》"      
; 
; #HotIf

; VirtualDesktopAccessor
; 必须使用 64b 编译.
VDA_PATH := "D:\bin\VirtualDesktopAccessor.dll"
hVirtualDesktopAccessor := DllCall("LoadLibrary", "Str", VDA_PATH, "Ptr")

GoToDesktopNumberProc := DllCall("GetProcAddress", "Ptr", hVirtualDesktopAccessor, "AStr", "GoToDesktopNumber", "Ptr")
GetCurrentDesktopNumberProc := DllCall("GetProcAddress", "Ptr", hVirtualDesktopAccessor, "AStr", "GetCurrentDesktopNumber", "Ptr")
MoveWindowToDesktopNumberProc := DllCall("GetProcAddress", "Ptr", hVirtualDesktopAccessor, "AStr", "MoveWindowToDesktopNumber", "Ptr")

MoveCurrentWindowToDesktop(number) {
    global MoveWindowToDesktopNumberProc, GoToDesktopNumberProc
    activeHwnd := WinGetID("A")
    DllCall(MoveWindowToDesktopNumberProc, "Ptr", activeHwnd, "Int", number, "Int")
    DllCall(GoToDesktopNumberProc, "Int", number, "Int")
}

GoToDesktopNumber(num) {
    global GoToDesktopNumberProc
    DllCall(GoToDesktopNumberProc, "Int", num, "Int")
    return
}
MoveOrGotoDesktopNumber(num) {
    ; If user is holding down Mouse left button, move the current window also
    if (GetKeyState("LButton")) {
        MoveCurrentWindowToDesktop(num)
    } else {
        GoToDesktopNumber(num)
    }
    return
}

!1::MoveOrGotoDesktopNumber(0)
!2::MoveOrGotoDesktopNumber(1)
!3::MoveOrGotoDesktopNumber(2)
!4::MoveOrGotoDesktopNumber(3)
