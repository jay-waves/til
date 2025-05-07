#SingleInstance Force  ; 仍然有效，确保脚本只运行一个实例

; 设置工作目录为脚本所在目录（v2 语法）
SetWorkingDir A_ScriptDir

; 将 CapsLock 映射为 Esc
CapsLock::Esc

; 将 Esc 映射为 CapsLock
Esc::CapsLock

; Ctrl + Win + T 切换当前窗口的置顶状态
^#t:: {
    hwnd := WinExist("A")  ; 获取当前活动窗口句柄
    WinSetAlwaysOnTop(-1, hwnd)
}

; 音量控制
!c::Send "{Volume_Up}"    ; Alt + C -> 音量增加
!x::Send "{Volume_Down}"  ; Alt + X -> 音量减少
!z::Send "{Volume_Mute}"  ; Alt + Z -> 静音

; 方向键
!h::Send "{Left}"         ; Alt + H -> 左
!j::Send "{Down}"         ; Alt + J -> 下
!k::Send "{Up}"           ; Alt + K -> 上
!l::Send "{Right}"        ; Alt + L -> 右

; 导航键
!u::Send "{Home}"         ; Alt + U -> Home
!i::Send "{End}"          ; Alt + I -> End
!n::Send "{PgUp}"         ; Alt + N -> Page Up
!m::Send "{PgDn}"         ; Alt + M -> Page Down


; 是否使用中文标点
Global chinesePunctuationActive := false

^.:: {
    Global chinesePunctuationActive
    chinesePunctuationActive := !chinesePunctuationActive ;切换状态

    If chinesePunctuationActive {
        ToolTip("中文标点已启用")
    } Else {
        ToolTip("中文标点已禁用")
    }
    ; 2秒后自动隐藏提示信息
    SetTimer () => ToolTip(), -2000
}

; -------- 中文标点符号热键定义 -----------
#HotIf chinesePunctuationActive

,::SendInput "，"         
.::SendInput "。"        
;::SendInput "；"         
'::SendInput "‘"         
[::SendInput "【"          ; 左方括号
]::SendInput "】"          ; 右方括号
\::SendInput "、"          ; 顿号 
!::SendInput "！"         
?::SendInput "？"        
:::SendInput "："       
"::SendInput "“"       
(::SendInput "（"          
)::SendInput "）"         
~::SendInput "～"        
^::SendInput "……"       
_::SendInput "——"      
$::SendInput "￥"         
<::SendInput "《"       
>::SendInput "》"      

#HotIf
