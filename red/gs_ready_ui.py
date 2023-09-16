#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import PySimpleGUI as sg
import os, threading, time

def cmd_step1():
    os.system("python step1_mapping.pyc")

def run_step1():
    t = threading.Thread(target=cmd_step1,name="cmd_step1")
    t.setDaemon(True)
    t.start()

def cmd_step2():
    os.system("python step2_savemap.pyc")

def run_step2():
    t = threading.Thread(target=cmd_step2,name="cmd_step2")
    t.setDaemon(True)
    t.start()

def cmd_exit_mapping():
    os.system("killall -9 cartographer* rviz python")

def run_exit_mapping():
    t = threading.Thread(target=cmd_exit_mapping,name="cmd_exit_mapping")
    t.setDaemon(True)
    t.start()

def cmd_step3():
    os.system("python step3_loadmap_relocalization.pyc")

def run_step3():
    t = threading.Thread(target=cmd_step3,name="cmd_step3")
    t.setDaemon(True)
    t.start()

def cmd_step4():
    os.system("python step4_save_goal_1.pyc") #start_area 1

def run_step4():
    t = threading.Thread(target=cmd_step4,name="run_step4")
    t.setDaemon(True)
    t.start()

def cmd_step5():
    os.system("python step5_save_goal_2.pyc") #shelf_myself 2

def run_step5():
    t = threading.Thread(target=cmd_step5,name="run_step5")
    t.setDaemon(True)
    t.start()

def cmd_step6():
    os.system("python step6_save_goal_3.pyc") #shelf_common 3

def run_step6():
    t = threading.Thread(target=cmd_step6,name="cmd_step6")
    t.setDaemon(True)
    t.start()

def cmd_step7():
    os.system("python step7_save_direct_1.pyc")

def run_step7():
    t = threading.Thread(target=cmd_step7,name="cmd_step7")
    t.setDaemon(True)
    t.start()

def cmd_step8():
    os.system("python step8_save_direct_2.pyc")

def run_step8():
    t = threading.Thread(target=cmd_step8,name="cmd_step8")
    t.setDaemon(True)
    t.start()

def cmd_step9():
    os.system("python step9_ready_forward_pose.pyc")
    os.system("python look_obj.pyc")

def run_step9():
    t = threading.Thread(target=cmd_step9,name="cmd_step9")
    t.setDaemon(True)
    t.start()

def cmd_step10():
    os.system("python save_desk_goal.pyc")

def run_step10():
    t = threading.Thread(target=cmd_step10,name="cmd_step10")
    t.setDaemon(True)
    t.start()

def cmd_step11():
    os.system("python goto_start_area.pyc")

def run_step11():
    t = threading.Thread(target=cmd_step11,name="cmd_step11")
    t.setDaemon(True)
    t.start()

def cmd_step12():
    os.system("python new_gsdemo_4.py")

def run_step12():
    t = threading.Thread(target=cmd_step12,name="cmd_step12")
    t.setDaemon(True)
    t.start()

layout = [
          [(sg.Text('2023 RObCOM 深圳史河国赛 DEMO', size=[40, 1]))],   

          [(sg.Text('建图准备工作示例:', size=[40, 1]))],
          [sg.Button('建图', button_color=(sg.YELLOWS[0], sg.BLUES[0])),
           sg.Button('保存地图', button_color=(sg.YELLOWS[0], sg.BLUES[0])),
           sg.Button('加载地图&重定位&导航', button_color=(sg.YELLOWS[0], sg.BLUES[0]))],

          [(sg.Text('记录目标位置准备工作示例:', size=[40, 1]))],
          [sg.Button('保存出发区位置', button_color=(sg.YELLOWS[0], sg.BLUES[0])),
           sg.Button('保存我方货架位置', button_color=(sg.YELLOWS[0], sg.BLUES[0])),
           sg.Button('保存公共货架位置', button_color=(sg.YELLOWS[0], sg.BLUES[0])),
           sg.Button('保存能源站位置', button_color=(sg.YELLOWS[0], sg.BLUES[0]))], 
          
          [(sg.Text('记录方向准备工作示例:', size=[40, 1]))],          
          [sg.Button('机械臂到达准备位置', button_color=(sg.YELLOWS[0], sg.BLUES[0])),
           sg.Button('保存垂直货架并可靠近的方向', button_color=(sg.YELLOWS[0], sg.BLUES[0])),
           sg.Button('保存平行货架并可前行抓取的方向', button_color=(sg.YELLOWS[0], sg.BLUES[0]))],

          [(sg.Text('使用demo:', size=[40, 1]))],
          [sg.Button('回到出发位置', button_color=(sg.YELLOWS[0], sg.BLUES[0])),
           sg.Button('国赛demo', button_color=(sg.YELLOWS[0], sg.BLUES[0]))], 
          [sg.Output(size=(80, 20))]
         ]

window = sg.Window('深圳史河 BEETLEROBOT', layout, default_element_size=(30, 2))

while True:
    event, value = window.read()
    if event == '建图':
        cmd_exit_mapping()
        run_step1()
        print("建图...")
    elif event == '保存地图':
        run_step2()
        print("保存地图...")
    elif event == '加载地图&重定位&导航':
        cmd_exit_mapping()
        run_step3()
        print("加载地图&重定位&导航...")
    elif event == '保存出发区位置':
        run_step4()
        print("保存出发区位置...")
    elif event == '保存我方货架位置':
        run_step5()
        print("保存我方货架位置...")
    elif event == '保存公共货架位置':
        run_step6()
        print("保存公共货架位置...")
    elif event == '保存能源站位置':
        run_step10()
        print("保存能源站位置...")
    elif event == '机械臂到达准备位置':
        run_step9()
        print("机械臂到达准备位置...")
    elif event == '保存垂直货架并可靠近的方向':
        run_step7()
        print("保存垂直货架并可靠近的方向...")
    elif event == '保存平行货架并可前行抓取的方向':
        run_step8()
        print("保存平行货架并可前行抓取的方向...")
    elif event == '回到出发位置':
        run_step11()
        print("回到出发位置...")
    elif event == '国赛demo':
        run_step12()
        print("国赛demo...")
    else:
        run_exit_mapping()
        break
window.close()