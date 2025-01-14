from ctypes import sizeof #gọi các hàm từ các thư viện khác không thuộc về python
from tkinter import * # thư viện GUI tiêu chuẩn cho Python
import tkinter
from tkinter import font
from typing import Mapping, Sized #thư viện mảng kich thuoc
import numpy as np #thư viện toán học

from serial.serialutil import Timeout #thu vien serial, timeout: limit the max time for calling a function
import testDH_V2 #thu vien dong hoc
import serial # cong serial
import time # thoi gian
import threading #luồng thực thi riêng biệt
import matplotlib.pyplot as plt
import numpy as np


def action(): #ham chuyen doi 2 giao dien

    top.destroy() # tat man hinh dau
    GiaoDien.deiconify() #hien thi man hinh giao dien

GiaoDien = Tk() #tao man hinh giao dien
top=Toplevel()  # màn hình hiện thị đầu tiên
top.title("GIAO DIỆN ROBOT 3 DOF") #tên
top.geometry('1280x720') #kích  thước 
MC = PhotoImage(file=r"D:\University\HK1_2024-2025_Năm4\DARobot\Robot_Thay_Ba_GiaoDien\Giaodien.png") #hinh C:\\Users\\84382\\Pictures\\
Label(top,image=MC).place(x=0,y=0)  # vi tri
#MC=PhotoImage(file="VSP.png") # hình
Label(top,image=MC).grid() # vị trí
Button(top,text="GO CONTROL INTERFACE",font='Times 15 bold', bg='#6666ff',fg='white',command=lambda:action()).place(x=400,y=605) # nút nhấn fg mau vien bg mau nen

GiaoDien.withdraw() # de chi hien thi 1 giao dien TOP ban dau
ser = serial.Serial('COM3',9600)

x_axis = 15
y_axis = 30
y_lable_IK = 245
value1 = 0
value2 = 0
value3 = 0
x=0
L1 = 65
L2 = 170
L3 = 160


background_color = '#F5F5F5' # mã TLP RGB xem ở https://htlit.maytinhhtl.com/lam-web/bang-ma-mau-css-html-code-thet-ke-design.html

GiaoDien.geometry('1280x720')
GiaoDien.title("CONTROL INTERFACE")
GiaoDien.configure(bg=background_color)

#------------------------------- VIẾT HÀM CON ------------------------------------
def FK(x):
    txb_Px_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Py_FK.delete(0,END) # Xóa dữ liệu trước đó  
    txb_Pz_FK.delete(0,END) # Xóa dữ liệu trước đó
    value1 = txb_slider_theta1.get() # Lấy giá trị hiện tại của theta1 từ thanh trượt
    value2 = txb_slider_theta2.get()
    value3 = txb_slider_theta3.get()
    Px = testDH_V2.Forward_Kinematic(value1,value2,value3)[0]
    Py = testDH_V2.Forward_Kinematic(value1,value2,value3)[1]
    Pz = testDH_V2.Forward_Kinematic(value1,value2,value3)[2]
    txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới vao PX PY PZ
    txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
    txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới
    

def IK():
    txb_theta1_IK.delete(0,END)
    txb_theta2_IK.delete(0,END)
    txb_theta3_IK.delete(0,END)
    Px = float(txb_Px_IK.get())
    Py = float(txb_Py_IK.get())
    Pz = float(txb_Pz_IK.get())
    # theta = float(txb_Theta.get())
    theta1 = testDH_V2.Inverse_Kinematic(Px,Py,Pz)[0]
    theta2 = testDH_V2.Inverse_Kinematic(Px,Py,Pz)[1]
    theta3 = testDH_V2.Inverse_Kinematic(Px,Py,Pz)[2]
    txb_theta1_IK.insert(0,theta1)
    txb_theta2_IK.insert(0,theta2)
    txb_theta3_IK.insert(0,theta3)

    value1 = theta1
    value2 = theta2 
    value3 = theta3
  
    mang = f'A{value3} {value1} {value2}'  # Ví dụ, bạn có thể thay đổi giá trị này
    ser.write(str(mang).encode())  # Chuyển đổi dữ liệu số thành chuỗi và gửi đi
    print(mang, type(mang))
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 

def Reset_Slider():
    txb_slider_theta1.delete(0,END)
    txb_slider_theta2.delete(0,END)
    txb_slider_theta3.delete(0,END) 
    txb_slider_theta1.insert(0,value1)
    txb_slider_theta2.insert(0,value2)
    txb_slider_theta3.insert(0,value3)    
    slider_theta1.set('0') #đặt lại vị trí thanh slider tương ứng với giá trị lấy từ textbox
    slider_theta2.set('0')
    slider_theta3.set('0')

def Reset_lable_Slider():
    Reset_Slider()
    Px = testDH_V2.Forward_Kinematic(0,0,0)[0]
    Py = testDH_V2.Forward_Kinematic(0,0,0)[1]
    Pz = testDH_V2.Forward_Kinematic(0,0,0)[2] 

    txb_Px_FK.delete(0,END) # hiển thị dữ liệu mới
    txb_Py_FK.delete(0,END) # hiển thị dữ liệu mới
    txb_Pz_FK.delete(0,END) # hiển thị dữ liệu mới 
    txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới
    txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
    txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới    

def ReSet_btn():
    new_thread = threading.Thread(target=Reset_lable_Slider) # Thread là các hàm hay thủ tục chạy độc lập đối với chương trình chính
    new_thread.start()
    mang = f'F0A0B0C'
    print(mang,type(mang))
    ser.write(mang.encode())
    time.sleep(0.01)


def theta1_set_btn():

    txb_Px_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Py_FK.delete(0,END) # Xóa dữ liệu trước đó  
    txb_Pz_FK.delete(0,END) # Xóa dữ liệu trước đó
    # Lấy giá trị hiện tại của theta1 từ thanh trượt
    value1 = txb_slider_theta1.get() # Lấy giá trị hiện tại của theta1 từ thanh trượt
    value2 = txb_slider_theta2.get()
    value3 = txb_slider_theta3.get()

    Px = testDH_V2.Forward_Kinematic(value1,value2,value3)[0]
    Py = testDH_V2.Forward_Kinematic(value1,value2,value3)[1]
    Pz = testDH_V2.Forward_Kinematic(value1,value2,value3)[2]
    txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới vao PX PY PZ
    txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
    txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới
    
    mang = f'A{value3} {value1} {value2}'  # Ví dụ, bạn có thể thay đổi giá trị này
    ser.write(str(mang).encode())  # Chuyển đổi dữ liệu số thành chuỗi và gửi đi
    print(mang, type(mang))
    time.sleep(0.01)


def Set_point_set_btn():
    try:
        AX = float(txb_theta1_PP_1.get()) if txb_theta1_PP_1.get() else 0.0             
        AY = float(txb_theta1_PP_2.get()) if txb_theta1_PP_2.get() else 0.0
        AZ = float(txb_theta1_PP_3.get()) if txb_theta1_PP_3.get() else 0.0

        BX = float(txb_theta2_PP_1.get()) if txb_theta2_PP_1.get() else 0.0
        BY = float(txb_theta2_PP_2.get()) if txb_theta2_PP_2.get() else 0.0
        BZ = float(txb_theta2_PP_3.get()) if txb_theta2_PP_3.get() else 0.0

        CX = float(txb_theta3_PP_1.get()) if txb_theta3_PP_1.get() else 0.0
        CY = float(txb_theta3_PP_2.get()) if txb_theta3_PP_2.get() else 0.0
        CZ = float(txb_theta3_PP_3.get()) if txb_theta3_PP_3.get() else 0.0

        DX = float(txb_theta4_PP_1.get()) if txb_theta4_PP_1.get() else 0.0
        DY = float(txb_theta4_PP_2.get()) if txb_theta4_PP_2.get() else 0.0
        DZ = float(txb_theta4_PP_3.get()) if txb_theta4_PP_3.get() else 0.0

        EX = float(txb_theta5_PP_1.get()) if txb_theta5_PP_1.get() else 0.0
        EY = float(txb_theta5_PP_2.get()) if txb_theta5_PP_2.get() else 0.0
        EZ = float(txb_theta5_PP_3.get()) if txb_theta5_PP_3.get() else 0.0

        EndX = float(txb_theta1_END.get()) if txb_theta1_END.get() else 0.0
        EndY = float(txb_theta2_END.get()) if txb_theta2_END.get() else 0.0
        EndZ = float(txb_theta3_END.get()) if txb_theta3_END.get() else 0.0

        value1A = testDH_V2.Inverse_Kinematic(AX,AY,AZ)[0]
        value2A = testDH_V2.Inverse_Kinematic(AX,AY,AZ)[1]
        value2A = value2A *(-1)
        value3A = testDH_V2.Inverse_Kinematic(AX,AY,AZ)[2]


        value1B = testDH_V2.Inverse_Kinematic(BX,BY,BZ)[0]
        value2B = testDH_V2.Inverse_Kinematic(BX,BY,BZ)[1]
        value2B = value2B*(-1)
        value3B = testDH_V2.Inverse_Kinematic(BX,BY,BZ)[2]
       

        value1C = testDH_V2.Inverse_Kinematic(CX,CY,CZ)[0]
        value2C = testDH_V2.Inverse_Kinematic(CX,CY,CZ)[1]
        value2C = value2C*(-1)
        value3C = testDH_V2.Inverse_Kinematic(CX,CY,CZ)[2]


        value1D = testDH_V2.Inverse_Kinematic(DX,DY,DZ)[0]
        value2D = testDH_V2.Inverse_Kinematic(DX,DY,DZ)[1]
        value2D = value2D*(-1)
        value3D = testDH_V2.Inverse_Kinematic(DX,DY,DZ)[2]
       

        value1E = testDH_V2.Inverse_Kinematic(EX,EY,EZ)[0]
        value2E = testDH_V2.Inverse_Kinematic(EX,EY,EZ)[1]
        value2E = value2E*(-1)
        value3E = testDH_V2.Inverse_Kinematic(EX,EY,EZ)[2]
        

        value1END = testDH_V2.Inverse_Kinematic(EndX,EndY,EndZ)[0]
        value2END = testDH_V2.Inverse_Kinematic(EndX,EndY,EndZ)[1]
        value2END = value2END*(-1)
        value3END = testDH_V2.Inverse_Kinematic(EndX,EndY,EndZ)[2]

        mang1 = f'IK 1 {value3A} {value1A} {value2A} {value3B} {value1B} {value2B} {value3C} {value1C} {value2C} {value3D} {value1D} {value2D} {value3E} {value1E} {value2E} {value3END} {value1END} {value2END}'
        ser.write(str(mang1).encode())  # Gửi chuỗi qua Serial
        print(mang1, type(mang1))
        time.sleep(0.01)

    except ValueError as e:
        print(f"Lỗi: {e}. Vui lòng kiểm tra lại giá trị đầu vào.")


def Start_btn(): 
    mang = 'S'  # Ví dụ, bạn có thể thay đổi giá trị này
    ser.write(str(mang).encode())  # Chuyển đổi dữ liệu số thành chuỗi và gửi đi
    print(mang, type(mang))
    time.sleep(0.01)


def Stop_btn():
    mang = 'L'
    ser.write(mang.encode())
    time.sleep(0.01)

def hut_btn():
    mang = 'H'
    ser.write(str(mang).encode())
    print(mang,type(mang))
    time.sleep(0.01)
def tha_btn():
    mang = 'T'
    ser.write(str(mang).encode())
    print(mang,type(mang))
    time.sleep(0.01)

#--------------------------------------------------------------------------

lbl_tieude = Label(GiaoDien,text="CONTROL INTERFACE",font=("Arial",21,font.BOLD),bg=background_color)
lbl_FK = Label(GiaoDien,text="FORWARD KINEMATIC",fg="blue",font=("Arial",14,font.BOLD),bg=background_color)
lbl_PP = Label(GiaoDien,text="PROGRAM POINT",fg="blue",font=("Arial",14,font.BOLD),bg=background_color)
lbl_IK = Label(GiaoDien,text="INVERSE KINEMATIC",fg="blue",font=("Arial",14,font.BOLD),bg=background_color)
lbl_theta1_FK = Label(GiaoDien,text="Theta1",fg="black",font=("Arial",12,font.BOLD),bg=background_color)

lbl_theta1_IK = Label(GiaoDien,text="Theta1",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta2_IK = Label(GiaoDien,text="Theta2",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta3_IK = Label(GiaoDien,text="Theta3",fg="black",font=("Arial",12,font.BOLD),bg=background_color)

lbl_theta_PP_1 = Label(GiaoDien,text="X",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta_PP_2 = Label(GiaoDien,text="Y",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta_PP_3 = Label(GiaoDien,text="Z",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_LINK_PP_1 = Label(GiaoDien,text="A",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_LINK_PP_2 = Label(GiaoDien,text="B",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_LINK_PP_3 = Label(GiaoDien,text="C",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_LINK_PP_4 = Label(GiaoDien,text="D",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_LINK_PP_5 = Label(GiaoDien,text="E",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_LINK_PP_6 = Label(GiaoDien,text="Theta",fg="black",font=("Arial",12,font.BOLD),bg=background_color)

lbl_theta1_END = Label(GiaoDien,text="END_X",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta2_END = Label(GiaoDien,text="END_Y",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta3_END = Label(GiaoDien,text="END_Z",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta_END = Label(GiaoDien,text="END_Theta",fg="black",font=("Arial",12,font.BOLD),bg=background_color)


lbl_Px_IK = Label(GiaoDien,text="Px",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Py_IK = Label(GiaoDien,text="Py",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Pz_IK = Label(GiaoDien,text="Pz",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Px_FK = Label(GiaoDien,text="Px",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Py_FK = Label(GiaoDien,text="Py",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_Pz_FK = Label(GiaoDien,text="Pz",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_SETPOINT = Label(GiaoDien,text="SET POINT",fg="black",font=("Arial",12,font.BOLD),bg=background_color)


txb_slider_theta1 = Entry(GiaoDien,width=6,font=("Arial",12,font.BOLD)) #tao o hien thi goc ben canh slider
txb_slider_theta1.insert(0,value1)

txb_slider_theta2 = Entry(GiaoDien,width=6,font=("Arial",12,font.BOLD))
txb_slider_theta2.insert(0,value2)

txb_slider_theta3 = Entry(GiaoDien,width=6,font=("Arial",12,font.BOLD))
txb_slider_theta3.insert(0,value3)

txb_Px_IK = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD)) # tao o hien thi Px Invert
txb_Py_IK = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_Pz_IK = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_Px_FK = Entry(GiaoDien,width=7,font=("Arial",12,font.BOLD))
txb_Py_FK = Entry(GiaoDien,width=7,font=("Arial",12,font.BOLD))
txb_Pz_FK = Entry(GiaoDien,width=7,font=("Arial",12,font.BOLD))
txb_theta1_IK = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD)) #tao o hien thi goc invert
txb_theta2_IK = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta3_IK = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))

txb_theta1_PP_1 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD)) 
txb_theta2_PP_1 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta3_PP_1 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta4_PP_1 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta5_PP_1 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta1_PP_2 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD)) 
txb_theta2_PP_2 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta3_PP_2 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta4_PP_2 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta5_PP_2 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta1_PP_3 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD)) 
txb_theta2_PP_3 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta3_PP_3 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta4_PP_3 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta5_PP_3 = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))


txb_theta1_END = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta2_END = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))
txb_theta3_END = Entry(GiaoDien,width=10,font=("Arial",12,font.BOLD))

btn_Start = Button(GiaoDien,text="Set Home",font=("Arial",12,font.BOLD),width=10,height=2,bg='#FF1493',command=Start_btn)
btn_Stop = Button(GiaoDien,text="Stop",font=("Arial",12,font.BOLD),width=10,height=2,bg='#FF0000',command=Stop_btn)
btn_Solve = Button(GiaoDien,text="Solve",font=("Arial",12,font.BOLD),width=10,height=2,bg='#98FB98',command=IK)
btn_ReSet = Button(GiaoDien,text="Reset",font=("Arial",12,font.BOLD),width=10,height=2,bg='#98FB98',command=ReSet_btn)
btn_Set_Theta1 = Button(GiaoDien,text="Set Theta",font=("Arial",10,font.BOLD),width=8,height=2,bg='#98FB98',command=theta1_set_btn)
btn_SETPOINT = Button(GiaoDien,text="SET POINT",font=("Arial",10,font.BOLD),width=8,height=2,bg='#98FB98',command=Set_point_set_btn)


btn_hut = Button(GiaoDien,text="Hut",font=("Arial",10,font.BOLD),width=8,height=2,bg='#98FB98',command=hut_btn)
btn_tha = Button(GiaoDien,text="Tha",font=("Arial",10,font.BOLD),width=8,height=2,bg='#98FB98',command=tha_btn)

lbl_FK.place(x=x_axis,y=y_axis)
lbl_PP.place(x=750,y=y_axis)
lbl_IK.place(x=x_axis,y=y_axis+y_lable_IK)


lbl_theta1_IK.place(x=200,y=y_axis+y_lable_IK+53)
lbl_theta2_IK.place(x=200,y=y_axis+y_lable_IK+103)
lbl_theta3_IK.place(x=200,y=y_axis+y_lable_IK+153)

lbl_theta_PP_1.place(x=840,y=y_axis+25)
lbl_theta_PP_2.place(x=990,y=y_axis+25)
lbl_theta_PP_3.place(x=1140,y=y_axis+25)
lbl_LINK_PP_1.place(x=775,y=y_axis+55)
lbl_LINK_PP_2.place(x=775,y=y_axis+105)
lbl_LINK_PP_3.place(x=775,y=y_axis+155)
lbl_LINK_PP_4.place(x=775,y=y_axis+205)
lbl_LINK_PP_5.place(x=775,y=y_axis+255)

lbl_theta1_END.place(x=810,y=y_axis+300)
lbl_theta2_END.place(x=960,y=y_axis+300)
lbl_theta3_END.place(x=1110,y=y_axis+300)

lbl_Px_FK.place(x=x_axis+180,y=y_axis+38)
lbl_Py_FK.place(x=x_axis+180,y=y_axis+83)
lbl_Pz_FK.place(x=x_axis+180,y=y_axis+128)
lbl_Px_IK.place(x=x_axis+20,y=y_axis+y_lable_IK+50)
lbl_Py_IK.place(x=x_axis+20,y=y_axis+y_lable_IK+100)
lbl_Pz_IK.place(x=x_axis+20,y=y_axis+y_lable_IK+150)

txb_slider_theta1.place(x=x_axis+20,y=y_axis+40)
txb_slider_theta2.place(x=x_axis+20,y=y_axis+85)
txb_slider_theta3.place(x=x_axis+20,y=y_axis+130)
txb_Px_FK.place(x=x_axis+220,y=y_axis+40)
txb_Py_FK.place(x=x_axis+220,y=y_axis+85)
txb_Pz_FK.place(x=x_axis+220,y=y_axis+130)
txb_Px_IK.place(x=x_axis+50,y=y_axis+y_lable_IK+53)
txb_Py_IK.place(x=x_axis+50,y=y_axis+y_lable_IK+103)
txb_Pz_IK.place(x=x_axis+50,y=y_axis+y_lable_IK+153)
txb_theta1_IK.place(x=270,y=y_axis+y_lable_IK+53)
txb_theta2_IK.place(x=270,y=y_axis+y_lable_IK+103)
txb_theta3_IK.place(x=270,y=y_axis+y_lable_IK+153)

txb_theta1_PP_1.place(x=800,y=y_axis+53)
txb_theta2_PP_1.place(x=800,y=y_axis+103)
txb_theta3_PP_1.place(x=800,y=y_axis+153)
txb_theta4_PP_1.place(x=800,y=y_axis+203)
txb_theta5_PP_1.place(x=800,y=y_axis+253)
txb_theta1_PP_2.place(x=950,y=y_axis+53)
txb_theta2_PP_2.place(x=950,y=y_axis+103)
txb_theta3_PP_2.place(x=950,y=y_axis+153)
txb_theta4_PP_2.place(x=950,y=y_axis+203)
txb_theta5_PP_2.place(x=950,y=y_axis+253)
txb_theta1_PP_3.place(x=1100,y=y_axis+53)
txb_theta2_PP_3.place(x=1100,y=y_axis+103)
txb_theta3_PP_3.place(x=1100,y=y_axis+153)
txb_theta4_PP_3.place(x=1100,y=y_axis+203)
txb_theta5_PP_3.place(x=1100,y=y_axis+253)


txb_theta1_END.place(x=800,y=y_axis+330) # 100
txb_theta2_END.place(x=950,y=y_axis+330) 
txb_theta3_END.place(x=1100,y=y_axis+330)

btn_Start.place(x=x_axis+460,y=95)
btn_Solve.place(x=x_axis+370,y=y_axis+y_lable_IK+85)
btn_Set_Theta1.place(x=x_axis+300,y=y_axis+70)
btn_SETPOINT.place(x=x_axis+940,y=y_axis+370)
btn_hut.place(x=270,y=y_axis+y_lable_IK+203)
btn_tha.place(x=x_axis+370,y=y_axis+y_lable_IK+203)

#------------------------------------ TRẠNG THÁI BAN ĐẦU --------------------------------------
Px = testDH_V2.Forward_Kinematic(0,0,0)[0]
Py = testDH_V2.Forward_Kinematic(0,0,0)[1]
Pz = testDH_V2.Forward_Kinematic(0,0,0)[2]
txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới
txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới
 
GiaoDien.mainloop() # vong lap