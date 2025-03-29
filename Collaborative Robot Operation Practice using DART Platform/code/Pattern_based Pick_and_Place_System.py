# Forward initial coordinates
pos1 = posx(496.61, 95.33, 25.5, 89.51, -176.83, 115.82)  # Teaching pose1
pos2 = posx(598.63, 92.15, 24.23, 77.47, -175.72, 103.94) # Teaching pose2
pos3 = posx(598.29, -8.18, 27.25, 78.24, -175.67, 104.87) # Teaching pose3
pos4 = posx(496.56, -7.78, 24.67, 70.2, -175.15, 96.01) # Teaching pose4

# Reverse initial coordinates
pos5=posx(496.71, -55.81, 26.16, 71.92, -174.45, 97.47) # Teaching pose 5
pos6=posx(598.13, -58.27, 25.38, 79.06, -174.06, 104.64) # Teaching pose 6
pos7=posx(598.32, -158.55, 26.87, 81.12, -173.12, 106.34) # Teaching pose 7
pos8=posx(495.92, -156.93, 27, 80.63, -172.37, 105.53) # Teaching pose 8

# Functions
def grip():
    set_digital_output(1,ON)
    set_digital_output(2, OFF)

def ungrip():
    set_digital_output(1, OFF)
    set_digital_output(2, ON)

def initial():
    set_digital_output(1, OFF)
    set_digital_output(2, OFF)

def ini():
    set_tcp("GripperDA_v0203")
    set_tool("Tooltest")
    if get_digital_input(1) == 1:
        tp_popup("관리자 확인 필요 !",DR_PM_ALARM,1)
        exit()
    else:
        ungrip()

#ini()
# Set get_pattern
direction = 0 # Normal Pallet -> 0: Snake, 1: Zigzag / Rhombus Pallet -> 2: Snake, 3: Zigzag
row = 3
column = 3
thickness = 0
stack = 1
point_offset = [0, 0, 0] # Offset for calculated pose

if direction < 2:
    total_count = row * column * stack
else:
    total_count =  (row * column -int(row2)) * stack


while True:
    for pallet_index in range(pallet_index, total_count):
        pallet_Pose1 = get_pattern_point(pos1, pos2, pos3,pos4, pallet_index,direction,row,column,stack,thickness,point_offset)
        pallet_Pose2 = get_pattern_point(pos5, pos6, pos7,pos8, pallet_index,direction,row,column,stack,thickness,point_offset)
        #tp_popup("mode={},cnt={},pos1={},pos2={}".format(mode,pallet_index,pallet_Pose1,pallet_Pose2))
        
        if mode == 1: # 정방향
            app1 = trans(pallet_Pose1, [0,0,70,0,0,0],DR_BASE,DR_BASE)
            movel(app1,v=40,a=30)
            movel(pallet_Pose1,v=40,a=30)
            grip()
            movel(app1,v=40,a=30)
            #tp_popup("cnt={},pos1={}".format(pallet_index,pallet_Pose1))
            app2 = trans(pallet_Pose2,[0,0,70,0,0,0],DR_BASE,DR_BASE)
            movel(app2, v=40,a=30)
            movel(pallet_Pose2,v=40,a=30)
            ungrip()
            movel(app2,v=40,a=30)
            #tp_popup("cnt={},pos2={}".format(pallet_index,pallet_Pose2))
            initial()
            
        elif mode == 2: # 역방향
            app2 =trans(pallet_Pose2, [0,0,70,0,0,0],DR_BASE,DR_BASE)
            movel(app2,v=40,a=30)
            movel(pallet_Pose2,v=40,a=30)
            grip()
            movel(app2,v=40,a=30)
            #tp_popup("cnt={},pos1={}".format(pallet_index,pallet_Pose2))
            app1 = trans(pallet_Pose1,[0,0,70,0,0,0],DR_BASE,DR_BASE)
            movel(app1, v=40,a=30)
            movel(pallet_Pose1,v=40,a=30)
            ungrip()
            movel(app1,v=40,a=30)
            #tp_popup("cnt={},pos1={}".format(pallet_index,pallet_Pose1))
            initial()
        else:
            tp_popup("MODE_NG")
       
        if pallet_index + 1 == total_count:
            pallet_index=0
            if mode == 1:
                mode = 2
            elif mode ==2 :
                mode = 1   