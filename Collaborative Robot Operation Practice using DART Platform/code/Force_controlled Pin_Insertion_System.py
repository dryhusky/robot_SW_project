#  System_target_start - 시작점 // System_target_arrival - 목표점


# Thread to measure external force
def th_force_applied():
	global force_applied 
	force_applied = get_tool_force() #로봇에 가하는 외력 측정
	wait(0.1)

# Displacement
delta_up = [0,0,60,0,0,0]
delta_rendv = [0,0,42,0,0,0]

# Waypoint Coordinates
start_above = trans(System_target_start,delta_up,DR_BASE,DR_BASE)
target_above = trans(System_target_arrival,delta_up,DR_BASE,DR_BASE)
target_approach = trans(System_target_arrival,delta_rendv,DR_BASE,DR_BASE)

# Functions
def grip():
	set_digital_output(1,ON)
	set_digital_output(2, OFF)

def release():
	set_digital_output(1, OFF)
	set_digital_output(2, ON)

def initial():
	set_digital_output(1, OFF)
	set_digital_output(2, OFF)

# Initial gripper setup
release()
initial()

# From start to move above target
movel(start_above,100,100)
movel(System_target_start,100,100)
grip()
movel(start_above,100,100)

movel(target_above,100,100)
movel(target_approach,80,80)

# Set compliance control
task_compliance_ctrl()
set_stiffnessx([1000,1000,1000,100,100,100], time=0.5) # Set stiffness

# Set force control
fd = [0, 0, -30, 0, 0, 0]
fctrl_dir= [0, 0, 1, 0, 0, 0]
set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL)

# Infinite loop thread for measuring external force
A = thread_run(th_force_applied,loop=True)


coin = 0
while(True):
	if(force_applied[2] >= 15): # External force more than 15 N
		move_periodic(amp=[0,0,5,0,0,20], period=1.0, atime=0.2, repeat=3, ref=DR_BASE) # 주기 운동
		coin += 1
		if (coin >5) :
			tp_popup("ERROR")
			break
		elif get_current_posx()[0][2] < 38:
			break
			
	elif(force_applied[2] < 15 and force_applied[2] > 5): # External force less than 15 N and more than 5 N
		move_periodic(amp=[0,0,3,0,0,10], period=1.0, atime=0.2, repeat=5, ref=DR_BASE) # 주기 운동
		coin += 1
		if (coin >5) :
			tp_popup("ERROR")
			break
		elif get_current_posx()[0][2] < 38:
			break
	

# Finishing setup
release_compliance_ctrl()
release()
movel(target_above,100,100)
movel(System_pin_home,100,100)



-----


# Example Data
pos1 = posx(496.83, 93.18, 22.45, 155.72, -179.44, -174.26) # Teaching pose1
pos2 = posx(598.47, 94.03, 26.8, 158.95, -179.3, -170.75) # Teaching pose2
pos3 = posx(497.48, -8.67, 25.76, 113.74, -176.82, 141.13) # Teaching pose3
pos4 = posx(597.54, -8.38, 26.98, 154.44, -179.2, -175.45) # Teaching pose4
direction = 0 # Normal Pallet -> 0: Snake, 1: Zigzag / Rhombus Pallet -> 2: Snake, 3: Zigzag
row = 3
column = 3
thickness = 0
stack = 1
point_offset = [5, 5, 0] # Offset for calculated pose
# Total count
if direction < 2: # Normal Pallet
    total_count = row * column * stack
else: # Rhombus Pallet
    total_count = (row * column - int(row/2)) * stack

    
 
# 시스템 전역 변수 생성
for i in range(1,10):
    start_pos = f"start_{}"
    set_fm(start_pos,0)
   
   
 
# Calculate Pallet Pose (Resulted in base coordinate)
for pallet_index in range(0, total_count):
    Pallet_Pose = get_pattern_point(pos1, pos2, pos3, pos4, pallet_index, direction, row, column, stack, thickness, point_offset)
    tp_popup("pallet_index={}, Pallet_Pose ={}".format(pallet_index,Pallet_Pose))
    

    
    
    
pos5=posx(600.59, -54.08, 27.58, 105.02, -176.56, 132.4)
pos6=posx(496.82, -55.46, 24, 104.18, -176.78, 131.56)
pos7=posx(495.6, -156.75, 25.56, 92.32, -176.76, 119.87)
pos8=posx(600.2, -156.54, 26.55, 71.59, -176.71, 99)