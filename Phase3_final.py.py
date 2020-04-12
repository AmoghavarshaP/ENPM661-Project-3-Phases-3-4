import numpy
import numpy as np
import cv2
import math
import time as time
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

step=10
# radius = 1
# clearance = 1
# crad = radius + clearance

class Nodes:
    def __init__(self,state):
        self.state = state
        self.cost = math.inf
        self.parent = None
        self.LeftVel= None
        self.RightVel=None




def circle_bottomleft(position,crad):
    p_x = position[0]
    p_y = position[1]
    location = np.sqrt((p_x - 310) ** 2 + (p_y - 210) ** 2)
    if location <= 100 + crad:
        return False
    else:
        return True
def circle_center(position,crad):
    p_x = position[0]
    p_y = position[1]
    location = np.sqrt((p_x - 510) ** 2 + (p_y - 510) ** 2)
    if location <= 100 + crad:
        return False
    else:
        return True
def circle_topright(position,crad):
    p_x = position[0]
    p_y = position[1]
    location = np.sqrt((p_x - 710) ** 2 + (p_y - 810) ** 2)
    if location <= 100 + crad:
        return False
    else:
        return True

def circle_bottomright(position,crad):
    p_x = position[0]
    p_y = position[1]
    location = np.sqrt((p_x - 710) ** 2 + (p_y - 210) ** 2)
    if location <= 100 + crad:
        return False
    else:
        return True

def square_left(position,crad):
    p1 = [35,435]
    p2 = [185,435]
    p3 = [185,585]
    p4 = [35, 585]

    p_x = position[0]
    p_y = position[1]

    line1 = (p_y + 1) - (p1[1] - crad)
    line2 = p_x - (p2[0] + crad)
    line3 = p_y - (p3[1] + crad)
    line4 = p_x - (p4[0] - crad)

    if line1 >= 0 and line3 <= 0 and line2 <= 0 and line4 >= 0:
        return False
    else:
        return True


def square_right(position,crad):
    p1 = [835,435]
    p2 = [985,435]
    p3 = [985,585]
    p4 = [835,585]

    p_x = position[0]
    p_y = position[1]

    line1 = (p_y + 1) - (p1[1] - crad)
    line2 = p_x - (p2[0] + crad)
    line3 = p_y - (p3[1] + crad)
    line4 = p_x - (p4[0] - crad)

    if line1 >= 0 and line3 <= 0 and line2 <= 0 and line4 >= 0:
        return False
    else:
        return True
def square_topleft(position,crad):
    p1 = [235,735]
    p2 = [385,735]
    p3 = [385,885]
    p4 = [235,885]

    p_x = position[0]
    p_y = position[1]

    line1 = (p_y + 1) - (p1[1] - crad)
    line2 = p_x - (p2[0] + crad)
    line3 = p_y - (p3[1] + crad)
    line4 = p_x - (p4[0] - crad)

    if line1 >= 0 and line3 <= 0 and line2 <= 0 and line4 >= 0:
        return False
    else:
        return True

def collision_check(position,crad):
    if square_left(position,crad):
        return True
    elif square_topleft(position,crad):
        return True
    elif square_right(position,crad):
        return True
    elif circle_topright(position,crad):
        return True
    elif circle_bottomright(position,crad):
        return True
    elif circle_bottomleft(position,crad):
        return True
    elif circle_center(position,crad):
        return True
    else:
        return False

def check_squares(position,crad):
    square1 = square_left(position,crad)
    square2= square_topleft(position,crad)
    square3 = square_right(position,crad)
    if square1 == True and square2 == True and square3 == True:
        return True
    else:
        return False

def check_circles(position,crad):
    circle1 = circle_topright(position,crad)
    circle2 = circle_bottomright(position,crad)
    circle3 = circle_bottomleft(position,crad)
    circle4 = circle_center(position,crad)
    if circle1 == True and circle2 == True and circle3 == True and circle4 == True:
        return True
    else:
        return False

def check_viableX(point):
    if point >= 0 and point < 1020:
        return True
    else:
        return False

def check_viableY(point):
    if point > 0 and point < 1020:
        return True
    else:
        return False

def Action(position,UL,UR,crad):
        t = 0
        r = 0.038
        L = 0.354
        dt = 0.1
        Xn = position[0]
        Yn = position[1]
        UL = (UL/60)*r*2*np.pi*100
        UR = (UR/60)*r*2*np.pi*100
        thetai = position[2]%360
        if thetai < 0:
            thetai = thetai +360
        #print("orient:",orientation)
        # print(UL,UR,"lalal")
        Thetan = 3.14 * thetai/180
        cost = 0
        # Xi, Yi,Thetai: Input point's coordinates
        # Xs, Ys: Start point coordinates for plot function
        # Xn, Yn, Thetan: End point coordintes
        while t<1:
            t = t + dt
            Xs = Xn
            Ys = Yn
            # print(r,UL,UR)
            dx = (50*r * (UL + UR) * math.cos(Thetan) * dt)
            dy = (50*r * (UL + UR) * math.sin(Thetan) * dt)
            Xn = Xn+dx
            Yn = Yn+dy
            dTheta = (r / L) * (UR - UL) * dt
            # print(dTheta)
            Thetan=Thetan+dTheta
            cost = cost + math.sqrt(dx**2+dy**2)
            if not check_viableX(Xn) or not check_viableY(Yn) or not check_circles((Xn,Yn),crad) or not check_squares((Xn,Yn),crad):
            # if not check_viableX(Xn) or not check_viableY(Yn) or  collision_check((Xn,Yn),crad):
                return None, None, None, None
            ax.plot([Xs, Xn], [Ys, Yn],linewidth=0.02,color="green")
        Thetan = 180 *(Thetan)/3.14
        Thetan = Thetan %360
        if Thetan < 0:
            Thetan = Thetan + 360
        if Thetan == 360:
            Thetan = 0
        # print(Xn,Yn)
        if check_viableX(Xn) and check_viableY(Yn) and check_circles((Xn,Yn),crad) and check_squares((Xn,Yn),crad):
            updated_pos=[Xn,Yn,Thetan]
            # print(updated_pos)
            # print("lala")
            return updated_pos,cost,UL,UR
        else:
            return None,None,None,None

def mover(position,move,crad,RPM_L,RPM_R):
    if move == 'mover1':
        [updated_pos,cost,UL,UR] = Action(position,0,RPM_L,crad)
    elif move == 'mover2':
        [updated_pos,cost,UL,UR] = Action(position,RPM_L,0,crad)
    elif move =='mover3':
        [updated_pos,cost,UL,UR] = Action(position,RPM_L,RPM_L,crad)
    elif move == 'mover4':
        [updated_pos,cost,UL,UR] = Action(position,0,RPM_R,crad)
    elif move == 'mover5':
        [updated_pos,cost,UL,UR] = Action(position,RPM_R,0,crad)
    elif move == 'mover6':
        [updated_pos,cost,UL,UR] = Action(position,RPM_R,RPM_R,crad)
    elif move == 'mover7':
        [updated_pos,cost,UL,UR] = Action(position,RPM_L,RPM_R,crad)
    elif move == 'mover8':
        [updated_pos,cost,UL,UR] = Action(position,RPM_R,RPM_L,crad)
    return updated_pos,cost,UL,UR

def mappingofneighbors(current_position,crad):
    neighbors=['mover1','mover2','mover3','mover4','mover5','mover6','mover7','mover8']

    active_points=[]
    for move in neighbors:
        new_pos,cost,UL,UR = mover(current_position,move,crad,RPM_L,RPM_R)
        active_points.append((new_pos,cost,UL,UR))
    #active_points = [new_pos for new_pos in active_points if new_pos!= None]
    return active_points


def priority_pop(queue):  # Priority Queue, outputs the node with least cost attached to it
    min_a = 0
    for elemt in range(len(queue)):
        if queue[elemt].cost < queue[min_a].cost:
            min_a = elemt
    return queue.pop(min_a)

def find_node(point, queue):
    for elem in queue:
        if elem.state == point:
            return queue.index(elem)
        else:
            return None

def track_back(node):
    print("Tracking Back")
    p = []
    p.append(node.parent)
    parent = node.parent
    if parent is None:
        return p
    while parent is not None:
        p.append(parent)
        parent = parent.parent
    return p

def CostToGoal(point,goal):
    xp = point[0]
    yp = point[1]
    xg = goal[0]
    yg = goal[1]
    euclidean_distance= np.sqrt((xp-xg)**2 + (yp-yg)**2)
    return euclidean_distance

def AStar_Algo(start,goal,fig,ax,crad):
    visited = []
    s_node = Nodes(start)
    s_node.cost = 0
    Q = [s_node]
    #img[start[1], start[0]] = [255, 0, 0]
    #img[goal[1], goal[0]] = [0, 0, 255]
    while Q:
        current = priority_pop(Q)
        visited.append(current.state)
        neighbors = mappingofneighbors(current.state,crad)
        for n in neighbors:
            if n[0] is not None:
                new_node = Nodes(n[0])
                new_node.parent = current
                new_node.RightVel = n[3]
                new_node.LeftVel = n[2]
                # if n[0][0]==goal[0] and n[0][1]==goal[1]:
                if math.sqrt((n[0][0] - goal[0])**2+(n[0][1] - goal[1])**2) <= 30:
                    print("Goal Reached")
                    # print(new_node())
                    return new_node.parent, new_node,visited


                #To increase speed of simualtion, setting a threshold value to limit the nodes considered
                dist = math.inf
                for nodes in visited:
                    dist = min(dist, math.sqrt((n[0][0] - nodes[0])**2+(n[0][1] - nodes[1])**2))
                # print(dist)
                if dist > 6.5:
                #if n[0] not in visited:
                    new_node.cost = n[1]+new_node.parent.cost + CostToGoal(n[0],goal)
                    visited.append(new_node.state)
                    # print(visited)
                    Q.append(new_node)

                else:
                    node_id = find_node(new_node,Q)

                    if node_id:
                        t_node = queue[node_id]
                        if t_node.cost> n[1]+new_node.parent.cost + CostToGoal(n[0],goal):
                            t_node.cost= n[1]+new_node.parent.cost + CostToGoal(n[0],goal)
                            t_node.parent = current
            else:
                continue

    return None,None,None



def plot_workspace(start,goal,crad):

    fig, ax = plt.subplots()
    fig.set_size_inches(7,7, forward=True)
    start = plt.scatter(start[0],start[1], color="darkblue", s=50)
    goal = plt.scatter(goal[0],goal[1], color="purple", s=5)

    # Plot the square right
    right_square = [(835,435), (985,435), (985,585), (835,585), (835,435)]
    codes1 = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY,]
    path1 = Path(right_square, codes1)
    patch1 = patches.PathPatch(path1, facecolor='red', lw=0)
    ax.add_patch(patch1)
    # Plot the left square
    left_square = [(35, 435), (185, 435), (185, 585), (35, 585), (35, 435)]
    codes2 = [Path.MOVETO,Path.LINETO, Path.LINETO,Path.LINETO,Path.CLOSEPOLY,]
    path2 = Path(left_square, codes2)
    patch2 = patches.PathPatch(path2, facecolor='red', lw=0)
    ax.add_patch(patch2)
    # Plot the top left square
    topleft_square = [(235, 735), (385, 735), (385, 885), (235, 885), (235, 735)]
    codes3 = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY,]
    path3 = Path(topleft_square, codes3)
    patch3 = patches.PathPatch(path3, facecolor='red', lw=0)
    ax.add_patch(patch3)
    ax.add_patch(patches.Circle((710,810), radius=100, color='red', lw=1))
    ax.add_patch(patches.Circle((510,510), radius=100, color='red', lw=1))
    ax.add_patch(patches.Circle((710,210), radius=100, color='red', lw=1))
    ax.add_patch(patches.Circle((310,210), radius=100, color='red', lw=1))


    ax.set_xlim(0, 1020)
    ax.set_ylim(0, 1020)

    return fig, ax





radius = (0.354/2)*100
wheel_Diameter = 0.076
length = 0.354
clearance = 5 #10 cm clearance
crader= radius+clearance
print("Enter the start node coordinates")
xi=int(input("x =  "))
yi=int(input("y =  "))
oi=int(input("Orientation = "))
start=[xi,yi,oi]
print("Enter the goal node coordinates")
xg=int(input("x =  "))
yg=int(input("y =  "))
goal=[xg,yg]
RPM_L = int(input("Enter RPM_L: "))
RPM_R = int(input("Enter RPM_R: "))
# start=[110,210,30]
# goal=[950,810]
# RPM_L=50
# RPM_R=50



timer = time.time()
timer = time.time()
#To check if the goal and start are in the obstacle space and if they are viable points
if not check_circles(start,crader) or not check_squares(start,crader) or not check_circles(goal,crader) or not check_squares(goal,crader):
    print("Start or goal nodes lie in the obstacle space")
    exit()
elif not check_viableX(xi) or not check_viableY(yi):
     print("Start nodes beyond the workspace")
     exit()
elif not check_viableX(xg) or not check_viableY(yg):
    print("goal nodes beyond the workspace")
    exit()
else:
    print("WORK In Progress: Please Wait: :)")


fig,ax = plot_workspace(start,goal,crader)

parent, final_node,visited= AStar_Algo(start,goal,fig,ax,crader)
print("Time to solve: " + str(time.time()-timer) + " seconds")
print("Visualization will start in a moment")
print("The visualization may take some time-Please be patient")
plt.draw()
plt.pause(0.001)
if parent is not None:
    parent_list = track_back(parent)
    Data = [[par.LeftVel,par.RightVel] for par in parent_list]
    Data.reverse()
    print(Data)
    print(len(parent_list))
    xend = final_node.state[0]
    yend = final_node.state[1]
    first_parent = final_node.parent
    x = first_parent.state[0]
    y = first_parent.state[1]
    plt.arrow(x,y,xend-x,yend-y,length_includes_head=True,head_width=10, head_length=1,color="red")
    xend = x
    yend = y
    frame=0
    for parent in parent_list:
        frame +=1
        x = parent.state[0]
        y = parent.state[1]
        plt.arrow(x,y,xend-x,yend-y,length_includes_head=True,head_width=10, head_length=1,color="red")
        plt.draw()
        plt.pause(0.000000001)
        xend = x
        yend = y
print("Time for the entire program to run with visualization: " + str(time.time()-timer) + " seconds")

if parent is None:
    print("No path to goal point")
