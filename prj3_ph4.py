import numpy
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches


radius = 1
clearance = 1
crad = radius + clearance

def circle_center(point,crad):
	p_x = point[0]
	p_y = point[1]
	location = np.sqrt((p_x -0)**2+(p_y -0)**2)
	if location <= 100 + crad:
		return False
	else:
		return True
        

    
def circle_topright(point,crad):
	p_x = point[0]
	p_y = point[1]
	location = np.sqrt((p_x -200)**2+(p_y -300)**2)
	if location <= 100 + crad:
		return False
	else:
		return True

   
def circle_bottomright(point,crad):
	p_x = point[0]
	p_y = point[1]
	location = np.sqrt((p_x -200)**2+(p_y +300)**2)
	if location <= 100 + crad:
		return False
	else:
		return True

def circle_bottomleft(point,crad):
	p_x = point[0]
	p_y = point[1]
	location = np.sqrt((p_x +200)**2+(p_y +300)**2)
	if location <= 100 + crad:
		return False
	else:
		return True
    
def square_right(point, crad):
    
    p1 = [325, -75]
    p2 = [475, -75]
    p3 = [475, 75]
    p4 = [325, 75]

    p_x = point[0]
    p_y = point[1]

    line1 = (p_y + 1) - (p1[1] - crad)
    line2 = p_x - (p2[0] + crad)
    line3 = p_y - (p3[1] + crad)
    line4 = p_x - (p4[0] - crad)

    if line1 >= 0 and line3 <= 0:
        return False
    if line2 <= 0 and line4 >= 0:
        return False
    else:
        return True
    
def square_topleft(point, crad):
    
    p1 = [-275, 225]
    p2 = [-125, 225]
    p3 = [-125, 375]
    p4 = [-275, 375]

    p_x = point[0]
    p_y = point[1]

    line1 = (p_y + 1) - (p1[1] - crad)
    line2 = p_x - (p2[0] + crad)
    line3 = p_y - (p3[1] + crad)
    line4 = p_x - (p4[0] - crad)

    if line1 >= 0 and line3 <= 0:
        return False
    if line2 <= 0 and line4 >= 0:
        return False
    else:
        return True
    
def square_left(point, crad):
    
    p1 = [-475, -75]
    p2 = [-325, -75]
    p3 = [-325,75]
    p4 = [-475, 75]

    p_x = point[0]
    p_y = point[1]

    line1 = (p_y + 1) - (p1[1] - crad)
    line2 = p_x - (p2[0] + crad)
    line3 = p_y - (p3[1] + crad)
    line4 = p_x - (p4[0] - crad)

    if line1 >= 0 and line3 <= 0:
        return False
    if line2 <= 0 and line4 >= 0:
        return False
    else:
        return True
    
def border(point, crad):
    

    p1 = [-510, -510]
    p2 = [510, -510]
    p3 = [510, 510]
    p4 = [-510, 510]

    if point[0] <= p1[0] or point[0] >= p2[0]:
        return False
    elif point[1] <= p1[1] or point[1] >= p3[1]:
        return False
    else:
        return True
    
def coll_check(point,crad):
    circle1 = circle_center(point,crad)
    circle2 = circle_topright(point,crad)
    circle3 = circle_bottomright(point,crad)
    circle4 = circle_bottomleft(point,crad)
    square1 = square_right(point, crad)
    square2 = square_topleft(point, crad)
    square3 = square_left(point, crad)
    if circle1 == True and circle2 == True and circle3 == True and circle4 == True and square1 == True and square2 == True and square3 == True:
        return True
    else:
        return False
    
        

def plot_workspace(crad):
    
    fig, ax = plt.subplots()
    fig.set_size_inches(7,7, forward=True)

    # Plot the square right
    right_square = [(325, -75), (475, -75), (475, 75), (325, 75), (325, -75)]
    codes1 = [ 
        Path.MOVETO,
        Path.LINETO,
        Path.LINETO,
        Path.LINETO,
        Path.CLOSEPOLY,
    ]
    path1 = Path(right_square, codes1)
    patch1 = patches.PathPatch(path1, facecolor='red', lw=0)
    ax.add_patch(patch1)
 
    # Plot the left square 
    left_square = [(-475, -75), (-325, -75), (-325, 75), (-475, 75),(-475, -75)]
    codes2 = [
        Path.MOVETO,
        Path.LINETO,
        Path.LINETO,
        Path.LINETO,
        Path.CLOSEPOLY,
    ]
    path2 = Path(left_square, codes2)
    patch2 = patches.PathPatch(path2, facecolor='red', lw=0)
    ax.add_patch(patch2)

    # Plot the top left square    
    topleft_square = [(-275, 225), (-125, 225), (-125, 375), (-275, 375), (-275, 225)]
    codes3 = [
        Path.MOVETO,
        Path.LINETO,
        Path.LINETO,
        Path.LINETO,
        Path.CLOSEPOLY,
    ]
    path3 = Path(topleft_square, codes3)
    patch3 = patches.PathPatch(path3, facecolor='red', lw=0)
    ax.add_patch(patch3)
    
    ax.add_patch(patches.Circle((-200,-300), radius=100, color='red', lw=1))
    ax.add_patch(patches.Circle((0,0), radius=100, color='red', lw=1))
    ax.add_patch(patches.Circle((200,300), radius=100, color='red', lw=1))
    ax.add_patch(patches.Circle((200,-300), radius=100, color='red', lw=1))
    
    
    ax.set_xlim(-500, 500)
    ax.set_ylim(-500,500)
    
    return fig,ax

'''
    # Corner fix
    corner_cords1 = np.array([[-510, -510], [-500, -510], [-500, 510], [-510, 5100]], dtype=np.int32)
    corner_cords2 = np.array([[-510, -510], [510, -510], [510, -500], [-510, -500]], dtype=np.int32)
    corner_cords3 = np.array([[500, -510], [510, -510], [510, 510], [500, 510]], dtype=np.int32)
    corner_cords4 = np.array([[-510, 500], [510, 500], [510, 510], [-510, 510]], dtype=np.int32)
    cv2.fillConvexPoly(img, corner_cords1, 255)
    cv2.fillConvexPoly(img, corner_cords2, 255)
    cv2.fillConvexPoly(img, corner_cords3, 255)
    cv2.fillConvexPoly(img, corner_cords4, 255)
'''

fig,ax = plot_workspace(crad)
plt.draw()
plt.pause(1)




