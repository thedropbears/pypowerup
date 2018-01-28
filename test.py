import math


def turn_and_go_to_cube():
    """The robot rotates in the direction specified by the vision
    system while moving towards the cube. Combines two angles to find the absolute
    angle towards the cube"""

    angle = float(input("angle"))
    vision_angle = float(input("vision angle"))
    absolute_cube_direction = angle + vision_angle
    print(math.cos(math.radians(absolute_cube_direction)), math.sin(math.radians(absolute_cube_direction)), (math.radians(vision_angle)))


i = 0
while i < 10:
    i += 1
    turn_and_go_to_cube()
