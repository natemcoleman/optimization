# Initial python script for optimization final project

from math import sqrt, pi, floor, ceil, acos
import matplotlib.pyplot as plt
from scipy.optimize import minimize, brute
from tqdm import tqdm

class point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __str__(self):
        return f"x:{self.x} y:{self.y}"

class line:
    # points = [point(0,0), point(1,1)]
    def __init__(self, firstInitialPointForLine, secondInitialPointForLine):
        self.points = []
        # self.points[0] = firstInitialPointForLine
        # self.points[1] = secondInitialPointForLine
        self.points.append(firstInitialPointForLine)
        self.points.append(secondInitialPointForLine)

    def __str__(self):
        return f"({self.points[0].x},{self.points[0].y}), ({self.points[1].x},{self.points[1].y})"


def distance(a,b):
    return sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

def is_between_points(a,b,c):
    return distance(a,c) + distance(c,b) == distance(a,b)
def is_on_line(line,point):
    return distance(line.points[0],point) + distance(point,line.points[1]) == distance(line.points[0],line.points[1])

def constraint1(guessPoint):
    trueStatements = [is_on_line(i, guessPoint) for i in polygonLines]
    return any(trueStatements)-1

con1 = {'type': 'eq', 'fun': constraint1}
cons = [con1]


point1 = point(0,0)
point2 = point(0,1)
point3 = point(1,1)
point4 = point(1,0)

line1 = line(point1, point2)
line2 = line(point2, point3)
line3 = line(point3, point4)
line4 = line(point1, point4)

polygonLines = [line1, line2, line3, line4]

point5 = point(0.5, 0.5)
point6 = point(0.5,0)

# print("point 5 is between:", is_between_points(point1, point2,point5), ". This should be False.")
# print("point 6 is between:", is_between_points(point1, point4,point6), ". This should be True.")
# print("Point 1 is on line 1:", is_on_line(line1, point1), ". This should be True.")
# print("Point 3 is on line 1:", is_on_line(line1, point3), ". This should be False.")
# print("Point 5 is on line 4:", is_on_line(line4, point5), ". This should be False.")
# print("Point 6 is on line 4:", is_on_line(line4, point6), ". This should be True.")

# print("Point 1 is on any line", constraint1(point1), ". This should be True.")
# print("Point 2 is on any line", constraint1(point2), ". This should be True.")
# print("Point 3 is on any line", constraint1(point3), ". This should be True.")
# print("Point 4 is on any line", constraint1(point4), ". This should be True.")
# print("Point 5 is on any line", constraint1(point5), ". This should be False.")
# print("Point 6 is on any line", constraint1(point6), ". This should be True.")

# print(line1)
# print(line2)
# print(line3)
# print(line4)
# print(point1)
# print(point2)
# print(point3)
# print(point4)
