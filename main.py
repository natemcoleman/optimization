# Initial python script for optimization final project

import math
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import numpy as np

sideLength = 1

def plotCurrentGrid(squareGridOrder):
    plt.axes()
    circleCenter = (sideLength*0.5*squareGridOrder, sideLength*0.5*squareGridOrder)
    circleRadius = sideLength*0.5*squareGridOrder
    squareValidColor = (199/255, 201/255, 199/255, 1)
    squareValidBorderColor = (0/255, 46/255, 96/255, 1)
    squareInvalidColor = (209/255, 65/255, 36/255, 1)
    circleColor = (158/255, 42/255, 43/255, 0.5)

    for i in range(squareGridOrder):
        for j in range(squareGridOrder):

            #find if square is within circle
            


            rectangle = plt.Rectangle((i*sideLength, j*sideLength), sideLength, sideLength, fc=squareValidColor, ec=squareValidBorderColor)
            plt.gca().add_patch(rectangle)
            plt.axis('scaled')

    circle = plt.Circle(circleCenter, circleRadius, color=circleColor)
    plt.gca().add_patch(circle)
    plt.axis('scaled')
    plt.axis('off')
    plt.show()

# for k in range(3):
#     plotCurrentGrid(k+1)

plotCurrentGrid(7)


# plt.show()
