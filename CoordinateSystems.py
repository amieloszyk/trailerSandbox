from __future__ import annotations

from typing import List
import math

import numpy as np
from matplotlib import pyplot as plt

class Position:
    def __init__(self, x: float, y: float, rotation: float = 0.0):
        self.x = x
        self.y = y
        self.rotation = rotation

    @staticmethod
    def fromXVector(x:float, y:float, xVector: List[float,float]) -> Position:
        rotation = math.atan2(xVector[1], xVector[0])
        return Position(x, y, rotation)
    
    def __str__(self):
        return f'Position(x={self.x}, y={self.y}, rotation={self.rotation})'

    def __repr__(self):
        return self.__str__()

    def __add__(self, other:Position) -> Position:
        return Position(self.x + other.x, self.y + other.y, self.rotation + other.rotation)

    def __iadd__(self, other:Position) -> Position:
        self.x += other.x
        self.y += other.y
        self.rotation += other.rotation
        return self
    
    def __sub__(self, other:Position) -> Position:
        return Position(self.x - other.x, self.y - other.y, self.rotation - other.rotation)
    
    def __isub__(self, other:Position) -> Position:
        self.x -= other.x
        self.y -= other.y
        self.rotation -= other.rotation
        return self

    def distance(self, other:Position) -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def plotPosition(self, axe:plt.axis, lineLength:float=0.5, color:str='r'):
        # Plot the position as a colored dot
        axe.plot(self.x, self.y, f'{color}o')

        # Plot the line as a colored line
        line_x = self.x + lineLength * math.cos(self.rotation)
        line_y = self.y + lineLength * math.sin(self.rotation)
        axe.plot([self.x, line_x], [self.y, line_y], color)



class Transform:

    def __init__(self, localDatum:Position, setTransformMatrices:bool=True) -> None:
        self.localDatum = localDatum

        self.globalToLocalRotation = localDatum.rotation

        if setTransformMatrices:
            self.updateTransformMatrices()

    def updateLocalToGlobalTransformMatrix(self) -> None:
        self.localToGlobalTransformMatrix = np.array([[math.cos(self.localDatum.rotation), -math.sin(self.localDatum.rotation), self.localDatum.x],[math.sin(self.localDatum.rotation), math.cos(self.localDatum.rotation), self.localDatum.y], [0, 0, 1]])

    def updateGlobalToLocalTransformMatrix(self) -> None:
        self.globalToLocalTransformMatrix = np.array([[math.cos(self.localDatum.rotation), math.sin(self.localDatum.rotation), -self.localDatum.x],[-math.sin(self.localDatum.rotation), math.cos(self.localDatum.rotation), -self.localDatum.y], [0, 0, 1]])
    
    def updateTransformMatrices(self) -> None:
        self.updateLocalToGlobalTransformMatrix()
        self.updateGlobalToLocalTransformMatrix()

    def getLocalPosition(self, globalPosition: Position, updateTransform: bool = False)->Position:

        if updateTransform:
            self.updateGlobalToLocalTransformMatrix()

        localPosition = self.globalToLocalTransformMatrix @ np.array([globalPosition.x, globalPosition.y, 1])

        return Position(localPosition[0], localPosition[1], globalPosition.rotation - self.localDatum.rotation)
        
    def getGlobalPosition(self, localPosition:Position, updateTransform: bool = False)->Position:
        if updateTransform:
            self.updateLocalToGlobalTransformMatrix()

        globalPosition = self.localToGlobalTransformMatrix @ np.array([localPosition.x, localPosition.y, 1])

        return Position(globalPosition[0], globalPosition[1], localPosition.rotation + self.localDatum.rotation)


if __name__ == "__main__":

    fig, axe = plt.subplots()
    
    zeroPosition = Position(0, 0, 0)
    zeroPosition.plotPosition(axe)
    firstPosition = Position(1, 1, math.pi/4)
    firstPosition.plotPosition(axe)
    secondPosition = Position(2, 2, math.pi/3)
    secondPosition.plotPosition(axe)
    thirdPosition = Position(3, 3, math.pi/2)
    thirdPosition.plotPosition(axe)
    fourthPosition = Position.fromXVector(4, 4, [-1, -1])
    fourthPosition.plotPosition(axe)

    transform = Transform(thirdPosition)

    transform.getLocalPosition(Position(4,4,0))

    localFirst = transform.getLocalPosition(firstPosition)
    print(localFirst)
    globalFirst = transform.getGlobalPosition(localFirst)
    print(globalFirst)

    print(transform.getGlobalPosition(Position(0,0,0)))
    print(transform.getLocalPosition(thirdPosition))

    fifthPostion = transform.getGlobalPosition(Position(-1,-1,math.pi))
    fifthPostion.plotPosition(axe,color="b")

    plt.show()


