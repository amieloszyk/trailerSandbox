from enum import Enum
from typing import Callable, List, Dict
import math

from matplotlib import pyplot as plt

from CoordinateSystems import Position, Transform

def isPointInOutline(self, point: Position, outline: List[Position]) -> bool:
    n = len(outline)
    inside = False

    p1x, p1y = outline[0].x, outline[0].y
    for i in range(n + 1):
        p2x, p2y = outline[i % n].x, outline[i % n].y
        if point.y > min(p1y, p2y):
            if point.y <= max(p1y, p2y):
                if point.x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (point.y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or point.x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside

class CoordinateSystem(Enum):
    LOCAL = 1
    GLOBAL = 2

class Body:
    localDatum : Transform
    localOpenOutline : List[Position]
    pointsOfInterest : Dict[str,Position]
    positionChange : Position

    def __init__(self, localCoords:Transform) -> None:
        self.localDatum = localCoords
        self.localOpenOutline = [Position(-0.5,-0.5), Position(-0.5,0.5), Position(0.5,0.5), Position(0.5,-0.5)]
        self.pointsOfInterest = {}
        self.positionChange = Position(0,0,0)

    def setRectagleOutline(self, width:float, length:float, offset:Position = Position(0.0,0.0)) -> None:
        self.localOpenOutline = [Position(-0.5*width,-0.5*length), Position(-0.5*width,0.5*length), Position(0.5*width,0.5*length), Position(0.5*width,-0.5*length)]
        self.localOpenOutline = [point + offset for point in self.localOpenOutline]

    def containsPoint(self, point:Position, coords:CoordinateSystem) -> bool:
        if coords == CoordinateSystem.GLOBAL:
            point = self.localDatum.getLocalPosition(point)
        return isPointInOutline(point, self.localOpenOutline)

    def updateDatum(self) -> None:

        globalPositionChange = self.localDatum.getGlobalPosition(self.positionChange)

        self.localDatum.localDatum = globalPositionChange
        self.localDatum.updateTransformMatrices()
        self.positionChange = Position(0,0,0)

    def getGlobalOpenOutline(self) -> List[Position]:
        return [self.localDatum.getGlobalPosition(point) + self.positionChange for point in self.localOpenOutline]

    def plot(self, axe:plt.axis, color:str = "b") -> None:
        
        cg = self.localDatum.getGlobalPosition(Position(0,0))
        cg += self.positionChange
        cg.plotPosition(axe, color=color)
        
        globalXs = []
        globalYs = []
        for globalPoint in self.getGlobalOpenOutline():
            globalPoint += self.positionChange
            globalXs.append(globalPoint.x)
            globalYs.append(globalPoint.y)
        globalXs.append(globalXs[0])
        globalYs.append(globalYs[0])
        axe.plot(globalXs, globalYs, 'o-', color=color)
        for point in self.pointsOfInterest.values():
            globalPoint = self.localDatum.getGlobalPosition(point)
            globalPoint += self.positionChange
            globalPoint.plotPosition(axe,color=color)

class MovableBody(Body):

    movementHook : Callable[[float],Position]

    def __init__(self, localCoords:Transform) -> None:
        super().__init__(localCoords)
        self.movementHook = lambda x: Position(0,0,0)

    def setMovementHook(self, hook:Callable[[float],Position]) -> None:
        self.movementHook = hook

    def move(self, timeIncrement:float, updateDatum: bool = False) -> None:
        self.positionChange += self.movementHook(timeIncrement)
        if updateDatum:
            self.updateDatum()

class SteeringFirstSloppy:

    wheelBaseLength: float
    cgTurningRadius: float = 0.0 # Positive is right, negative is left.
    minTurningRadius: float = 1.0
    speed: float = 0.0

    def __init__(self, wheelBaseLength: float) -> None:
        self.wheelBaseLength = wheelBaseLength

    def restrictTurningRadius(self) -> float:
        if self.cgTurningRadius >= 0.0 and self.cgTurningRadius < self.minTurningRadius:
            self.cgTurningRadius = self.minTurningRadius
        elif self.cgTurningRadius < 0.0 and self.cgTurningRadius > -self.minTurningRadius:
            self.cgTurningRadius = -self.minTurningRadius

    def move(self, timeIncrement: float) -> Position:
        self.restrictTurningRadius()
        changeInCircumference = self.speed * timeIncrement
        changeInAngle = changeInCircumference / -self.cgTurningRadius
        return Position(self.cgTurningRadius * math.sin(-changeInAngle),
                        self.cgTurningRadius * (1.0 - math.cos(changeInAngle)),
                        changeInAngle)

    def calcSteeringAngle(self) -> float:
        self.restrictTurningRadius()
        cgTurningAngle = math.asin(0.5 * self.wheelBaseLength / self.cgTurningRadius)
        rearTurningRadius = 0.5 * self.wheelBaseLength / math.tan(cgTurningAngle)
        return math.atan(self.wheelBaseLength / rearTurningRadius)

    def setTurningRadius(self, steeringAngle: float) -> None:

        # TODO: I don't trust this math
        if steeringAngle == 0.0:
            self.cgTurningRadius = self.wheelBaseLength * 1.0e6
            return None

        rearTurningRadius = self.wheelBaseLength / math.tan(steeringAngle)
        cgTurningAngle = math.atan(0.5 * self.wheelBaseLength / rearTurningRadius)
        self.cgTurningRadius = 0.5 * self.wheelBaseLength / math.sin(cgTurningAngle)
        self.restrictTurningRadius()

class Steering:

    wheelBaseLength: float
    frontAxelTurningRadius: float = 0.0 # Positive is right, negative is left.
    minTurningRadius: float
    speed: float = 0.0
    cgFractionRearToFront: float

    def __init__(self, wheelBaseLength: float, cgFractionRearToFront: float = 0.5) -> None:
        self.wheelBaseLength = wheelBaseLength
        self.cgFractionRearToFront = cgFractionRearToFront
        self.minTurningRadius = wheelBaseLength * 1.1

    def restrictTurningRadius(self) -> float:
        if self.frontAxelTurningRadius >= 0.0 and self.frontAxelTurningRadius < self.minTurningRadius:
            self.frontAxelTurningRadius = self.minTurningRadius
        elif self.frontAxelTurningRadius < 0.0 and self.frontAxelTurningRadius > -self.minTurningRadius:
            self.frontAxelTurningRadius = -self.minTurningRadius

    def calcRearAxleTurningRadius(self) -> float:
        turnAngle = self.calcSteeringAngle()
        return math.cos(turnAngle) * self.frontAxelTurningRadius

    def moveCg(self, timeIncrement: float) -> Position:
        self.restrictTurningRadius()
        arcLength = self.speed * timeIncrement
        
        axelRotation = arcLength / -self.frontAxelTurningRadius
        dxFront = self.frontAxelTurningRadius * math.sin(-axelRotation)
        dyFront = self.frontAxelTurningRadius * (math.cos(axelRotation) - 1.0)

        rearAxelTurningRadius = self.calcRearAxleTurningRadius()
        dxRear = rearAxelTurningRadius * math.sin(-axelRotation)
        dyRear = rearAxelTurningRadius * (math.cos(axelRotation) - 1.0)

        dxCg = dxRear * self.cgFractionRearToFront + dxFront * (1.0 - self.cgFractionRearToFront)
        dyCg = dyRear * self.cgFractionRearToFront + dyFront * (1.0 - self.cgFractionRearToFront)
        dRotation = axelRotation

        return Position(dxCg, dyCg, dRotation)

    def calcSteeringAngle(self) -> float:
        return math.asin(self.wheelBaseLength / -self.frontAxelTurningRadius)

    def setTurningRadius(self, steeringAngle: float) -> None:
        if steeringAngle == 0.0:
            self.frontAxelTurningRadius = self.wheelBaseLength * 1.0e6
            return None
        self.frontAxelTurningRadius = -self.wheelBaseLength / math.sin(steeringAngle)


class Truck:

    body: MovableBody
    steering: Steering

    def __init__(self, wheelbaseLength: float, length: float, width: float, startingPosition: Position) -> None:    
        
        self.body = MovableBody(Transform(startingPosition))
        self.body.setRectagleOutline(length, width)
        self.body.pointsOfInterest["hitch"] = Position(-length / 2.0,0.0)
        self.body.pointsOfInterest["rearAxle"] = Position(-wheelbaseLength/2.0,0.0)
        
        self.steering = Steering(wheelbaseLength)
        self.body.setMovementHook(self.steering.moveCg)

    def move(self, timeIncrement: float, updateDatum: bool = False) -> None:
        self.body.move(timeIncrement, updateDatum)

class Trailer:

    body: MovableBody
    steering: Steering
    truck: Truck

    def __init__(self, truck: Truck, ballToAxleLength: float, length: float, width: float) -> None:
        self.truck = truck
        
        hitchPos = truck.body.localDatum.getGlobalPosition(truck.body.pointsOfInterest["hitch"])
        # TODO: This should project based on the orientation of the truck
        self.body = MovableBody(Transform(hitchPos+Position(-ballToAxleLength/2.0,0.0)))

        self.body.setRectagleOutline(length, width, Position((ballToAxleLength-length)/2.0,0.0))
        self.body.pointsOfInterest["hitch"] = Position(ballToAxleLength / 2.0,0.0)
        self.body.pointsOfInterest["rearAxle"] = Position(-ballToAxleLength / 2.0,0.0)
        
        self.steering = Steering(ballToAxleLength, 1.0)
        self.body.setMovementHook(self.steering.moveCg)

    def move(self, timeIncrement: float, updateDatum: bool = False) -> None:

        # TODO: This is all wrong.
        # Calculate the trailer hitch turn radius as the hypotenus of the truck rear axle turn radius
        # and the distance from the truck rear axle to the trailer hitch.
        # Set the trailer speed as speed of the truck hitch.

        truckRearAxleTurnRadius = self.truck.steering.calcRearAxleTurningRadius()
        hitchToTruckRearAxleDistance = self.truck.body.pointsOfInterest["hitch"].distance(self.truck.body.pointsOfInterest["rearAxle"])
        hitchTurnRadius = math.hypot(truckRearAxleTurnRadius, hitchToTruckRearAxleDistance)

        self.steering.frontAxelTurningRadius = hitchTurnRadius
        self.steering.speed = self.truck.steering.speed * hitchTurnRadius / self.truck.steering.frontAxelTurningRadius

        # rearAxleGlobal = self.truck.body.localDatum.getGlobalPosition(self.truck.body.pointsOfInterest["rearAxle"])
        # truckRearAxleLocal = self.body.localDatum.getLocalPosition(rearAxleGlobal)
        # dx = truckRearAxleLocal.x - self.body.pointsOfInterest["hitch"].x
        # dy = truckRearAxleLocal.y - self.body.pointsOfInterest["hitch"].y
        # steeringAngle = math.atan2(dy,dx)

        # self.steering.setTurningRadius(steeringAngle)

        self.body.move(timeIncrement, updateDatum)


if __name__ == "__main__":

    import math
    
    fig, axe = plt.subplots()

    testCircleRadius = 10.0

    testTruck = Truck(3.18,6.0,2.0,Position(0,testCircleRadius,0))
    testTruck.body.plot(axe)

    testTrailer = Trailer(testTruck, 7.0, 10.0, 2.6)
    testTrailer.body.plot(axe, "r")
    
    testTruck.steering.speed = 1.0
    testTruck.steering.frontAxelTurningRadius = testCircleRadius
    testTruck.steering.minTurningRadius = 5.0
    testTrailer.steering.speed = testTruck.steering.speed
    
    nStep = 2
    fracOfCircle = 0.25/4
    dt = 2.0 * math.pi * testCircleRadius * fracOfCircle / testTruck.steering.speed / nStep
    for idx in range(nStep):
        testTruck.move(dt, True)
        testTrailer.move(dt, True)

        testTruck.body.plot(axe,color=f"C{idx%10}")
        testTrailer.body.plot(axe,color=f"C{(idx+2)%10}")

    # TODO: Add an inspector that tracks the points of interest as lists through time as entries in a dictionary
    # TODO: Use outline patch to plot the body outline
    # TODO: make turning left positive, the math is easier

    plt.show()



