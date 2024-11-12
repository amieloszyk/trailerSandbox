"""
This is just running the math much less elegantly to get some analyses going.
"""
from typing import Tuple
from math import sin, cos, tan, atan2, pi
from dataclasses import dataclass

from scipy.integrate import solve_ivp

from Body import Body
from CoordinateSystems import Position, Transform

def calcConnectionPointState(speed: float, rotation: float, rotationRate: float,
                            axleToConnDist: float) -> Tuple[float, float, float]:
    velConnX = speed * cos(rotation) - rotationRate * axleToConnDist * cos(rotation)
    velConnY = speed * sin(rotation) - rotationRate * axleToConnDist * sin(rotation)

    return (velConnX, velConnY, rotation)

def calcVehicleVelocity(speed: float, rotation: float, rotationRate: float) -> Tuple[float, float, float]:
    velVehicleX = speed * cos(rotation)
    velVehicleY = speed * sin(rotation)

    return (velVehicleX, velVehicleY, rotationRate)

def calcVehicleRotationRate(steeringAngle: float, speed: float, wheelBase: float) -> float:
    return speed * tan(steeringAngle) / wheelBase

def calcTrailerVelocity(vehicleSpeed: float, connToTrailerAxleDist: float,
                       connToVehicleAxleDist: float, vehicleRotation: float,
                       trailerRotation: float, vehicleWheelBase: float,
                       steeringAngle: float) -> Tuple[float, float, float]:
    
    rotationDiff = vehicleRotation - trailerRotation

    trailerAxleSpeed = cos(rotationDiff) + connToVehicleAxleDist * sin(rotationDiff) * tan(steeringAngle) / vehicleWheelBase
    trailerAxleSpeed *= vehicleSpeed

    trailerRotationRate = sin(rotationDiff) - connToVehicleAxleDist * cos(rotationDiff) * tan(steeringAngle) / vehicleWheelBase
    trailerRotationRate *= vehicleSpeed / connToTrailerAxleDist

    return (trailerAxleSpeed * cos(trailerRotation), trailerAxleSpeed * sin(trailerRotation), trailerRotationRate)

@dataclass
class VehicleTrailerDescription:
    vehicleWheelBase : float
    connToTrailerAxleDist : float
    connToVehicleAxleDist : float

class VehicleTrailerPosition:

    vehicle: Body
    trailer: Body

    def __init__(self, vehicle: Body, trailer: Body) -> None:
        self.vehicle = vehicle
        self.trailer = trailer

    def copyFromStateVector(self, stateVector: Tuple[float, float, float, float, float, float]) -> 'VehicleTrailerPosition':
        
        newVehicle = Body(Transform(Position(stateVector[0], stateVector[1], stateVector[2])))
        newVehicle.localOpenOutline = self.vehicle.localOpenOutline
        newVehicle.pointsOfInterest = self.vehicle.pointsOfInterest

        newTrailer = Body(Transform(Position(stateVector[3], stateVector[4], stateVector[5])))
        newTrailer.localOpenOutline = self.trailer.localOpenOutline
        newTrailer.pointsOfInterest = self.trailer.pointsOfInterest
        
        return VehicleTrailerPosition(newVehicle, newTrailer)

    def toStateVector(self) -> Tuple[float, float, float, float, float, float]:
        return (self.vehicle.localDatum.localDatum.x, self.vehicle.localDatum.localDatum.y, self.vehicle.localDatum.localDatum.rotation,
                self.trailer.localDatum.localDatum.x, self.trailer.localDatum.localDatum.y, self.trailer.localDatum.localDatum.rotation)

    @property
    def vehiclePositionX(self) -> float:
        return self.vehicle.localDatum.localDatum.x

    @vehiclePositionX.setter
    def vehiclePositionX(self, value: float) -> None:
        self.vehicle.localDatum.localDatum.x = value

    @property
    def vehiclePositionY(self) -> float:
        return self.vehicle.localDatum.localDatum.y

    @vehiclePositionY.setter
    def vehiclePositionY(self, value: float) -> None:
        self.vehicle.localDatum.localDatum.y = value

    @property
    def vehicleRotation(self) -> float:
        return self.vehicle.localDatum.localDatum.rotation

    @vehicleRotation.setter
    def vehicleRotation(self, value: float) -> None:
        self.vehicle.localDatum.localDatum.rotation = value

    @property
    def trailerPositionX(self) -> float:
        return self.trailer.localDatum.localDatum.x

    @trailerPositionX.setter
    def trailerPositionX(self, value: float) -> None:
        self.trailer.localDatum.localDatum.x = value

    @property
    def trailerPositionY(self) -> float:
        return self.trailer.localDatum.localDatum.y

    @trailerPositionY.setter
    def trailerPositionY(self, value: float) -> None:
        self.trailer.localDatum.localDatum.y = value

    @property
    def trailerRotation(self) -> float:
        return self.trailer.localDatum.localDatum.rotation

    @trailerRotation.setter
    def trailerRotation(self, value: float) -> None:
        self.trailer.localDatum.localDatum.rotation = value

class VehicleTrailerVelocityCalculator:

    steeringAngle : float
    vehicleSpeed : float
    description: VehicleTrailerDescription

    def __init__(self, steeringAngle: float, vehicleSpeed: float, description: VehicleTrailerDescription) -> None:
        self.steeringAngle = steeringAngle
        self.vehicleSpeed = vehicleSpeed
        self.description = description

    def calcVelocityVector(self, _: float, stateVector):
        """
        stateVector:
        0: vehiclePositionX
        1: vehiclePositionY
        2: vehicleRotation
        3: trailerPositionX
        4: trailerPositionY
        5: trailerRotation

        Returns:
        0: vehicleVelocityX
        1: vehicleVelocityY
        2: vehicleRotationRate
        3: trailerVelocityX
        4: trailerVelocityY
        5: trailerRotationRate
        """

        vehicleRotation = stateVector[2]
        trailerRotation = stateVector[5]

        vehicleRotationRate = calcVehicleRotationRate(self.steeringAngle, self.vehicleSpeed,
                                                      self.description.vehicleWheelBase)
        
        vehicleVelocity = calcVehicleVelocity(self.vehicleSpeed, vehicleRotation, vehicleRotationRate)
        trailerVelocity = calcTrailerVelocity(self.vehicleSpeed, self.description.connToTrailerAxleDist,
                                              self.description.connToVehicleAxleDist, vehicleRotation,
                                              trailerRotation, self.description.vehicleWheelBase,
                                              vehicleSteeringAngle)

        return vehicleVelocity + trailerVelocity

def calcNextTrailerState(velCalcer: VehicleTrailerVelocityCalculator,
                         startingState:VehicleTrailerPosition,
                         vehicleTravel: float = 1.0) -> VehicleTrailerPosition:

    dTime = vehicleTravel / velCalcer.vehicleSpeed

    predictionBunch = solve_ivp(velCalcer.calcVelocityVector, (0, dTime),
                                startingState.toStateVector(), max_step=dTime/10.0)

    newState = startingState.copyFromStateVector(predictionBunch.y[:,-1])

    ### Corrector step
    connPosition = (newState.vehiclePositionX + cos(newState.vehicleRotation - pi) * connToVehicleAxleDist,
                    newState.vehiclePositionY + sin(newState.vehicleRotation - pi) * connToVehicleAxleDist,
                    newState.vehicleRotation)
    connToTrailerAngle = atan2(newState.trailerPositionY - connPosition[1], newState.trailerPositionX - connPosition[0])
    correctionAngle = connToTrailerAngle

    newState.trailerPositionX = connPosition[0] + cos(correctionAngle) * connToTrailerAxleDist
    newState.trailerPositionY = connPosition[1] + sin(correctionAngle) * connToTrailerAxleDist
    newState.trailerRotation = correctionAngle - pi

    return newState

if __name__ == "__main__":

    from matplotlib import pyplot as plt
 
    class PltDrivingControls:
        axe: plt.Axes
        velCalcer: VehicleTrailerVelocityCalculator
        oldState: VehicleTrailerPosition
        state: VehicleTrailerPosition

        speed: float = 0.5
        angleInc: float = pi / 180.0

        vehiclePosHistX: list = []
        vehiclePosHistY: list = []
        vehiclePosHistLine: plt.Line2D = None

        trailerPosHistX: list = []
        trailerPosHistY: list = []
        trailerPosHistLine: plt.Line2D = None

        textBox: plt.Text = None
        vehicleOutline: plt.Line2D = None
        trailerOutline: plt.Line2D = None
        connectionPoint: plt.Line2D = None
        
        steeringLineLength: float = None
        steeringLine: plt.Line2D = None
        
        def __init__(self, axe: plt.Axes, velCalcer: VehicleTrailerVelocityCalculator,
                     state: VehicleTrailerPosition) -> None:
            self.axe = axe
            self.velCalcer = velCalcer
            self.oldState = state
            self.state = state
            
            self.vehiclePosHistX.append(state.vehiclePositionX)
            self.vehiclePosHistY.append(state.vehiclePositionY)
            self.trailerPosHistX.append(state.trailerPositionX)
            self.trailerPosHistY.append(state.trailerPositionY)
            self.vehiclePosHistLine = axe.plot(self.vehiclePosHistX, self.vehiclePosHistY, "C0-")[0]
            self.trailerPosHistLine = axe.plot(self.trailerPosHistX, self.trailerPosHistY, "C1-")[0]

            vehicleOutline = self.state.vehicle.getGlobalOpenOutline()
            longestSide = max([point.distance(vehicleOutline[(i+1)%len(vehicleOutline)]) for i, point in enumerate(vehicleOutline)])
            self.steeringLineLength = longestSide / 4.0

            plt.connect('key_press_event', self.update_text)

        def getString(self) -> str:
            return f"{self.velCalcer.steeringAngle/pi*180.0:.1f}"+"$^{\circ}$"

        def makeTextBox(self) -> None:
            self.textBox = self.axe.text(0.05, 0.05, self.getString(), fontsize=15, ha='left', va='bottom', transform=axe.transAxes)
            plt.draw()

        def makeBodies(self) -> None:
            
            self.vehicleOutline = self.axe.plot(self.state.vehiclePositionX, self.state.vehiclePositionY, "C0")[0]
            self.trailerOutline = self.axe.plot(self.state.trailerPositionX, self.state.trailerPositionY, "C1")[0]

            frontAxle = self.state.vehicle.localDatum.getGlobalPosition(self.state.vehicle.pointsOfInterest["frontAxle"])
            self.steeringLine = self.axe.plot([frontAxle.x, frontAxle.x + self.steeringLineLength * cos(frontAxle.rotation + self.velCalcer.steeringAngle)],
                                              [frontAxle.y, frontAxle.y + self.steeringLineLength * sin(frontAxle.rotation + self.velCalcer.steeringAngle)], "k")[0]

            connectionPoint = self.state.vehicle.localDatum.getGlobalPosition(self.state.vehicle.pointsOfInterest["connection"])
            self.connectionPoint = self.axe.plot(connectionPoint.x, connectionPoint.y, "C0o")[0]

            self.updateBodies()
            self.updateAxe()

        def updateBodies(self) -> None:
            vehicleOutlineX = []
            vehicleOutlineY = []
            for vehiclePoint in self.state.vehicle.getGlobalOpenOutline():
                vehicleOutlineX.append(vehiclePoint.x)
                vehicleOutlineY.append(vehiclePoint.y)
            vehicleOutlineX.append(vehicleOutlineX[0])
            vehicleOutlineY.append(vehicleOutlineY[0])

            self.vehicleOutline.set_xdata(vehicleOutlineX)
            self.vehicleOutline.set_ydata(vehicleOutlineY)

            connectionPoint = self.state.vehicle.localDatum.getGlobalPosition(self.state.vehicle.pointsOfInterest["connection"])
            self.connectionPoint.set_xdata([connectionPoint.x])
            self.connectionPoint.set_ydata([connectionPoint.y])

            trailerOutlineX = []
            trailerOutlineY = []
            for trailerPoint in self.state.trailer.getGlobalOpenOutline():
                trailerOutlineX.append(trailerPoint.x)
                trailerOutlineY.append(trailerPoint.y)
            trailerOutlineX.append(trailerOutlineX[0])
            trailerOutlineY.append(trailerOutlineY[0])

            self.trailerOutline.set_xdata(trailerOutlineX)
            self.trailerOutline.set_ydata(trailerOutlineY)

        def updateSteeringLine(self) -> None:
            frontAxle = self.state.vehicle.localDatum.getGlobalPosition(self.state.vehicle.pointsOfInterest["frontAxle"])
            self.steeringLine.set_xdata([frontAxle.x, frontAxle.x + self.steeringLineLength * cos(frontAxle.rotation + self.velCalcer.steeringAngle)])
            self.steeringLine.set_ydata([frontAxle.y, frontAxle.y + self.steeringLineLength * sin(frontAxle.rotation + self.velCalcer.steeringAngle)])
            
        def updateAxe(self):
            axe.relim()
            axe.autoscale_view()
            axe.set_aspect('equal')

            plt.draw()

        def plotOldToNewState(self, colorInc:int = 0) -> None:
            self.vehiclePosHistX.append(self.state.vehiclePositionX)
            self.vehiclePosHistY.append(self.state.vehiclePositionY)
            self.trailerPosHistX.append(self.state.trailerPositionX)
            self.trailerPosHistY.append(self.state.trailerPositionY)

            self.vehiclePosHistLine.set_xdata(self.vehiclePosHistX)
            self.vehiclePosHistLine.set_ydata(self.vehiclePosHistY)
            self.trailerPosHistLine.set_xdata(self.trailerPosHistX)
            self.trailerPosHistLine.set_ydata(self.trailerPosHistY)
            
        def update_text(self, event):
            move = False
            colorInc: int = None
            if event.key == 'up':
                self.velCalcer.vehicleSpeed = self.speed
                move = True
                colorInc = 0
            elif event.key == 'down':
                self.velCalcer.vehicleSpeed = -self.speed
                move = True
                colorInc = 2
            elif event.key == 'right':
                self.velCalcer.steeringAngle -= self.angleInc
            elif event.key == 'left':
                self.velCalcer.steeringAngle += self.angleInc
            elif event.key == 'escape':
                plt.close("all")
                exit()

            self.textBox.set_text(self.getString())

            if move:
                self.oldState = self.state
                self.state = calcNextTrailerState(self.velCalcer, self.oldState, self.velCalcer.vehicleSpeed)
                self.updateBodies()
                self.plotOldToNewState(colorInc)

            self.updateSteeringLine()
            self.updateAxe()



    vehicleRearAxlePostion = (0, 0, 0) # x, y, rotation [m, m, rad]
    vehicleWheelBase = 3.9 # [m]
    vehicleWidth = 2.2 # [m]
    vehicleLength = 5.8 # [m]
    connToVehicleAxleDist = 1.0 # [m]

    vehicle = Body(Transform(Position(*vehicleRearAxlePostion)))
    vehicle.localOpenOutline = [
                                Position(vehicleLength-connToVehicleAxleDist, -vehicleWidth/2),
                                Position(vehicleLength-connToVehicleAxleDist, vehicleWidth/2),
                                Position(-connToVehicleAxleDist, vehicleWidth/2),
                                Position(-connToVehicleAxleDist, -vehicleWidth/2),
                                ]
    vehicle.pointsOfInterest["connection"] = Position(-connToVehicleAxleDist, 0.0)
    vehicle.pointsOfInterest["frontAxle"] = Position(vehicleWheelBase, 0.0)

    connToTrailerAxleDist = 7.3 # [m]
    trailerLength = 9.2 # [m]
    trailerWidth = 2.5 # [m]
    trailerAxlePosition = (-(connToVehicleAxleDist + connToTrailerAxleDist), 0, 0) # x, y, rotation [m, m, rad]
    trailer = Body(Transform(Position(*trailerAxlePosition)))
    trailer.localOpenOutline = [
                                Position(connToTrailerAxleDist, -trailerWidth/2),
                                Position(connToTrailerAxleDist, trailerWidth/2),
                                Position(-(trailerLength-connToTrailerAxleDist), trailerWidth/2),
                                Position(-(trailerLength-connToTrailerAxleDist), -trailerWidth/2),
                                ]
    trailer.pointsOfInterest["connection"] = Position(connToTrailerAxleDist, 0.0)

    vehicleSpeed = 1.0 # [m/s]
    vehicleSteeringAngle = 0.0 # [rad]

    velCalcer = VehicleTrailerVelocityCalculator(vehicleSteeringAngle, vehicleSpeed, VehicleTrailerDescription(vehicleWheelBase,
                                                 connToTrailerAxleDist, connToVehicleAxleDist))
    state = VehicleTrailerPosition(vehicle, trailer)

    fig, axe = plt.subplots()
    driveView = PltDrivingControls(axe, velCalcer, state)
    driveView.makeTextBox()
    driveView.makeBodies()

    axe.plot([-10.0, 20.0], [4.9, 4.9], "k-")
    axe.plot([-10.0, 20.0], [-1.3, -1.3], "k-")
    axe.plot([-10.0, -10.0], [-1.3, -1.3 - 6.7], "k-")
    axe.plot([-10.0, -8], [-1.3 - 6.7, -10.0], "k-")
    axe.plot([-10.0 - 3.6, -10.0 - 3.6], [-1.3, -1.3 - 8.25], "k-")
    axe.plot([-10.0 - 3.6, -16.0], [-1.3 - 8.25, -12.0], "k-")
    axe.plot([-10.0 - 3.6, -20], [-1.3, -1.3], "k-")
    axe.plot([-10.0, -10.0], [4.9, 4.9 + 7.5], "k-")
    axe.plot([-10.0 - 3.6, -10.0 - 3.6], [4.9, 4.9 + 7.5], "k-")
    axe.plot([-10.0 - 3.6, -20], [4.9, 4.9], "k-")



    plt.show()


