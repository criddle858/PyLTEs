__author__ = 'Mariusz Slabicki, Konrad Polys'

import math
import csv
import numpy as np

class NetworkDevice:
    """Network device, needed for inheritance"""
    def __init__(self):
        self.x = 0
        self.y = 0

class UE(NetworkDevice):
    """UE"""
    def __init__(self):
        self.ID = 0
        self.connectedToBS = 0
        self.inside = True

    def distanceToBS(self, BS):
        return math.sqrt((self.x-BS.x)**2+(self.y-BS.y)**2)
    
    def vAngleFromBS(self, BS):
        distance_bs_ue = self.distanceToBS(BS)
        v_angle_rad = math.atan(BS.height/distance_bs_ue)
        v_angle = math.degrees(v_angle_rad)
        return int(v_angle)

    def hAngleFromBS(self, BS):
        a_y = BS.y-self.y
        distance_bs_ue = self.distanceToBS(BS)
        h_angle_rad = math.acos(a_y/distance_bs_ue)
        h_angle = math.degrees(h_angle_rad)
        if self.x <= BS.x:
            h_angle = 360 - h_angle
        return int(h_angle)

    def connectToNearestBS(self, BS_vector):
        closestDistance = -1
        foundBS = -1
        for bs in BS_vector:
            currentDistance = self.distanceToBS(bs)
            if currentDistance < closestDistance or foundBS == -1:
                closestDistance = currentDistance
                foundBS = bs.ID
        self.connectedToBS = foundBS

    def connectToTheBestBS(self, BS_vector, obstacleVector = None):
    #def connectToTheBestBS(self, BS_vector, obstacleVector):
        theBestSINR = -1000
        foundBS = -1
        for bs in BS_vector:
            self.connectedToBS = bs.ID
            currentSINR = self.calculateSINR(BS_vector, obstacleVector)
            if theBestSINR < currentSINR or foundBS == -1:
                theBestSINR = currentSINR
                foundBS = bs.ID
        self.connectedToBS = foundBS

    def calculateWallLoss(self, BS_vector, obstacleVector):
        wallLoss = 0
        for obstacle in obstacleVector:
            s10_x = self.x - BS_vector[self.connectedToBS].x
            s10_y = self.y - BS_vector[self.connectedToBS].y
            s32_x = obstacle[2] - obstacle[0]
            s32_y = obstacle[3] - obstacle[1]

            denom = s10_x * s32_y - s32_x * s10_y

            if denom == 0 :
                continue

            denom_is_positive = denom > 0

            s02_x = BS_vector[self.connectedToBS].x - obstacle[0]
            s02_y = BS_vector[self.connectedToBS].y - obstacle[1]

            s_numer = s10_x * s02_y - s10_y * s02_x

            if (s_numer < 0) == denom_is_positive:
                continue

            t_numer = s32_x * s02_y - s32_y * s02_x

            if (t_numer < 0) == denom_is_positive:
                continue

            if (s_numer > denom) == denom_is_positive or (t_numer > denom) == denom_is_positive :
                continue


            wallLoss = wallLoss + obstacle[4]
        return wallLoss

    def calculateReceivedPower(self, pSend, distance):
        #
        # Replace with simpler model used in Bijan's Matlab
        #
        PL = 128.1 + 37.6*math.log10(distance/1000)
        pRec = pSend - PL
        if(pRec > pSend):
            pRec = pSend
        return pRec


    def calculateNoise(self, bandwidth=20):
        k = 1.3806488 * math.pow(10, -23)
        T = 293.0
        BW = bandwidth * 1000 * 1000
        N = 10*math.log10(k*T) + 10*math.log10(BW)
        return N

    def calculateSINRfor(self, where, BS_vector, obstacleVector = None):
        if (where not in ["in", "out"]):
            raise Exception("wrong argument")

        R = self.distanceToBS(BS_vector[self.connectedToBS])
        if (where=="in"):
            receivedPower_connectedBS=self.calculateReceivedPower(BS_vector[self.connectedToBS].insidePower, R)
        else: # where=="out"
            receivedPower_connectedBS=self.calculateReceivedPower(BS_vector[self.connectedToBS].outsidePower, R)

        h_angle = self.hAngleFromBS(BS_vector[self.connectedToBS])
        v_angle = self.vAngleFromBS(BS_vector[self.connectedToBS])
        
        # 
        # Apply horizontal and vertical antenna gain patterns
        #
        if len(BS_vector[self.connectedToBS].characteristic) != 0:
            receivedPower_connectedBS += float(BS_vector[self.connectedToBS].characteristic[v_angle])
        if obstacleVector != None:
            receivedPower_connectedBS -= self.calculateWallLoss(BS_vector, obstacleVector)

        myColor = BS_vector[self.connectedToBS].color
        receivedPower_otherBS_mw = 0
        receivedPower_one = -999 # dB
        for bs_other in BS_vector:
            if self.connectedToBS == bs_other.ID:
                continue

            if (where=="in" and BS_vector[self.connectedToBS].useSFR):
                sum_power_mw = 0
                for i in range(1,4):
                    if (myColor == i):
                        continue
                    if(bs_other.color == i):
                        bs_other_power = bs_other.outsidePower
                    else:
                        bs_other_power = bs_other.insidePower

                    sum_power_mw += math.pow(10, self.calculateReceivedPower(bs_other_power, self.distanceToBS(bs_other))/10)
                receivedPower_one = 10*math.log10(sum_power_mw/2.0)
            else: # where=="out"
                if(bs_other.color == myColor):
                    bs_other_power = bs_other.outsidePower
                else:
                    bs_other_power = bs_other.insidePower
                receivedPower_one = self.calculateReceivedPower(bs_other_power, self.distanceToBS(bs_other))
                #
                # Add horizontal and vertical antenna pattern gains
                #
        if obstacleVector != None:
            receivedPower_one = receivedPower_one - self.calculateWallLoss(BS_vector, obstacleVector)

        receivedPower_otherBS_mw = receivedPower_otherBS_mw + math.pow(10, receivedPower_one/10)

        I_mw = receivedPower_otherBS_mw
        S_mw = math.pow(10, receivedPower_connectedBS/10)
        N_mw = math.pow(10, self.calculateNoise()/10)

        SINR_mw = S_mw/(I_mw+N_mw)
        SINR = 10*math.log10(SINR_mw)

        return SINR

    def calculateSINR(self, BS_vector, obstacleVector = None):
        if BS_vector[self.connectedToBS].useSFR:
            SINRin = self.calculateSINRfor("in", BS_vector, obstacleVector)
            if(SINRin > BS_vector[self.connectedToBS].mi):
                SINR=SINRin
                self.inside = True
            else:
                SINR=self.calculateSINRfor("out", BS_vector, obstacleVector)
                self.inside = False
        else:
            SINR=self.calculateSINRfor("out", BS_vector, obstacleVector)
            self.inside = False
        return SINR

    def calculateMaxThroughputOfTheNode(self, bs_vector, obstacles = None):
        r_i = 0.0
        M_i = 0.0
        sinr = self.calculateSINR(bs_vector, obstacles)
        if sinr < -5.45:
            r_i = 0
            M_i = 1
        elif -5.45 <= sinr < -3.63:
            r_i = 78/1024
            M_i = 4
        elif -3.63 <= sinr < -1.81:
            r_i = 120/1034
            M_i = 4
        elif -1.81 <= sinr < 0:
            r_i = 193/1024
            M_i = 4
        elif 0 <= sinr < 1.81:
            r_i = 308/1024
            M_i = 4
        elif 1.81 <= sinr < 3.63:
            r_i = 449/1024
            M_i = 4
        elif 3.63 <= sinr < 5.45:
            r_i = 602/1024
            M_i = 4
        elif 5.45 <= sinr < 7.27:
            r_i = 378/1024
            M_i = 16
        elif 7.27 <= sinr < 9.09:
            r_i = 490/1024
            M_i = 16
        elif 9.09 <= sinr < 10.90:
            r_i = 616/1024
            M_i = 16
        elif 10.90 <= sinr < 12.72:
            r_i = 466/1024
            M_i = 64
        elif 12.72 <= sinr < 14.54:
            r_i = 567/1024
            M_i = 64
        elif 14.54 <= sinr < 16.36:
            r_i = 666/1024
            M_i = 64
        elif 16.36 <= sinr < 18.18:
            r_i = 772/1024
            M_i = 64
        elif 18.18 <= sinr < 20:
            r_i = 873/1024
            M_i = 64
        elif 20 <= sinr:
            r_i = 948/1024
            M_i = 64

        if bs_vector[self.connectedToBS].useSFR == True:
            if self.inside:
                capacityForUE_ms = r_i * math.log2(M_i) * 12 * 7 * ((200*(2/3))/1)
                capacityForUE_s = capacityForUE_ms * 1000
            else:
                capacityForUE_ms = r_i * math.log2(M_i) * 12 * 7 * ((200*(1/3))/1)
                capacityForUE_s = capacityForUE_ms * 1000
        else:
            capacityForUE_ms = r_i * math.log2(M_i) * 12 * 7 * ((200)/1)
            capacityForUE_s = capacityForUE_ms * 1000
        return capacityForUE_s

class BS(NetworkDevice):
    """Base Station"""
    def __init__(self):
        self.ID = 0
        self.insidePower = 0
        self.outsidePower = 0
        self.mi = 0
        self.Rc = 1666.3793
        self.color = 1
        self.angle = 0
        self.turnedOn = False
        self.type = "MakroCell"
        self.omnidirectionalAntenna = False
        self.useSFR = False
        self.characteristic = []
        self.height = 32 # feet
        self.tilt = 0    # degrees
        self.vBeamwidth = 0
        self.hBeamwidth = 0
        self.antennaGain = 0    # dBi
        self.hGain = []  # Horizontal antenna gain
        self.vGain = []  # Vertical antenna gain
        self.calculateGain()
        
    def loadCharacteristic(self, filename):
        readCharacteristic = csv.reader(open(filename), delimiter=';', quotechar='|')
        for oneAngle in readCharacteristic:
            self.characteristic.append(float(oneAngle[1]))
    
    def calculateGain(self):
        if self.omnidirectionalAntenna == True:
            self.antnenaGain = 9 #dBi
            self.hBeamwidth = 360 #degrees
            self.vBeamwidth = 11  #degrees
            self.hGain = [0] * 360
            for degree in range(360):
                nDegree = min(degree, (360-degree))
                self.vGain.append(-min(12*(nDegree/self.vBeamwidth)**2, 20))
        else:
            self.antennaGain = 15 #dBi
            self.hBeamwidth = 70  #degrees
            self.vBeamwidth = 11  #degrees
            for degree in range(360):
                nDegree = min(degree, (360-degree))
                self.hGain.append(-min(12*(nDegree/self.hBeamwidth)**2, 25))
                self.vGain.append(-min(12*(nDegree/self.vBeamwidth)**2, 20))
