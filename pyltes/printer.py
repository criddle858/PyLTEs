__author__ = 'Mariusz'

from pyltes import devices
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import math

class Printer:
    """Class that prints network deployment"""
    def __init__(self,parent):
        self.parent = parent

    def drawHistogramOfUEThroughput(self, filename):
        thr_vector = self.parent.returnRealUEThroughputVectorRR()
        thr_MBit = [x / (1024*1024) for x in thr_vector]
        plt.hist(thr_MBit)
        # plt.savefig(filename, format="pdf", dpi=300)
        plt.savefig(filename+".png", format="png", dpi=300)
        plt.clf()

    def drawHistogramOfSetPowers(self, filename):
        power_vector = []
        for bs in self.parent.bs:
            power_vector.append(bs.outsidePower)
        plt.hist(power_vector, bins=np.arange(self.parent.minFemtoTxPower, self.parent.maxTxPower + 1, 1))
        plt.xlim(0, 100)
        # plt.savefig(filename, format="pdf", dpi=300)
        plt.savefig(filename+".png", format="png", dpi=300)
        plt.clf()

    def drawNetwork(self, filename, BS=True, UE=True, links=True, obstacles=True, fillMethod="SINR", colorMap = 'viridis', drawLegend=True, tilesInLine = 100, figSize = (8, 8), colorMinValue = None, colorMaxValue = None, outputFileFormat = ["png"]):
        main_draw = plt.figure(1, figsize=figSize)
        ax = main_draw.add_subplot(111)
        
        cm = plt.cm.get_cmap(colorMap)
        ue = devices.UE()
        imageMatrix = np.zeros((tilesInLine, tilesInLine))
        d_x = round(self.parent.constraintAreaMaxX/tilesInLine)
        d_y = round(self.parent.constraintAreaMaxY/tilesInLine)

#         if fillMethod == "SINR":
        for x in range(0, tilesInLine):
            for y in range(0, tilesInLine):
                ue.x = x * d_x
                ue.y = y * d_y
                if fillMethod == "SINR":
                    ue.connectToTheBestBS(self.parent.bs, self.parent.obstacles)
                    debug = 0
                    SINR, RSSI = ue.calculateSINR(self.parent.bs, self.parent.obstacles)#, debug)
                    imageMatrix[y][x] = SINR
                if fillMethod == "ccr":
                    RSSI_best = -1000
                    BS_best = -1
                    for bs in self.parent.bs:
                        ue.connectedToBs = bs.ID
                        temp_RSSI, RSRP = ue.calculateSINR(self.parent.bs)
                        if temp_RSSI > RSSI_best:
                            RSSI_best = temp_RSSI
                            BS_best = bs.ID
                    imageMatrix[y][x] = bs.ID
                    
        if colorMinValue != None:
            colorMin = colorMinValue
        else:
            colorMin = imageMatrix.min()
        if colorMaxValue != None:
            colorMax = colorMaxValue
        else:
            colorMax = imageMatrix.max()
        image = plt.imshow(imageMatrix, vmin=colorMin, vmax=colorMax, origin='lower', extent=[0, self.parent.constraintAreaMaxX, 0, self.parent.constraintAreaMaxY], interpolation='nearest', cmap=cm)
        if drawLegend == True:
            from mpl_toolkits.axes_grid1 import make_axes_locatable
            divider = make_axes_locatable(ax)
            cax1 = divider.append_axes("right", size="5%", pad=0.05)
            cbar = plt.colorbar(image, cax = cax1)
            cbar.set_clim(-40, 40) #end

        if fillMethod == "Sectors":
            cm = plt.cm.get_cmap(colorMap)
            ue = devices.UE()
            imageMatrix = np.zeros((tilesInLine, tilesInLine))
            d_x = round(self.parent.constraintAreaMaxX/tilesInLine)
            d_y = round(self.parent.constraintAreaMaxY/tilesInLine)
            for x in range(0, tilesInLine):
                for y in range(0, tilesInLine):
                    RSSI_best = -1000
                    BS_best = -1
                    for bs in self.parent.bs:
                        ue.x = x * d_x
                        ue.y = y * d_y

                        ue.connectedToBS = bs.ID
                        temp_RSSI, _ = ue.calculateSINR(self.parent.bs)
                        if temp_RSSI > RSSI_best:
                            RSSI_best = temp_RSSI
                            BS_best = bs.ID

                    imageMatrix[y][x] = BS_best
            plt.imshow(imageMatrix, origin='lower', extent=[0, self.parent.constraintAreaMaxX, 0, self.parent.constraintAreaMaxY], interpolation='nearest', cmap=cm)

        if BS == True:
            bs_x_locations = []
            bs_y_locations = []
            bs_ID = []
            bs_angle = []
            bs_count = 0
            for bs in self.parent.bs:
                bs_x_locations.append(bs.x)
                bs_y_locations.append(bs.y)
                bs_ID.append(bs.ID)
                bs_angle.append(bs.angle)
                bs_count+=1
            ax.plot(bs_x_locations, bs_y_locations, 'r^', color="red", markersize=10)
            for i in range(0,bs_count):
                offsetAngle = math.radians(bs_angle[i])
                distance = (self.parent.constraintAreaMaxX + self.parent.constraintAreaMaxY) / 50
                z = distance*complex(math.sin(offsetAngle), -math.cos(offsetAngle))
                ax.annotate(bs_ID[i], xy=(bs_x_locations[i],bs_y_locations[i]), xytext=(bs_x_locations[i]+z.real, bs_y_locations[i]+z.imag), color='red')
                        
        if UE == True:
            ue_x_locations = []
            ue_y_locations = []
            for ue in self.parent.ue:
                ue_x_locations.append(ue.x)
                ue_y_locations.append(ue.y)
            ax.plot(ue_x_locations, ue_y_locations, 'b*', color="black", markersize=10)

        if links == True:
            for ue in self.parent.ue:
                ax.arrow(ue.x, ue.y, self.parent.bs[ue.connectedToBS].x - ue.x, self.parent.bs[ue.connectedToBS].y - ue.y)

        if obstacles == True:
            for obstacle in self.parent.obstacles:
                ax.arrow(obstacle[0], obstacle[1], obstacle[2] - obstacle[0], obstacle[3] - obstacle[1], width=10, color="red")

        networkBorder = plt.Rectangle((0,0), self.parent.constraintAreaMaxX, self.parent.constraintAreaMaxY, color='black', fill=False)
        ax.add_patch(networkBorder)
        ax.axis('equal')

        ax.axis([0, self.parent.constraintAreaMaxX, 0, self.parent.constraintAreaMaxY])
        ax.axis('off')
        ax.xaxis.set_visible(False)
        ax.yaxis.set_visible(False)
        for outputFormat in outputFileFormat:
            if outputFormat == "png":
                main_draw.savefig(filename+".png", format="png", dpi=300, bbox_inches='tight')
            if outputFormat == "pdf":
                main_draw.savefig(filename+".pdf", format="pdf", dpi=300, bbox_inches='tight')
        
        plt.clf()
