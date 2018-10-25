#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
UAV properties class
also includes a position class for Lat/Long, Altitude, Heading and speed
Oscar Bowen Schofield (osbow17@student.sdu.dk)

"""

class uavDetails(object):
    """docstring for uavProp."""
    def __init__(self, inputID, inAuthKey = ""):
        self.uavID = inputID
        self.authKey = inAuthKey

        self.uavName = ""
        self.uavStatus = -1
        self.uavWeight = 0
        self.uavMaxVel = 0
        self.uavMaxEndur = 0
        self.currPos = geoPosition()
        self.wpPos = geoPosition()
        self.serverTime = 000000
        self.GDPRComp = "No"

    def getCurrPos(self):
        geoPos = {self.currPos.Lat, self.currPos.Lon, self.currPos.Alt}
        return geoPos

    def getWaypoint(self):
        geoPos = {self.currPos.Lat, self.currPos.Lon, self.currPos.Alt}
        return geoPos

    def setAuth(self, inputID, inAuthKey):
        self.uavID = inputID
        self.authKey = inAuthKey,
        pass

    def updatePosition(self, inLat, inLon, inAlt, inHeading):
        self.currPos.Lat = inLat
        self.currPos.Lon = inLon
        self.currPos.Alt = inAlt
        self.currPos.heading = inHeading

class geoPosition(object):
    """
        geoPosition class.
        provides Latitude/longitude (stored in WGS84 format.)

    """
    def __init__(self):
        self.Lat = 0
        self.Lon = -1
        self.Alt = -1
        self.Heading =-1
        self.time = 00000
if __name__ == '__main__':
    testUAV = uavDetails(1000, "TEST", )
