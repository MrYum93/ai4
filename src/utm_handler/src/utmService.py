#!/usr/bin/env python
"""
Oscar Bowen Schofield (osbow17@student.sdu.dk)
UTM Service: Register Drone with the droneid system (https://droneid.dk/borger/index.php)
Based on the work by Tobias Lundby (tolu@mmmi.sdu.dk)
04-10-2018  created files

"""
import sys
import requests
import json
import time
import rospy

from mavlink_lora.msg import mavlink_lora_pos
from classUavProp import uavDetails

class utmService():
    """docstring for utmService."""
    def __init__(self):
        #setup publishers/ subscribers
        rospy.init_node("utmServiceHandler")
        self.coordSub = rospy.Subscriber('/mavlink_pos', mavlink_lora_pos, self.on_positionUpdate)
        self.droneID = 3011
        self.droneAuthKey = "2dff89a9c4446c961135538623177dcf8916319edf6e34fed034394777ea136ba7201b8026f58312779101bf5a6afadb472512b4917abd1878efa87f92537056"
        self.droneDetails = uavDetails(self.droneID, self.droneAuthKey)
    """ checkResponse: check whether incoming/outgoing comms work """
    def checkResponse(serverResp):

        goodResp = False
        try:
            serverResp.raise_for_status()
        except requests.exceptions.Timeout:
    	    # Maybe set up for a retry, or continue in a retry loop
    		print colored('Request has timed out', 'red')
        except requests.exceptions.TooManyRedirects:
            # Tell the user their URL was bad and try a different one
            print colored('Request has too many redirects', 'red')
        except requests.exceptions.HTTPError as err:
            print colored('HTTP error', 'red')
            print colored(err, 'yellow')
            #sys.exit(1) # Consider the exit since it might be unintentional in some cases
        except requests.exceptions.RequestException as err:
            # Catastrophic error; bail.
            print colored('Request error', 'red')
            print colored(err, 'yellow')
        else:
            goodResp = True
            return goodResp

    ## Core UTM-Server funcions ##
    def registerUav(self):
        pass
    def sendUavPos(self):
        # TODO: update coordinates to current position.
        payload = {
        'uav_id'            : self.droneID,
        'uav_auth_key'      : self.droneAuthKey,
        'uav_op_status'     : -1,
        'pos_cur_lat_dd'    : self.droneDetails.currPos.Lon,
        'pos_cur_lng_dd'    : self.droneDetails.currPos.Lat,
        'pos_cur_alt_m'     : self.droneDetails.currPos.Alt,
        'pos_cur_hdg_deg'   : self.droneDetails.currPos.Heading,
        'pos_cur_vel_mps'   : 0,
        'pos_cur_gps_timestamp' : 123456,
        'wp_next_lat_dd'    : 55.367945,
        'wp_next_lng_dd'    : 10.434963,
        'wp_next_alt_m'     : 50,
        'wp_next_hdg_deg'   : 0,
        'wp_next_vel_mps'   : 0,
        'wp_next_eta_epoch' : time.time()+60,          #this is the time now + 1 min
        'uav_bat_soc': 100
        }
        r = ''
        # r = requests.post(url = 'https://droneid.dk/rmuasd/utm/tracking_data.php', data=payload, timeout = 2)
        print ('posted payload:')
        print ('lat: %.4f , lon: %.4f, Alt: %.4f, heading: %.4f' % (payload['pos_cur_lat_dd'], payload['pos_cur_lng_dd'], payload['pos_cur_alt_m'], payload['pos_cur_hdg_deg']))
    def getUAVPos(uavID = 0):
        r = ''
        if uavID > 0 :          # request a specific uav data
            payload = {'uav_id': uavID}
            r = requests.get(url = 'https://droneid.dk/rmuasd/utm/tracking_data.php', params=payload, timeout = 2)
        else:                   # request all available tracking data
            r = requests.get(url = 'https://droneid.dk/rmuasd/utm/tracking_data.php', timeout = 2)
        if checkResponse(r) == True:
            try:
                dataIn = ''
                dataIn = json.loads(r.text)
                print(r.text)
            except:
                print colored('Error in parsing of data to JSON', 'red')
            else:
                for entry in dataIn:
                    # TODO: convert print to uavDetails format
                    print("Drone_ID %i, \t Drone_name %s, Op_Status %i, Server_Entry: %i" % (entry['uav_id'],entry['uav_op_status'],entry['uav_bat_soc'],entry['time_epoch']))
                    print("Pos_Lat %f, \t Pos_Lon %f \t Curr_Alt %i \t Curr_Head %i \t Curr_Vel %i \t GPS_time %i" % (entry['pos_cur_lat_dd'],entry['pos_cur_lng_dd'],entry['pos_cur_alt_m'],entry['pos_cur_hdg_deg'],entry['pos_cur_vel_mps'],entry['pos_cur_gps_timestamp']))
                    print("WP_Lat %f, \t WP_Lon %f \t WP_Alt %i \t WP_Head %i \t WP_Vel %i \t WP_ETA %i" % (entry['wp_next_lat_dd'],entry['wp_next_lng_dd'],entry['wp_next_alt_m'],entry['wp_next_hdg_deg'],entry['wp_next_vel_mps'],entry['wp_next_eta_epoch']))

    def getStaticNFZ(self):
        pass
    def getUAVInfo(self, uavID):
        r = ''
        payload = {'uav_id': uavID}
        r = requests.get(url = 'https://droneid.dk/rmuasd/utm/uav.php', params=payload, timeout = 2)
        if checkResponse(r) == True:
            try:
                dataIn = ''
                dataIn = json.loads(r.text)
            except:
                print colored('Error in parsing of data to JSON', 'red')
            else:
                for entry in dataIn:
                    print("Drone_ID %i, \t Drone_name %s" % (entry['uav_id'], entry['uav_name']))
                    print("Drone_Weight (Kg) %.2f, Drone_Max_Velocity (m/s): %.2f" % (entry['uav_weight_kg'], entry['uav_max_vel_mps']))
                    print("Drone_Max_Endurance (s) %i "% entry['uav_max_endurance_s'])

    ## ROS-Specific functions ##
    def on_positionUpdate(self, msg):
        lat = msg.lat
        lon = msg.lon
        alt = msg.alt
        head = msg.heading

        print("RCVD: Lat:%.4f \t Lon: :%.4f, \t Alt:%.2f, \t Head::%.1f" % (lat, lon, alt, head))
        droneDetails.updatePosition(msg.lat, msg.lon, msg.alt, msg.heading)
        # if time elapsed > 1 second, update the UTM server
        # rospy.Timer(rospy.Duration(1), self.sendUavPos())

def main():

    utmCMD = utmService()
    r = rospy.Rate(10)

    while not (rospy.is_shutdown()):
        rospy.Timer(rospy.Duration(1), utmCMD.sendUavPos())
        r.sleep()


if __name__ == '__main__':
    main()
