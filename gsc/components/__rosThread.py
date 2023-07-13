#!/usr/bin/env python3
########################################################################
# Filename    : __rosThread.py
# Description : ros node controller for gcs
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################

import math
import json
import socket
import numpy as np
from time import process_time, sleep

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage # Image is the message type
from std_msgs.msg import String, Int8, UInt16
from geometry_msgs.msg import Vector3


from ..utils.__utils_objects import AVAILABLE_TOPICS, DRONES_NAMES, PEER


class GCSNode( Node ):

    def __init__( self, Master=None, **kwargs):

        super().__init__("gui", namespace="master")

        self._master = Master
            
        self._drone_index = -1
        self._gameplayStatus = {}
        #heartbeats part
        self._pub_heartbeat = None
        self._beat_pulsation = 1.0

        self._pub_shutdown = None

        #remap subscribers
        self._sub_video = None

        self._video_bridge = None 
        self._propulsion_level = 50

        self._is_peer_connected = False
        self._force_commands_enabled = False

        self._operator_type = PEER.MASTER.value

        self._save_direction = 0 

        self._start()


    def _start(self):

        self.get_local_ip()
        self._init_components()
        self._init_subscribers()
        self._init_publishers()
        
    
    def get_local_ip( self ):

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:

            s.connect(('8.8.8.8', 80))
            self._address = s.getsockname()[0]
        except socket.error:
            # Si la connexion échoue, nous renvoyons l'adresse IP de la machine locale
            self._address = socket.gethostbyname(socket.gethostname())
        finally:
            s.close()


    def get_drones_names( self ):
        return self.get_parameter('names').value
    
        
    def _init_components( self ):

        self._video_bridge = CvBridge()
        
        gameplay_default_state = {
            "enable" : False, 
            "playtime": 10*60,
            "stop" : False
        }

        for i in range( len( DRONES_NAMES ) ):

            if( i == 0): continue
            else:
                self._gameplayStatus[f"peer_{i-1}"] = gameplay_default_state


    def _init_subscribers( self ):
            
        self._listen_drones_sensors()
        self._listen_operator_sensors()

    def _listen_operator_sensors( self ):
            
        for i in range( len( DRONES_NAMES ) ):
                
            if( i == 0): continue
            else:
                    
                operator_index = i-1
                sub_sensor = self.create_subscription(
                    String,
                    f"/{PEER.USER.value}_{operator_index}/{AVAILABLE_TOPICS.SENSOR.value}",
                    self._operator_sensors_datas,
                    qos_profile = qos_profile_sensor_data
                )

                sub_sensor   


    def _listen_drones_sensors( self ):

        for i in range( len( DRONES_NAMES ) ):
                
            if( i == 0): continue
            else:

                drone_index = i-1

                sub_sensor = self.create_subscription(
                    String,
                    f"/{PEER.DRONE.value}_{drone_index}/{AVAILABLE_TOPICS.SENSOR.value}",
                    self._drone_sensors_datas,
                    qos_profile = qos_profile_sensor_data
                )

                sub_sensor
    

    def _init_publishers( self ):

        self._pub_shutdown = self.create_publisher(
            String, 
            AVAILABLE_TOPICS.SHUTDOWN.value,
            10
        )

        self._pub_shutdown

        self._pub_heartbeat = self.create_publisher(
            String, 
            AVAILABLE_TOPICS.HEARTBEAT.value,
            qos_profile=qos_profile_sensor_data
        )

        self._pub_heartbeat

        self.timer = self.create_timer( self._beat_pulsation, self._pulse )

        
    def _remap_drone_control( self, update_index ):
            
        update_index -= 1

        if update_index != self._drone_index :
                
            self._drone_index = update_index
            self._remap_video_subscriber( )

            
    def _remap_video_subscriber( self ):
            
        if self._sub_video is not None:
                    
            self.destroy_subscription( self._sub_video )
            sleep( 2 )

            self._sub_video = None

        if self._drone_index != None and self._drone_index != -1:

            self._sub_video = self.create_subscription(
                CompressedImage, 
                f"/{PEER.DRONE.value}_{self._drone_index}/{AVAILABLE_TOPICS.STREAM.value}", 
                self._on_videostream, 
                qos_profile=qos_profile_sensor_data
            )

            self._sub_video


    def  _enable_gameplay( self, index, enable, playtime ):
        
        peer = f"peer_{index}"

        if peer in self._gameplayStatus:

            gameplayUpdate = {
                "enable" : enable,
                "playtime" : playtime
            }

            self._gameplayStatus[peer] = gameplayUpdate

    def _set_on_stop(self, index, isStopped ):
        
        peer = f"peer_{index}"

        if peer in self._gameplayStatus:

            self._gameplayStatus[peer]["stop"] = isStopped


    def _pulse( self ):
            
        if self._pub_heartbeat is not None: 
                
            pulse_msg = String()

            info = {
                "address" : self._address,
                "operator" : self._operator_type,
                "control" : self._drone_index, 
                "peers" : self._gameplayStatus
            }

            pulse_msg.data = json.dumps( info )

            self._pub_heartbeat.publish( pulse_msg )


    def _on_videostream( self, frame ):

        current_frame = self._video_bridge.compressed_imgmsg_to_cv2(frame, desired_encoding="passthrough")

        if self._master is not None:
            self._master._gui._update_frame( current_frame )


    def _update_direction( self, spin_direction = 0 ):
        """
        update direction state value
        """
        spin = int(np.clip( spin_direction, -1, 1 ))
            
        if self._save_direction != spin: 
            self._save_direction = spin

            dir_msg = Int8()
            dir_msg.data = spin

            self._master._gui._set_direction(spin )

    def _update_propulsion( self, update_pwm = 50.0 ):
        
        """
        update propulsion slider value
        """

        increment = math.floor( np.clip( update_pwm, 50, 200 ) ) 
            
        prop_msg = UInt16()
        prop_msg.data = increment

        ratio = ( increment - 50 ) / 150
        self._master._gui._set_propulsion( ratio )

    def _update_panoramic( self, panAngle = 90, TiltAngle = 90 ):
        """
        update panoramic slider value
        """
        vec = Vector3()
        vec.x = float(TiltAngle)
        vec.z = float(panAngle)

        self._master._gui._set_pan_value( panAngle )
        self._master._gui._set_tilt_value( TiltAngle  )

    def _drone_sensors_datas( self, msg ): 
            
        drone_sensors = json.loads( msg.data )

        if( "index" in drone_sensors and "datas" in drone_sensors ):
            self._master._gui.OnDronesDatas( drone_sensors["index"], drone_sensors["datas"] )

    def _operator_sensors_datas( self, msg ): 
            
        operator_sensors = json.loads( msg.data )

        if( "index" in operator_sensors and "datas" in operator_sensors ):
            self._master._gui.OnOperatorDatas( operator_sensors["index"], operator_sensors["datas"] )


    def _on_gui_closed( self, value ):

        if self._pub_shutdown is not None:
                
            msg = String()

            instruction = {
                "operator" : self._operator_type,
                "status" : value
            }

            msg.data = json.dumps( instruction )        

            self._pub_shutdown.publish( msg )
   
    def _shutdown(self):
            
        self.destroy_node()
        #print("shutdown")
    

