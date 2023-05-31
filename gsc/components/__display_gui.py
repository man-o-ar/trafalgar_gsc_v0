#!/usr/bin/env python
import math
import os
import base64
import datetime
import numpy as np
from enum import Enum
from time import sleep
from typing import Optional, Tuple, Union
import cv2 # OpenCV library
#import numpy as np
import tkinter
import customtkinter
from tkintermapview import TkinterMapView
from PIL import ImageTk, Image

customtkinter.set_appearance_mode("dark")  # Modes: system (default), light, dark
customtkinter.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

import geocoder

from ..utils.__utils_objects import SENSORS_TOPICS

class EXIT_STATE(str, Enum ):
    ALIVE = "alive"
    SHUTDOWN = "shutdown"
    RESTART = "restart"




DIRECTION_STP = ("EN STANDBY", "#868686")
DIRECTION_FWD = ("MARCHE AVANT", "#028400")
DIRECTION_BWD = ("MARCHE ARRIERE", "#CB4D00")
    

class TrackedDrone( customtkinter.CTkFrame ):
    
    
    def __init__(self, master: any, width: int = 200, height: int = 200, corner_radius: int or str or None = None, border_width: int or str or None = None, bg_color: str or Tuple[str, str] = "transparent", fg_color: str or Tuple[str, str] or None = None, border_color: str or Tuple[str, str] or None = None, background_corner_colors: Tuple[str or Tuple[str, str]] or None = None, overwrite_preferred_drawing_method: str or None = None, masterApp = None, drone_index = 0, drone_name = "", map=None, shipColors = ("#193296", "#ffffff" ), cb_playtime = None,cb_operator = None, **kwargs):
        super().__init__(master, width, height, corner_radius, border_width, bg_color, fg_color, border_color, background_corner_colors, overwrite_preferred_drawing_method, **kwargs)

        self._masterApp = masterApp
        self._index = drone_index

        self._name = drone_name.upper()
        self._ip = ""

        self._map = map
        self._marker = None

        self._shipColors = shipColors

        self._cb_playtime = cb_playtime
        self._cb_operator = cb_operator


        self._playtime = None
        self._operator_switch = None
        self._autopilot_switch = None

        self.operator_status = customtkinter.BooleanVar(value=False)
        self.playtime_value = customtkinter.StringVar(value="10 minutes")

        self.autopilot_status = customtkinter.BooleanVar(value=False) #StringVar

        self._isDroneSendingUpdates = False
        self._isDroneHasSentUpdates  =  False
        
        self._droneConnectionTimeToGCS = 0
        self._droneUpdatesDelay = 5
        self._droneUpdatesTimer = self._droneUpdatesDelay 

        #STORE VALUES

        self._droneGauge = None
        self._operatorGauge = None
        self._voltage = 0

        self._propulsion_status = None
        self._propulsion_level = 0
        self._steering = 0

        self._camera_azimuth = 90
        self._camera_tilt = 90

        self._latitude = 0
        self._longitude = 0
        self._azimuth = 0
        self._speed = 0

        self._pitch = 0
        self._roll = 0
        self._yaw =0

        self._obstacle = 0

        self._game_timer_enable = False
        self._playtime_left = 0
        self.playtime_nb =  10 * 60 

        self._corner_radius = 0
        self.pack(pady=10, padx=0, fill="both", expand=True)

        self.populate_frame()


    def _loop( self ):

        if( self._game_timer_enable ):
            
            if( self._playtime_left > 0 ):

                self._playtime_left -= 1
                self.update_time_left_label( )

            else:

                self.enable_game_timer( False )


        if( self._isDroneSendingUpdates is True ):
            self._update_connection_time()

        self._check_drone_status()


    def _check_drone_status(self):

        self._droneUpdatesTimer -= 1

        if( self._droneUpdatesTimer <= 0 ):
            
            self._droneUpdatesTimer = self._droneUpdatesDelay

            if not self._isDroneHasSentUpdates :
                self._isDroneSendingUpdates = False
                    
            self._isDroneHasSentUpdates  =  False

    def _update_connection_time( self ):
        # add total connection time
        self._droneConnectionTimeToGCS +=1

        time_delta = datetime.timedelta(seconds=self._droneConnectionTimeToGCS)
        formatted_time = str(time_delta).split(".")[0]
        self._connectionTime_label.configure( text= formatted_time )

    def populate_frame( self ): 
        
        self.configure_grid()
        self.add_name_label()
        self.add_autopilot_switch()
        self.add_propulsion_label()
        self.add_devices_label()
        self.add_droneGauge_progressbar()
        self.add_operatorGauge_progressbar()
        self.add_connection_time_labels( )
        self.add_operator_switch()
        self.add_playtime_dropdown()
        self.add_playtime_left_label( )


    def configure_grid(self):

        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=0)
        self.columnconfigure(2, weight=0)
        self.columnconfigure(3, weight=0)
        self.columnconfigure(4, weight=0)
        self.columnconfigure(5, weight=1)

        self.rowconfigure(0, weight=0)
        self.rowconfigure(1, weight=0)


    def add_name_label( self ):

        label = customtkinter.CTkLabel(
            self, 
            text=self._name, 
            width=100
        )

        label.configure(fg_color = self._shipColors[0])
        label.grid(row=0, rowspan=2, column=0, padx=10, pady=0, sticky="nsew")


    def add_autopilot_switch( self ):

        self._autopilot_switch  = customtkinter.CTkSwitch(
            master=self, 
            text=f"AUTOPILOT",
            command=self.onAutopilotSwitch,
            variable=self.autopilot_status, 
            onvalue=True,
            offvalue=False
            )
        
        self._autopilot_switch.deselect()

        self._autopilot_switch.grid(row=0,  column=1, padx=5, pady=0, sticky="nsew")

    def add_propulsion_label( self ):
        self._propulsion_status = customtkinter.CTkLabel(
            self,
            text=""
        )

        self._propulsion_status.grid(row=1, column=1, padx=5, pady=5, sticky="nsew")

        self.update_propulsion( 0 )


    def add_devices_label( self ):
       
        operator_label = customtkinter.CTkLabel(self, justify=customtkinter.LEFT, text="NAVISCOPE")
        operator_label.grid(row=0, column=2, padx=(10,0), pady=0, sticky="e") 

        drone_label = customtkinter.CTkLabel(self, justify=customtkinter.CENTER, text="DRONE")
        drone_label.grid(row=1, column=2, padx=(10,0), pady=0, sticky="e") 


    def add_droneGauge_progressbar( self ):
        self._droneGauge  = customtkinter.CTkProgressBar(self,
                                                    height = 10, 
                                                    orientation="horizontal")
        self._droneGauge.grid(row=0, column=3, padx=(5,10), pady=0, sticky="ew")

        self.update_gauge(100)


    def add_operatorGauge_progressbar( self ):
        self._operatorGauge  = customtkinter.CTkProgressBar(self,
                                                    height = 10, 
                                                    orientation="horizontal")
        self._operatorGauge.grid(row=1, column=3, padx=(5,10), pady=0, sticky="ew")

        self.update_operatorGauge(100)


    def add_connection_time_labels( self ):

        self._connectionTime_label = customtkinter.CTkLabel(
            self, 
            text="00:00:00",
            justify=customtkinter.LEFT
            )
        self._connectionTime_label.grid(row=0, column=4, padx=(5,10), pady=0, sticky="ns")

        self._connectionTimeOperator_label = customtkinter.CTkLabel(
            self, 
            text="00:00:00",
            justify=customtkinter.LEFT)
        self._connectionTimeOperator_label.grid(row=1, column=4, padx=(5,10), pady=0, sticky="ns")


    def add_playtime_dropdown( self ):
        self._playtime = customtkinter.CTkOptionMenu(
            self, 
            values=["10 minutes", "20 minutes"],
            variable=self.playtime_value,
            command=self.onPlayTimeChanged
        )
    
        self._playtime.grid(row=1, column=5, padx=(40,10), pady=(5,0), sticky="ew")


    def add_operator_switch( self ):

        self._operator_switch = customtkinter.CTkSwitch(
            master=self, 
            text="START A GAME",
            command=self.onOperatorSwitch,
            variable=self.operator_status, 
            onvalue=True,
            offvalue=False
        )

        self._operator_switch.grid(row=0, column=5, padx=(40,10), pady=0, sticky="ew")


    def add_playtime_left_label( self ):

        custom_font =("Times",20,'bold')

        self._time_left_label = customtkinter.CTkLabel(
            self, 
            text="", 
            font=custom_font,
            justify=customtkinter.CENTER )

        self._time_left_label.grid_forget()

    def update_time_left_label( self ):

        minutes = self._playtime_left // 60
        seconds = self._playtime_left % 60
        formatted_time = "{:02d}:{:02d}".format(minutes, seconds)

        self._time_left_label.configure( text=f" {formatted_time} ")
        

    def enable_game_timer( self, isEnable ):
            
        self._game_timer_enable = isEnable

        if( isEnable ):
            
            self._playtime_left = self.playtime_nb

            self._operator_switch.configure(text="STOP THE GAME")

            if( self._playtime.winfo_viewable()):
                self._playtime.grid_forget()

            self.update_time_left_label( )
            self._time_left_label.grid(row=0, rowspan=2, column=5, padx=0, pady=0, sticky="nsew")

            self._masterApp.onOperatorPlay( self._index, True )

        else:
            
            self._playtime_left = 0
            self._time_left_label.grid_forget()

            self._operator_switch.configure(text="START A GAME")

            self._playtime.set("10 minutes")
            self.adjust_playtime()

            self._playtime.grid(row=0, rowspan=2, column=5, padx=0, pady=0, sticky="nsew")

            self._masterApp.onOperatorPlay( self._index, False )


    def add_marker( self, latitude, longitude ):

        self._marker = self._map.set_marker(
            latitude,
            longitude,
            text= self._name,
            marker_color_circle=self._shipColors[0],
            marker_color_outside=self._shipColors[1]
            #command = self.display_update
        )


    def onPlayTimeChanged( self, value ):

        self.adjust_playtime()


    def adjust_playtime( self ):

        value = self.playtime_value.get()

        if( value == "20 minutes"):
            self.playtime_nb = 20 * 60
        elif( value == "30 minutes"):
            self.playtime_nb = 30 * 60
        else:
            self.playtime_nb =  10 * 60

    def onOperatorSwitch( self ):

        operator_state = self.operator_status.get()
        self.enable_game_timer(operator_state)

    def onAutopilotSwitch(self):

        autopilot_state = self.autopilot_status.get()

        if( autopilot_state is True):

            operator_state = self.operator_status.get()

            if operator_state is True:
                self._operator_switch.toggle()

            self._operator_switch.configure(state="disabled")

            print( "autopilot enable ")

        elif ( autopilot_state is False):
            self._operator_switch.configure(state="normal")
            print("autopilot disable")

    def update_propulsion( self, data ): 

        if( data == -1):

            self._propulsion_status.configure(text=DIRECTION_BWD[0])
            self._propulsion_status.configure(fg_color=DIRECTION_BWD[1])

        elif( data == 1 ):

            self._propulsion_status.configure(text=DIRECTION_FWD[0])
            self._propulsion_status.configure(fg_color=DIRECTION_FWD[1])

        else:
            self._propulsion_status.configure(text= DIRECTION_STP[0])
            self._propulsion_status.configure(fg_color=DIRECTION_STP[1])

    def update_gauge( self, data ):
        
        update_gauge = np.clip( data/100, 0, 1)

        self._droneGauge.set( update_gauge )

        if update_gauge > 0.5:

            if ( update_gauge >= 0.65 ):
                self._droneGauge.configure(progress_color =  "#47D600")
            else:
                self._droneGauge.configure(progress_color =  "#FFCD00")

        elif update_gauge <= 0.5:

            if( update_gauge > 0.3):
                self._droneGauge.configure(progress_color =  "#FF8000")
            else:
                self._droneGauge.configure(progress_color =  "#FF0000")

    def update_operatorGauge( self, data ):
        
        update_gauge = np.clip( data/100, 0, 1)

        self._operatorGauge.set( update_gauge )

        if update_gauge > 0.5:

            if ( update_gauge >= 0.65 ):
                self._operatorGauge.configure(progress_color =  "#47D600")
            else:
                self._operatorGauge.configure(progress_color =  "#FFCD00")

        elif update_gauge <= 0.5:

            if( update_gauge > 0.3):
                self._operatorGauge.configure(progress_color =  "#FF8000")
            else:
                self._operatorGauge.configure(progress_color =  "#FF0000")


    def onGaugeUpdate( self, data ):

        if( data is not None ) :
            self.update_gauge( data )

    def onVoltageUpdate( self, data ):

        if( data is not None):
            self._voltage = data
    
    def onDirectionUpdate( self, data ):

        if( data is not None):
            self.update_propulsion(data)

    def onThrustUpdate( self, data ):

        if data is not None:
            self._propulsion_level = data 

    def onSteeringUpdate( self, data ):

        if data is not None:
            self._steering = data 

    def onLatitudeUpdate( self, data ):
           
        if data is not None:
            self._latitude = data             
    
    def onLongitudeUpdate( self, data ):
           
        if data is not None:
            self._longitude = data  

    def onAzimuthUpdate( self, data ):
           
        if data is not None:
            self._azimuth = data                 

    def onSpeedUpdate( self, data ):
           
        if data is not None:
            self._speed = data  

    def onPanUpdate( self, data ):
           
        if data is not None:
            self._camera_azimuth = data  

    def onTiltUpdate( self, data ):
           
        if data is not None:
            self._camera_tilt = data  

    def onPitchUpdate( self, data ):
           
        if data is not None:
            self._pitch = data  

    def onRollUpdate( self, data ):
           
        if data is not None:
            self._roll = data  

    def onYawUpdate( self, data ):
           
        if data is not None:
            self._yaw = data  

    def onObstacleUpdate( self, data ):
           
        if data is not None:
            self._obstacle = data  

    def onPanUpdate( self, data ):
           
        if data is not None:
            self._camera_azimuth = data  

    def onTiltUpdate( self, data ):
           
        if data is not None:
            self._camera_tilt = data  

    def GPSUpdate( self ):
        
        if( self._latitude != 0 and self._longitude != 0 ):

            if( self._marker is None ):
                self.add_marker(self._latitude, self._longitude )
            else:
                self.update_position(self._latitude, self._longitude)
    
    def update_position( self, latitude, longitude ): 
        self._marker.set_position(latitude, longitude)
        #print("update ", latitude, " / ", longitude)

    def onSensorsUpdate( self, sensor_datas ):

        self._isDroneSendingUpdates = True
        self._isDroneHasSentUpdates = True

        for topic in sensor_datas:

            if( topic == SENSORS_TOPICS.BATTERY_GAUGE ) :
                self.onGaugeUpdate( sensor_datas[topic])

            elif(topic == SENSORS_TOPICS.BATTERY_VOLTAGE  ):
                self.onVoltageUpdate( sensor_datas[topic])
            
            elif(topic == SENSORS_TOPICS.DIRECTION  ):
                self.onDirectionUpdate( sensor_datas[topic])

            elif(topic == SENSORS_TOPICS.THRUST  ):
                self.onThrustUpdate( sensor_datas[topic])
            
            elif(topic == SENSORS_TOPICS.STEERING  ):
                self.onSteeringUpdate( sensor_datas[topic])

            elif(topic == SENSORS_TOPICS.LAT  ):
                self.onLatitudeUpdate( sensor_datas[topic])
            
            elif(topic == SENSORS_TOPICS.LON  ):
                self.onLongitudeUpdate( sensor_datas[topic])

            elif(topic == SENSORS_TOPICS.AZI  ):
                self.onAzimuthUpdate( sensor_datas[topic])
            
            elif(topic == SENSORS_TOPICS.SPEED  ):
                self.onSpeedUpdate( sensor_datas[topic])

            elif(topic == SENSORS_TOPICS.PITCH  ):
                self.onPitchUpdate( sensor_datas[topic])
            
            elif(topic == SENSORS_TOPICS.ROLL  ):
                self.onRollUpdate( sensor_datas[topic])

            elif(topic == SENSORS_TOPICS.YAW  ):
                self.onYawUpdate( sensor_datas[topic])
            
            elif(topic == SENSORS_TOPICS.OBSTACLE  ):
                self.onObstacleUpdate( sensor_datas[topic])

            elif(topic == SENSORS_TOPICS.CAM_PAN  ):
                self.onPanUpdate( sensor_datas[topic])

            elif(topic == SENSORS_TOPICS.CAM_TILT  ):
                self.onTiltUpdate( sensor_datas[topic])

        self.GPSUpdate( )



class DisplayWindow(customtkinter.CTk):

    APP_NAME = "MASTER CONTROL"
    WIDTH = 1280
    HEIGHT = 800
    GRID_SIZE = 5

    def __init__(
            self, 
            Master=None, 
            Names = ["Aucun", "Sovereign of the seas", "Daisy Jack", "Rei Pelluci", "Arjeroc", "Lady Idosia"],
            VERBOSE = False, 
            displayResolution = (320, 240) #120p resolution
        ):
        
        super().__init__()

        self._master = Master
        self._verbose = VERBOSE

        self._map = None
        self._master_location = [45.942346, -0.955582]
        self._names = Names
        
        self._navigation_marker = None

        self._drone_index = None

        self.title(DisplayWindow.APP_NAME)

        self.geometry(str( self.winfo_screenwidth()  ) + "x" + str(self.winfo_screenheight() ))
        self.minsize(DisplayWindow.WIDTH, DisplayWindow.HEIGHT)

        #self.attributes("-fullscreen", True) 

        self.canvaResolution = displayResolution 
        #self._video = None

        self.supervision_tab_name = "       ___ SUPERVISION ___       "
        self.control_tab_name = "       ___ PILOTAGE MANUEL ___       "

        self._sensor_index = 0
        self._drone_control_by_master = None

        self._tracked_drones = {}
        self._radios_btn_drones = []

        self._dialogBox = None

        self._default_thrust_range = 50
        self._active_thrust = self._default_thrust_range

        self.bind("<Control-c>", self._closing_from_gui)
        #self.protocol("WM_DELETE_WINDOW", self._closing_from_gui)

        self._frame = None
        self.last_frame = None
        self._is_frame_updated = False

        self._initialize()
        
    def _initialize( self ):

        #self.get_user_location()

        self._create_window()
        self._render_frame()
        self._loop()


    #def load_base_images( self ):
        #camera_64 = "base64Img"
        #self._camera_icon = base64.b64decode( camera_64 )


    def _start( self ):

        self.mainloop()


    def _loop( self ):

        if( self._tracked_drones is not None ):

            for drone in self._tracked_drones.values():
                drone._loop()
            
        self.after(1000, self._loop)#wait for 1 second


    def get_user_location(self):
        g = geocoder.ip('me')
        self._master_location = g.latlng
        sleep(2)
    
    def _create_window( self ): 

        for i in range( DisplayWindow.GRID_SIZE ):
            self.columnconfigure(i, weight=1)
            self.rowconfigure(i, weight=1)

        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=2)
        self.columnconfigure(3, weight=2)
        self.columnconfigure(4, weight=2)

        self.tabview = customtkinter.CTkTabview(self, corner_radius=0, command= self.onTabSelect )
        self.tabview.grid(row=0, rowspan =DisplayWindow.GRID_SIZE, column=0, columnspan = 3, padx=(5, 5), pady=(5, 5), sticky="nsew")
        
        self.tabview.add( self.supervision_tab_name )
        self.tabview.add( self.control_tab_name )

        self.supervision_tab = self.tabview.tab( self.supervision_tab_name )
        self.control_tab = self.tabview.tab( self.control_tab_name )

        self.create_map_panel()
        self.create_supervision_panel()
        self.create_control_panel()


    def onTabSelect( self ):

        active_tab = self.tabview.get()

        if ( active_tab != self.control_tab_name ):
            self._radios_btn_drones[0].invoke()


    def create_supervision_panel( self ):

        self.supervision_tab.columnconfigure(0, weight=0)
        self.supervision_tab.columnconfigure(1, weight=1)
        self.supervision_tab.columnconfigure(2, weight=0)
        
        self.supervision_tab.rowconfigure(0, weight=0)
        self.supervision_tab.rowconfigure(1, weight=1)
        self.supervision_tab.rowconfigure(2, weight=1)
        self.supervision_tab.rowconfigure(3, weight=1)
        self.supervision_tab.rowconfigure(4, weight=0)
   
       
        # drones list panel set up after map initialized
        self.scrollable_frame = customtkinter.CTkScrollableFrame(self.supervision_tab, label_text="")
        self.scrollable_frame.grid(row=0,rowspan=DisplayWindow.GRID_SIZE, column=0, columnspan = 4, padx=(0, 0), pady=(0, 0), sticky="nsew")
        self.scrollable_frame.grid_columnconfigure(0, weight=1)
        

        for i in range( len( self._names ) ):
            
            if( i == 0): continue
            else:
                drone_name = self._names[i][0]

                new_drone_frame = TrackedDrone(
                    self.scrollable_frame,
                    masterApp=self,
                    drone_index=i-1,
                    drone_name=drone_name,
                    shipColors = self._names[i][1],
                    map = self._map
                )

                new_drone_frame.pack(pady=10, padx=0, fill="both", expand=True )
                self._tracked_drones[f"drone_{i-1}"] = new_drone_frame

    def create_map_panel( self ):

         # Création du panneau de localisation

        map_panel = customtkinter.CTkFrame(self, corner_radius=0)
        map_panel.grid(row=0, rowspan=DisplayWindow.GRID_SIZE, column=3, columnspan=3, padx=0, pady=0, sticky="nsew")
        
        map_panel.grid_columnconfigure(0, weight=0) 
        map_panel.grid_columnconfigure(1, weight=1) 


        map_panel.grid_rowconfigure(0, weight=0)
        map_panel.grid_rowconfigure(1, weight=1)
        map_panel.grid_rowconfigure(2, weight=1) 
        map_panel.grid_rowconfigure(3, weight=1)
        map_panel.grid_rowconfigure(4, weight=0)
        
        self._map = TkinterMapView(map_panel, corner_radius=0)
        self._map.grid(row=1, rowspan=4, column=0, columnspan=2, sticky="nsew", padx=(0, 0), pady=(20, 0))
        
        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(
            map_panel, values=["Light", "Dark", "System"],
            command=self.change_appearance_mode
        )

        self.appearance_mode_optionemenu.grid(row=0, column=0, sticky="nsw", padx=(0, 0), pady=(10, 0))


        self.map_option_menu = customtkinter.CTkOptionMenu(
            map_panel, 
            values=["OpenStreetMap", "Google normal", "Google satellite"],
            command=self.change_map
        )
        self.map_option_menu.grid(row=0, column=1, padx=(5, 5),sticky="nsw", pady=(10, 0))


        self.set_map_events()


    def create_control_panel( self ):

        self.control_tab.columnconfigure(0, weight=1)
        self.control_tab.columnconfigure(1, weight=1)
        #self.control_tab.columnconfigure(2, weight=2)

        self.control_tab.rowconfigure(0, weight=1)
        self.control_tab.rowconfigure(1, weight=0)
        self.control_tab.rowconfigure(2, weight=1)
        self.control_tab.rowconfigure(3, weight=1)
        self.control_tab.rowconfigure(4, weight=1)
        self.control_tab.rowconfigure(5, weight=1)

        self.add_radio_list( self.control_tab )

        control_panel = customtkinter.CTkFrame( self.control_tab, corner_radius=0)
        control_panel.grid(row=0, rowspan=DisplayWindow.GRID_SIZE, column=1,  padx=0, pady=0, sticky="n")

        control_panel.grid_columnconfigure(0, weight=0)
        control_panel.grid_columnconfigure(1, weight=0)

        control_panel.grid_rowconfigure(0, weight=0)
        control_panel.grid_rowconfigure(1, weight=2)
        control_panel.grid_rowconfigure(2, weight=2)
        control_panel.grid_rowconfigure(3, weight=0)
        control_panel.grid_rowconfigure(4, weight=1)


        control_panel.configure(fg_color="transparent")

        self._shipColor_label = customtkinter.CTkLabel(
            control_panel,
            justify=customtkinter.LEFT,
            text=""
        )

        self._shipColor_label.grid(row=0, column=0, padx=(10,0), pady=(20, 5), sticky="nsew")
        self._shipColor_label.configure(fg_color="green")

        self._ip_label = customtkinter.CTkLabel(
            control_panel,
            justify=customtkinter.LEFT,
            text="192.168.0.144"
        )

        fpv_frame = self.add_fpv_frame(control_panel)

        battery_frame = self.add_battery_frame( control_panel )
        propulsion_frame = self.add_propulsion_frame(control_panel)
        gyro_frame = self.add_gyro_frame( control_panel ) 

        self._ip_label.grid(row=0, column=1, padx=(20,0), pady=(20, 5), sticky="nsw")

        fpv_frame.grid( row = 1, column =0 , columnspan = 2, padx=(0,0), pady=(10,0), sticky ="new")
        battery_frame.grid( row = 2, column =0, columnspan = 2, padx=(0,0), pady=(20,0), sticky ="new")
        propulsion_frame.grid( row = 3, column =0, columnspan = 2, padx=(0,0), pady=(20,0),  sticky ="new")
        gyro_frame.grid( row = 4, column =0, columnspan = 2, padx=(0,0), pady=(10,20),  sticky ="new")


    def add_radio_list( self, container ):
        # Création du panneau à gauche
        drone_panel = customtkinter.CTkFrame( container, corner_radius=0 ) #bg="lightgray"
        drone_panel.grid(row=0, column=0, rowspan=DisplayWindow.GRID_SIZE, padx=0, pady=0, sticky="nsew")

        # Ajout d'une liste de radios boutons au panneau de gauche permettant de passer
        # d'un drone à l'autre
        self.radio_var = tkinter.StringVar(value=self._names[0][0])
    
        for i in range( len( self._names ) ):
            
            drone_name = self._names[i][0]

            radio = customtkinter.CTkRadioButton(
                master=drone_panel, 
                variable=self.radio_var, 
                value=drone_name,
                command=self.onDroneRadioSelection
            )

            radio.configure( text= drone_name  )

            radio.grid(row=i+1, column=0, padx=(30, 5), pady=(20, 10), sticky="nsew")
            self._radios_btn_drones.append( radio )

        
    def add_fpv_frame( self, container ):

        fpv_container_frame = customtkinter.CTkFrame(container, corner_radius=0)

        fpv_frame = customtkinter.CTkFrame(fpv_container_frame, corner_radius=0)
        fpv_frame.pack( fill="x", padx=(10,10), pady=(10,10), expand=True )

        fpv_frame.grid_columnconfigure(0, weight=1)
        fpv_frame.grid_columnconfigure(1, weight=1)

        fpv_frame.grid_rowconfigure(0, weight=0)
        fpv_frame.grid_rowconfigure(1, weight=1)
        fpv_frame.grid_rowconfigure(2, weight=1)
        fpv_frame.grid_rowconfigure(3, weight=0)
        fpv_frame.grid_rowconfigure(4, weight=0)
        fpv_frame.grid_rowconfigure(5, weight=0)
        
        self.canvas = customtkinter.CTkCanvas(fpv_frame, width=self.canvaResolution[0], height=self.canvaResolution[1])
        self.canvas.grid(row=0, rowspan=6, column=0, padx=0, pady=0, sticky="ew")

        desc = customtkinter.CTkLabel(
            fpv_frame,
            justify=customtkinter.CENTER,
            text="OBSTACLES"
        )

        desc.grid(row=0, column=1, padx=(20,0), pady=(20,0), sticky="s")

        self._range_label = customtkinter.CTkLabel(
            fpv_frame,
            justify=customtkinter.LEFT,
            text="0 m"
        )

        self._range_label.grid(row=1, column=1, padx=(0,0), pady=(5,10), sticky="ew")

        self._pan_label = customtkinter.CTkLabel(master=fpv_frame, justify=customtkinter.CENTER, text="Azimuth")
        self._pan_label.grid(row=2, column=1, padx=0, pady=(20, 0), sticky="s")

        self._pan_slider = customtkinter.CTkSlider(master=fpv_frame, from_=0, to=180, number_of_steps=179)
        self._pan_slider.grid(row=3, column=1, padx=0, pady=(5, 10), sticky="ew")
        self._pan_slider.set(90)

        self._tilt_label = customtkinter.CTkLabel(master=fpv_frame, justify=customtkinter.CENTER, text="Inclinaison")
        self._tilt_label.grid(row=4, column=1, padx=0, pady=(20, 0), sticky="s")

        self._tilt_slider = customtkinter.CTkSlider(master=fpv_frame, from_=0, to=180, number_of_steps=179)
        self._tilt_slider.grid(row=5, column=1, padx=0, pady=(5, 20), sticky="ew")
        self._tilt_slider.set(90)

        return fpv_container_frame


    def add_propulsion_frame( self, container ):
        
        propulsion_frame = customtkinter.CTkFrame( container, corner_radius=0)

        propulsion_frame.grid_columnconfigure(0,weight=1)
        propulsion_frame.grid_columnconfigure(1,weight=0)
        propulsion_frame.grid_columnconfigure(2,weight=1)
        propulsion_frame.grid_columnconfigure(3,weight=0)

        propulsion_frame.grid_rowconfigure(0,weight=0)
        propulsion_frame.grid_rowconfigure(1,weight=0)
        propulsion_frame.grid_rowconfigure(2,weight=0)


        self._propulsion_label = customtkinter.CTkLabel(
            propulsion_frame,
            justify=customtkinter.LEFT,
            text="MARCHE AVANT"
        )

        self._propulsion_label.grid(row=0, rowspan=2, column=0, padx=5, pady=5, sticky="nsew")


        left_desc = customtkinter.CTkLabel(
            propulsion_frame,
            justify=customtkinter.LEFT,
            text="Gauche"
        )

        left_desc.grid(row=0, column=1, padx=(0,0), pady=(10,0), sticky="w")

        center_desc = customtkinter.CTkLabel(
            propulsion_frame,
            justify=customtkinter.CENTER,
            text="Centre"
        )

        center_desc.grid(row=0, column=2, padx=(0,10), pady=(10,0), sticky="nsew")

        right_desc = customtkinter.CTkLabel(
            propulsion_frame,
            justify=customtkinter.RIGHT,
            text="Droit"
        )

        right_desc.grid(row=0, column=3, padx=(0,10), pady=(10,0), sticky="e")

        self._direction_slider = customtkinter.CTkSlider(master=propulsion_frame, from_=-100, to=100, number_of_steps= 199)
        self._direction_slider.grid(row=1, column=1, columnspan=3, padx=(5,15), pady=(0,20), sticky="nsew")
        self._direction_slider.set(0)
    
        self._speed_label = customtkinter.CTkLabel(
            propulsion_frame,
            justify=customtkinter.CENTER,
            text="5.25 km/h"
        )

        self._speed_label.grid(row=2, column=0,  padx=(5,0), pady=(5,15), sticky="nsew")

        self._thrust_slider = customtkinter.CTkProgressBar(master=propulsion_frame, height=8) #, orientation='vertical'
        self._thrust_slider.grid(row=2, column=1, columnspan=3, padx=(5,15), pady=(5,15), sticky="ew")
        self._thrust_slider.set(0)

        return propulsion_frame
 

    def add_battery_frame( self, container ):
        
        battery_frame = customtkinter.CTkFrame( container, corner_radius=0)

        battery_frame.grid_columnconfigure(0,weight=0)
        battery_frame.grid_columnconfigure(1,weight=1)
        battery_frame.grid_columnconfigure(2,weight=1)
        battery_frame.grid_columnconfigure(3,weight=1)

        battery_frame.grid_rowconfigure(0,weight=0)
        battery_frame.grid_rowconfigure(1,weight=1)


        desc = customtkinter.CTkLabel(
            battery_frame,
            justify=customtkinter.CENTER,
            text="AUTONOMIE"
        )

        desc.grid(row=0, rowspan=2, column=0, columnspan=2, padx=(50,0), pady=(0,0), sticky="nsew")

        voltage_desc = customtkinter.CTkLabel(
            battery_frame,
            justify=customtkinter.LEFT,
            text="Tension de la batterie : "
        )

        voltage_desc.grid(row=0, column=2, padx=(70,20), pady=(5,5), sticky="ew")

        self._battery_voltage_label = customtkinter.CTkLabel(
            battery_frame,
            justify=customtkinter.LEFT,
            text="0.0 V"
        )

        self._battery_voltage_label.grid(row=0, column=3,  padx=(0,0), pady=(5,15), sticky="w")

        self._battery_gauge_slider = customtkinter.CTkProgressBar(master=battery_frame, height=10) #, orientation='vertical'
        self._battery_gauge_slider.grid(row=1, column=2, columnspan = 2, padx=(100,15), pady=(5,5), sticky="ew")
        self._battery_gauge_slider.set(0)

        return battery_frame
    
    def add_gyro_frame( self, container ):
        
        gyro_frame = customtkinter.CTkFrame( container, corner_radius=0)

        gyro_frame.grid_columnconfigure(0,weight=1)
        gyro_frame.grid_columnconfigure(1,weight=1)
        gyro_frame.grid_columnconfigure(2,weight=0)
        gyro_frame.grid_columnconfigure(3,weight=0)
        gyro_frame.grid_columnconfigure(4,weight=1)

        gyro_frame.grid_rowconfigure(0,weight=0)
        gyro_frame.grid_rowconfigure(1,weight=0)
        gyro_frame.grid_rowconfigure(2,weight=0)
        gyro_frame.grid_rowconfigure(3,weight=0)
        gyro_frame.grid_rowconfigure(4,weight=0)
        gyro_frame.grid_rowconfigure(5,weight=0)

        # >>>>>>>>>>>>>>>>> pitch
        pitch_desc = customtkinter.CTkLabel(
            gyro_frame,
            justify=customtkinter.LEFT,
            text="PITCH"
        )

        pitch_desc.grid(row=0, rowspan=2, column=0, columnspan=2, padx=0, pady=(5,0), sticky="nsew")

        
        pitch_l = customtkinter.CTkLabel(
            gyro_frame,
            justify=customtkinter.CENTER,
            text="Avant"
        )

        pitch_l.grid(row=0, column=2, padx=0, pady=(5,0), sticky="w")
        
        pitch_r = customtkinter.CTkLabel(
            gyro_frame,
            justify=customtkinter.CENTER,
            text="Arrière"
        )

        pitch_r.grid(row=0, column=4, padx=0, pady=(5,0), sticky="e")
        
        self._pitch_label = customtkinter.CTkLabel(
            gyro_frame,
            justify=customtkinter.CENTER,
            text=""
        )

        self._pitch_label.grid(row=0, column=3, padx=0, pady=(5,0), sticky="ew")

        self._pitch_slider = customtkinter.CTkSlider(master=gyro_frame, from_=-180, to=180, number_of_steps= 359)
        self._pitch_slider.grid(row=1, column=2, columnspan=3, padx=(5,15), pady=(0,0), sticky="nsew")
        self._pitch_slider.set(0)
    
        # >>>>>>>>>>>>>>>>> roll
        roll_desc = customtkinter.CTkLabel(
            gyro_frame,
            justify=customtkinter.LEFT,
            text="ROLL"
        )

        roll_desc.grid(row=2, rowspan=2, column=0,columnspan=2, padx=0, pady=(5,0), sticky="nsew")
        
        roll_l = customtkinter.CTkLabel(
            gyro_frame,
            justify=customtkinter.LEFT,
            text="Gauche"
        )

        roll_l.grid(row=2, column=2, padx=0, pady=(5,0), sticky="w")
        
        roll_r = customtkinter.CTkLabel(
            gyro_frame,
            justify=customtkinter.LEFT,
            text="Droite"
        )

        roll_r.grid(row=2, column=4, padx=0, pady=(5,0), sticky="e")

        self._roll_label = customtkinter.CTkLabel(
            gyro_frame,
            justify=customtkinter.CENTER,
            text=""
        )

        self._roll_label.grid(row=2, column=3, padx=0, pady=(5,0), sticky="ew")

        self._roll_slider = customtkinter.CTkSlider(master=gyro_frame, from_=-180, to=180, number_of_steps= 359)
        self._roll_slider.grid(row=3, column=2, columnspan=3, padx=(5,15), pady=(0,0), sticky="nsew")
        self._roll_slider.set(0)

        # >>>>>>>>>>>>>>>>> yaw
        yaw_desc = customtkinter.CTkLabel(
            gyro_frame,
            justify=customtkinter.LEFT,
            text="YAW"
        )

        yaw_desc.grid(row=4, rowspan=2, column=0,columnspan=2, padx=0, pady=(5,0), sticky="nsew")
        
        yaw_l = customtkinter.CTkLabel(
            gyro_frame,
            justify=customtkinter.LEFT,
            text="Est"
        )

        yaw_l.grid(row=4, column=2, padx=0, pady=(5,0), sticky="w")
        
        yaw_r = customtkinter.CTkLabel(
            gyro_frame,
            justify=customtkinter.LEFT,
            text="Ouest"
        )

        yaw_r.grid(row=4, column=4, padx=0, pady=(5,0), sticky="e")

        self._yaw_label = customtkinter.CTkLabel(
            gyro_frame,
            justify=customtkinter.CENTER,
            text=""
        )

        self._yaw_label.grid(row=4, column=3, padx=0, pady=(5,0), sticky="ew")

        self._yaw_slider = customtkinter.CTkSlider(master=gyro_frame, from_=-180, to=180, number_of_steps= 359)
        self._yaw_slider.grid(row=5, column=2, columnspan=3, padx=(5,15), pady=(0,0), sticky="nsew")
        self._yaw_slider.set(0)

        return gyro_frame

    def onDroneRadioSelection( self ):

        drone_index = -1
        radio_value = self.radio_var.get()

        for i in range( len( self._names ) ):
            
            if( self._names[i][0] == radio_value ): 
                drone_index +=1
                break

        self._master._on_gui_drone_index_change( drone_index )

        self._sensor_index = drone_index - 1

        if( self._sensor_index >= 0):

            self._drone_control_by_master = self._tracked_drones[f"drone_{self._sensor_index}"]

            if( self._drone_control_by_master is not None ):
                
                drone = self._drone_control_by_master
                autopilot_state = drone.autopilot_status.get()

                if autopilot_state is True:
                    drone._autopilot_switch.deselect()

                operator_state = drone.operator_status.get()

                if operator_state is True:
                    drone._operator_switch.toggle()
                
                self.update_drone_control_by_master( self )

        else:
            
            self._drone_control_by_master = None

            self._shipColor_label.configure( fg_color="transparent")
            self._propulsion_label.configure(text=DIRECTION_STP[0])
            self._propulsion_label.configure(fg_color=DIRECTION_STP[1])


    def onOperatorPlay( self, index, enable ):
        self._master._node._enable_gameplay( index, enable )


    def OnDronesDatas( self, index, data ):

        drone = self._tracked_drones[f"drone_{index}"]

        if( drone is not None):
            drone.onSensorsUpdate( data )


    def set_map_events( self ): 

        if( self._map is not None ): 
            
            #self._map.set_address("quai Bellot, Rochefort, Charente-Maritime, France")
            self._map.set_position(self._master_location[0], self._master_location[1])
            self._master_location
            self.map_option_menu.set("OpenStreetMap")
            self.appearance_mode_optionemenu.set("Dark")

            self._map.add_right_click_menu_command(label="Placer une cible de navigation",
                                        command=self.add_marker_event,
                                        pass_coords=True)


    def add_marker_event(self, coords):
        
        if( self._navigation_marker is not None ):

            self._navigation_marker.set_position(
                coords[0], 
                coords[1]
            )

        else:

            self._navigation_marker = self._map.set_marker(
                coords[0], 
                coords[1], 
                text="Cible"
            )
    

    def clear_map( self ):
        self._map.delete_all_marker()
    
    def change_appearance_mode(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)

    def change_map(self, new_map: str):
        if new_map == "OpenStreetMap":
            self._map.set_tile_server("https://a.tile.openstreetmap.org/{z}/{x}/{y}.png")
        elif new_map == "Google normal":
            self._map.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        elif new_map == "Google satellite":
            self._map.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)


    def update_drone_control_by_master( self ):

        if self._drone_control_by_master is not None:
        
            drone = self._drone_control_by_master

            self._shipColor_label.configure( fg_color=drone._shipColors[0] )

            self._set_obstacle_range( drone )
            self._set_pan( drone )
            self._set_tilt( drone )
            self._set_gauge( drone )
            self._set_propulsion( drone )
            self._set_imu( drone )


    def _set_obstacle_range( self, drone ):
        meter = drone._obstacle / 100
        self._range_label.configure(text = f"{meter:.2f}m")

    def _set_pan( self, drone ):
        self._pan_slider.set( drone._camera_azimuth )
        self._pan_label.configure(text = f"Azimuth: {math.floor(drone._camera_azimuth)}°" )

    def _set_tilt( self, drone ):
        self._tilt_slider.set( drone._camera_tilt )
        self._tilt_label.configure(text = f"Inclinaison: {math.floor(drone._camera_tilt)}°" )

    def _set_gauge(self, drone ):
        self._battery_voltage_label .configure(text = f"Tension: {drone._voltage} V" )
        self._battery_gauge_slider.set( drone._droneGauge )

    def _set_propulsion( self, drone ):
        self._propulsion_label.configure(text=drone._propulsion_status.cget("text"))
        self._propulsion_label.configure(fg_color=drone._propulsion_status.cget("fg_color"))
        
        self._thrust_slider.set( drone._propulsion_level )

        self._direction_slider.set(drone._steering )
        self._speed_label.configure(text = f"{drone._speed } km/h" )

    def _set_imu( self, drone ):

        self._pitch_label.configure(text=f"{drone._pitch}°" )
        self._pitch_slider.set( drone._pitch )

        self._roll_label.configure(text=f"{drone._roll}°" )
        self._roll_slider.set(drone._roll)

        self._yaw_label.configure(text=f"{drone._yaw}°" )
        self._yaw_slider.set(drone._yaw)


    def _update_frame( self, frame ):

        if( frame is not None ):
            
            #np.asarray(frame),
            resized_frame = cv2.resize( frame, (self.canvas.winfo_height(), self.canvas.winfo_width() ))
            color_conv = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)

            img = Image.fromarray(color_conv)
            img_canvas = ImageTk.PhotoImage(img)

            self._frame = img_canvas
            self._is_frame_updated = True
            
    
    def _render_frame(self):
        
        if self._is_frame_updated is True:

            if self.last_frame != self._frame:

                self.last_image = self._frame
                self.canvas.delete("all")
                self.canvas.create_image(0, 0, anchor="nw", image=self.last_image)
            

        self._is_frame_updated = False
        # schedule the next update
        self.after(1, self._render_frame)


    def _open_popup(self):
        self._dialogBox = customtkinter.CTkToplevel(self)
        self._dialogBox .geometry("300x150")
        self._dialogBox .title("Exit Window")

        exit_frame = customtkinter.customtkinter.CTkFrame( master= self._dialogBox  )
        exit_frame.pack( pady=20, padx=60, fill="both", expand=True )
        
        customtkinter.customtkinter.CTkLabel(exit_frame, text= "Quelle action pour les drones ?").pack(pady=10, padx=10)
        exit_segmented_button = customtkinter.CTkSegmentedButton(master=exit_frame, values=[ EXIT_STATE.SHUTDOWN.value, EXIT_STATE.RESTART.value ], command = self._action_on_shutdown)
        
        exit_segmented_button.pack(pady=10, padx=10)

    def _stop(self):
        
        #sleep(2)
        self.destroy()


    def _action_on_shutdown(self, value):
        
        self._dialogBox.destroy()

        if self._master is not None:

            self._master._on_shutdown_selection( value )

        else:
            
            self._stop()


    def _closing_from_gui(self, event):
        
        if self._master is not None:
            self._open_popup()
        else:
            self._stop()

    



def main(args=None):

    app =  DisplayWindow()

    try:

        app._start()

    except Exception as exception:
        print( "an exception has been raised while spinning the movement node : ", exception)

    finally:

        app._stop()

        

if __name__ == '__main__':
    main()

    