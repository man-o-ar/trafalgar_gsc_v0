#!/usr/bin/env python
import datetime
import numpy as np
from typing import Tuple

import customtkinter

from ..utils.__utils_objects import SENSORS_TOPICS, DIRECTION_STP, DIRECTION_FWD, DIRECTION_BWD

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

        self._isOperatorSendingUpdates = False
        self._isOperatorHasSentUpdates = False
        self._operatorConnectionTimeToGCS = 0
        self._operatorUpdatesDelay = 5
        self._operatorUpdatesTimer = self._operatorUpdatesDelay 
        
        #STORE VALUES

        self._droneGauge = 0
        self._operatorGauge = 0
        self._voltage = 0
        self._operatorVoltage = 0

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
                
                self._operator_switch.toggle()
                self.enable_game_timer( False )

        self._update_connection_time()
        self._update_operator_connection_time( )

        if self._isOperatorSendingUpdates is False:
            self._reset_operator_widgets()

        if self._isDroneSendingUpdates is False: 
            self._reset_widgets()
    
        self._check_drone_status()
        self._check_operator_status()

   

    def _check_drone_status(self):

        self._droneUpdatesTimer -= 1

        if( self._droneUpdatesTimer <= 0 ):
            
            self._droneUpdatesTimer = self._droneUpdatesDelay

            if not self._isDroneHasSentUpdates :
                self._isDroneSendingUpdates = False
                    
            self._isDroneHasSentUpdates  =  False

    def _check_operator_status(self):

        self._operatorUpdatesTimer -= 1

        if( self._operatorUpdatesTimer <= 0 ):
            
            self._operatorUpdatesTimer = self._operatorUpdatesDelay

            if not self._isOperatorHasSentUpdates :
                self._isOperatorSendingUpdates = False
                    
            self._isOperatorHasSentUpdates  =  False

    def _update_connection_time( self ):
        # add total connection time
        self._droneConnectionTimeToGCS +=1
        
        if self._isDroneSendingUpdates is False: 
            self._droneConnectionTimeToGCS = 0

        time_delta = datetime.timedelta(seconds=self._droneConnectionTimeToGCS)
        formatted_time = str(time_delta).split(".")[0]
        
        if self._connectionTime_label is not None : self._connectionTime_label.configure( text= formatted_time )


    def _update_operator_connection_time( self ):
        
        self._operatorConnectionTimeToGCS +=1
        
        if self._isOperatorSendingUpdates is False :
            self._operatorConnectionTimeToGCS = 0

        time_delta = datetime.timedelta(seconds=self._operatorConnectionTimeToGCS)
        formatted_time = str(time_delta).split(".")[0]
        
        if self._connectionTimeOperator_label is not None : self._connectionTimeOperator_label.configure( text= formatted_time )


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
        self._drone_battery_gauge = customtkinter.CTkProgressBar(self,
                                                    height = 10, 
                                                    orientation="horizontal")
        self._drone_battery_gauge.grid(row=1, column=3, padx=(5,10), pady=0, sticky="ew")

        self.update_gauge(100)


    def add_operatorGauge_progressbar( self ):
        self._operator_battery_gauge  = customtkinter.CTkProgressBar(self,
                                                    height = 10,
                                                    orientation="horizontal")
        self._operator_battery_gauge.grid(row=0, column=3, padx=(5,10), pady=0, sticky="ew")

        self.update_operatorGauge(100)


    def add_connection_time_labels( self ):

        self._connectionTime_label = customtkinter.CTkLabel(
            self, 
            text="00:00:00",
            justify=customtkinter.LEFT
            )
        self._connectionTime_label.grid(row=1, column=4, padx=(5,10), pady=0, sticky="ns")

        self._connectionTimeOperator_label = customtkinter.CTkLabel(
            self, 
            text="00:00:00",
            justify=customtkinter.LEFT)
        self._connectionTimeOperator_label.grid(row=0, column=4, padx=(5,10), pady=0, sticky="ns")


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

        if self._time_left_label is not None : self._time_left_label.configure( text=f" {formatted_time} ")
        

    def enable_game_timer( self, isEnable ):
            
        self._game_timer_enable = isEnable

        if( isEnable ):
            
            self._playtime_left = self.playtime_nb

            if self._operator_switch is not None : self._operator_switch.configure(text="STOP THE GAME")

            if( self._playtime.winfo_viewable()):
                self._playtime.grid_forget()

            self.update_time_left_label( )
            self._time_left_label.grid(row=1, column=5, padx=(40,10), pady=(5,0), sticky="ew")

            self._masterApp.onOperatorPlay( self._index, True, self.playtime_nb  )

        else:
            
            self._playtime_left = 0
            self._time_left_label.grid_forget()

            if self._operator_switch is not None : self._operator_switch.configure(text="START A GAME")

            self._playtime.set("10 minutes")
            self.adjust_playtime()

            self._playtime.grid(row=1, column=5, padx=(40,10), pady=(5,0), sticky="ew")

            self._masterApp.onOperatorPlay( self._index, False, self.playtime_nb )


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

            if self._operator_switch is not None: self._operator_switch.configure(state="disabled")

            print( "autopilot enable ")

        elif ( autopilot_state is False):
            if self._operator_switch is not None : self._operator_switch.configure(state="normal")
            print("autopilot disable")

    def update_propulsion( self, data ): 

        if self._propulsion_status is not None: 
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
        
        update_gauge = np.clip( data, 0, 100) / 100
        self._droneGauge = update_gauge

        if self._drone_battery_gauge is not None:

            self._drone_battery_gauge.set( update_gauge )


            if update_gauge > 0.50:

                if ( update_gauge >= 0.65 ):
                    self._drone_battery_gauge.configure(progress_color =  "#47D600")
                else:
                    self._drone_battery_gauge.configure(progress_color =  "#FFCD00")

            elif update_gauge <= 0.50:

                if( update_gauge > 0.30):
                    self._drone_battery_gauge.configure(progress_color =  "#FF8000")
                else:
                    self._drone_battery_gauge.configure(progress_color =  "#FF0000")

    def update_operatorGauge( self, data ):
        
        update_gauge = np.clip( data, 0, 100) /100

        if self._operator_battery_gauge is not None: 
            self._operator_battery_gauge.set( update_gauge )

            if update_gauge > 0.5:

                if ( update_gauge >= 0.65 ):
                    self._operator_battery_gauge.configure(progress_color =  "#47D600")
                else:
                    self._operator_battery_gauge.configure(progress_color =  "#FFCD00")

            elif update_gauge <= 0.5:

                if( update_gauge > 0.3):
                    self._operator_battery_gauge.configure(progress_color =  "#FF8000")
                else:
                    self._operator_battery_gauge.configure(progress_color =  "#FF0000")


    def onGaugeUpdate( self, data, isDrone = True ):

        if( data is not None ) :
            if isDrone is True:
                self.update_gauge( data )
            else:
                self.update_operatorGauge( data )


    def onVoltageUpdate( self, data, isDrone = True ):

        if( data is not None):
            if isDrone is True:
                self._voltage = data
            else:
                self._operatorVoltage = data

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

    def onIPUpdate( self, data ):
           
        if data is not None:
            self._ip = data  

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

            elif(topic == SENSORS_TOPICS.IP ):
                self.onIPUpdate( sensor_datas[topic])
     

        #self.GPSUpdate( )

    def onOperatorUpdate( self, sensor_datas ):

        self._isOperatorSendingUpdates = True
        self._isOperatorHasSentUpdates = True

        for topic in sensor_datas:

            if( topic == SENSORS_TOPICS.BATTERY_GAUGE ) :
                self.onGaugeUpdate( sensor_datas[topic], False )

            elif(topic == SENSORS_TOPICS.BATTERY_VOLTAGE  ):
                self.onVoltageUpdate( sensor_datas[topic], False )
            

    def _reset_widgets( self ):
        
        self.onGaugeUpdate( 0, True )
        self.onVoltageUpdate( 0, True )
    
        self._droneGauge = 0
        self._operatorGauge = 0
        self._voltage = 0
        self._operatorVoltage = 0

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


    def _reset_operator_widgets(self):
        self.onGaugeUpdate( 0, False )
        self.onVoltageUpdate( 0, False )