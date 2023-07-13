#!/usr/bin/env python
import math
import os
from time import sleep
import cv2 # OpenCV library
#import numpy as np
import tkinter
import customtkinter
from tkintermapview import TkinterMapView
from PIL import ImageTk, Image

customtkinter.set_appearance_mode("dark")  # Modes: system (default), light, dark
customtkinter.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

import geocoder

from ..utils.__utils_objects import EXIT_STATE, DIRECTION_STP
from .__trackedDroneFrame import TrackedDrone

    
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
        self._active_tab = None
        
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
        self._blackScreen is False

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

        if self._active_tab is None :
            self._active_tab = self.tabview.get()

        if ( self._active_tab == self.control_tab_name ):
            self.update_drone_control_by_master()

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

        self._active_tab = self.tabview.get()

        if ( self._active_tab != self.control_tab_name ):
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
        #self._shipColor_label.configure(fg_color="green")

        self._ip_label = customtkinter.CTkLabel(
            control_panel,
            justify=customtkinter.LEFT,
            text=""
        )
        self._ip_label.grid(row=0, column=1, padx=(20,0), pady=(20, 5), sticky="nsw")

        fpv_frame = self.add_fpv_frame(control_panel)

        battery_frame = self.add_battery_frame( control_panel )
        propulsion_frame = self.add_propulsion_frame(control_panel)
        gyro_frame = self.add_gyro_frame( control_panel ) 


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
            text=""
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

        self.drawBlackScreen()
        
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

        self._steering_angle_label = customtkinter.CTkLabel(
            propulsion_frame,
            justify=customtkinter.CENTER,
            text="Centre"
        )

        self._steering_angle_label.grid(row=0, column=2, padx=(0,10), pady=(10,0), sticky="nsew")

        right_desc = customtkinter.CTkLabel(
            propulsion_frame,
            justify=customtkinter.RIGHT,
            text="Droit"
        )

        right_desc.grid(row=0, column=3, padx=(0,10), pady=(10,0), sticky="e")

        self._direction_slider = customtkinter.CTkSlider(master=propulsion_frame, from_=0, to=180, number_of_steps= 179)
        self._direction_slider.grid(row=1, column=1, columnspan=3, padx=(5,15), pady=(0,20), sticky="nsew")
        self._direction_slider.set(90)
    
        self._speed_label = customtkinter.CTkLabel(
            propulsion_frame,
            justify=customtkinter.CENTER,
            text=""
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

        desc.grid(row=0, column=0, columnspan=2, padx=(50,0), pady=(0,0), sticky="nsew")

        self._autonomy_left_label = customtkinter.CTkLabel(
            battery_frame,
            justify=customtkinter.CENTER,
            text=""
        )

        self._autonomy_left_label.grid(row=1, column=0, columnspan=2, padx=(50,0), pady=(0,0), sticky="nsew")

        voltage_desc = customtkinter.CTkLabel(
            battery_frame,
            justify=customtkinter.LEFT,
            text="Tension de la batterie : "
        )

        voltage_desc.grid(row=0, column=2, padx=(70,20), pady=(5,5), sticky="ew")

        self._battery_voltage_label = customtkinter.CTkLabel(
            battery_frame,
            justify=customtkinter.LEFT,
            text=""
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
        gyro_frame.grid_columnconfigure(3,weight=1)
        gyro_frame.grid_columnconfigure(4,weight=0)

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

        drone_index = 0
        radio_value = self.radio_var.get()

        for i in range( len( self._names ) ):
            
            if( self._names[i][0] == radio_value ): 
                break
            drone_index +=1


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

                #if operator_state is True:
                #    drone._operator_switch.toggle()
                
                self.update_drone_control_by_master()

        else:
            
            self._drone_control_by_master = None
            
            self.canvas.delete("all")
            self.canvas.configure(bg='white')

            self._shipColor_label.configure( fg_color="transparent")
            self._propulsion_label.configure(text=DIRECTION_STP[0])
            self._propulsion_label.configure(fg_color=DIRECTION_STP[1])

            self._range_label.configure( text="0 m")
            self._tilt_slider.set(90)
            self._pan_slider.set(90)
            self._autonomy_left_label.configure(text="")
            self._speed_label.configure(text="")
            self._battery_voltage_label.configure(text="")
            self._battery_gauge_slider.set(0)
            self._direction_slider.set(90)
            self._thrust_slider.set(0)
            self._pitch_slider.set(0)
            self._roll_slider.set(0)
            self._yaw_slider.set(0)



    def onOperatorPlay( self, index, enable, playtime ):
        self._master._node._enable_gameplay( index, enable, playtime )


    def OnDronesDatas( self, index, data ):

        drone = self._tracked_drones[f"drone_{index}"]

        if( drone is not None):
            drone.onSensorsUpdate( data )

    def OnOperatorDatas( self, index, data ):

        drone = self._tracked_drones[f"drone_{index}"]

        if( drone is not None):
            drone.onOperatorUpdate( data )
            

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
            self._set_ip( drone )


    def _set_obstacle_range( self, drone ):

        if drone._obstacle is not None: 
            meter = drone._obstacle / 100
            self._range_label.configure(text = f"{meter:.2f}m")

    def _set_pan( self, drone ):
        
        if(drone._camera_azimuth is not None ):
            self._pan_slider.set( drone._camera_azimuth )
            self._pan_label.configure(text = f"Azimuth: {math.floor(drone._camera_azimuth)}°" )

    def _set_tilt( self, drone ):
        
        if(drone._camera_tilt is not None ):
            self._tilt_slider.set( drone._camera_tilt )
            self._tilt_label.configure(text = f"Inclinaison: {math.floor(drone._camera_tilt)}°" )

    def _set_gauge(self, drone ):
        
        if( drone._droneGauge is not None ):
            self._autonomy_left_label.configure(text = "{:.2f} %".format(drone._droneGauge * 100))
            self._battery_voltage_label.configure(text = f"Tension: {drone._voltage} V" )
            self._battery_gauge_slider.set( drone._droneGauge )


    def _set_propulsion( self, drone ):

        if drone._propulsion_status is not None:
            self._propulsion_label.configure(text=drone._propulsion_status.cget("text"))
            self._propulsion_label.configure(fg_color=drone._propulsion_status.cget("fg_color"))
        
        if drone._propulsion_level is not None:
            self._thrust_slider.set( drone._propulsion_level / 100 )

        if drone._steering is not None:
            self._direction_slider.set(drone._steering )

            delta = drone._steering - 90
            steering_info = f"{ delta }°"
            
            self._steering_angle_label.configure(text = steering_info )
            self._speed_label.configure(text = f"{drone._speed } km/h" )
            

    def _set_imu( self, drone ):

        if drone._pitch is not None:
            self._pitch_label.configure(text=f"{drone._pitch}°" )
            self._pitch_slider.set( drone._pitch )

        if drone._roll is not None:
            self._roll_label.configure(text=f"{drone._roll}°" )
            self._roll_slider.set(drone._roll)

        if drone._yaw is not None:
            self._yaw_label.configure(text=f"{drone._yaw}°" )
            self._yaw_slider.set(drone._yaw)

    def _set_ip( self, drone ):
        if drone._ip is not None:
            self._ip_label.configure(text=drone._ip )


    def _update_frame( self, frame ):

        if( frame is not None ):
            
            #np.asarray(frame),
            resized_frame = cv2.resize( frame, ( self.canvas.winfo_width(), self.canvas.winfo_height() ))
            color_conv = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)

            img = Image.fromarray(color_conv)
            img_canvas = ImageTk.PhotoImage(img)

            self._frame = img_canvas
            self._is_frame_updated = True


    def drawBlackScreen( self ):

        if self._blackScreen is False: 

            self.canvas.update_idletasks()
            self.canvas.delete("all")

            self.canvas.create_rectangle(0, 0, self.canvas.winfo_width(), self.canvas.winfo_height(), fill="black")

            self._blackScreen = True 


    def _render_frame(self):
        
        if self._is_frame_updated is True:

            if self.last_frame != self._frame:

                self.last_image = self._frame
                self.canvas.delete("all")
                self.canvas.create_image(0, 0, anchor="nw", image=self.last_image)
        else:   
            self.drawBlackScreen()

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

    