#!/usr/bin/env python
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication,QMainWindow,QWidget,QAction,QLineEdit,QComboBox,QMessageBox
from PyQt5 import QtWidgets
from PyQt5.QtCore import *


import sys
import glob
import argparse
import json

import carla
import re

#-------for plots------------------
from PyQt5.QtWidgets import QFileDialog,QVBoxLayout
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg as FigureCanvas,
    NavigationToolbar2QT as NavigationToolbar
)
from matplotlib.figure import Figure

import pandas as pd
import sys
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("carla_control.ui",self)
        self.actors_ids=[]
        self.hero_vehicle=None
        self.vehicle_id=None
        self.vehicle_name=None
        self.vehicle_model=None
        self.vehicle_make=None
        self.steering_curve_list=[] #list of tupples of (x,y) points
        self.torq_curve_list=[]
        
        # menu bar buttons------------------
        self.current_file=None
        self.actionSave.triggered.connect(self.save_file)
        self.actionSave_As.triggered.connect(self.save_file_as)
        self.actionOpen.triggered.connect(self.open_file)
        self.actionNew.triggered.connect(self.new_file)
        self.actionExit.triggered.connect(self.exit)


        # Create figures and canvases---------------------
        self.figure_torq = Figure()
        self.canvas_torq = FigureCanvas(self.figure_torq)

        self.figure_steer = Figure()
        self.canvas_steer = FigureCanvas(self.figure_steer)

         # Add canvases to UI frames
        layout_torq = QVBoxLayout(self.VFRM_Eng_torq)
        layout_torq.addWidget(self.canvas_torq)

        layout_steer = QVBoxLayout(self.VFRM_body_steer)
        layout_steer.addWidget(self.canvas_steer)

        self.df = None
        
        self.current_vehilcle_physics_control=carla.VehiclePhysicsControl() #it will contain all temp vehichle physics changes 
        
        self.current_weheels=[] # it will save all the temp wheel modifications
        self.current_wheel=carla.WheelPhysicsControl() # save temp wheel changes

        self.current_forward_gears=[]
        self.current_froward_gear=carla.GearPhysicsControl() 



        # Connect buttons and pass target plots
        self.VEPB_torque.clicked.connect(lambda: self.open_file_dialog(self.figure_torq, self.canvas_torq))
        self.VBPB_sc.clicked.connect(lambda: self.open_file_dialog(self.figure_steer, self.canvas_steer))

        # Connect plot clicks
        self.canvas_torq.mpl_connect("button_press_event", self.on_torq_clicked)
        self.canvas_steer.mpl_connect("button_press_event", self.on_steer_clicked)

        #-------------------------------------------------
        self.connect_server('localhost',2000)
        
       
        
        self.on_tab_changes()
        

        self.VPB_apply.clicked.connect(self.apply_changes)
        self.VPB_save_wheel.clicked.connect(self.save_wheel)
        self.VPB_save_gear.clicked.connect(self.save_gear)
        
        self.VCB_id.activated.connect(self.fetch_vehicle_details)
        self.VCB_wheels.activated.connect(self.fetch_wheel_details)
        self.VCB_gears.activated.connect(self.fetch_gear_details)
        
        
        self.WPB_set_default.clicked.connect(self.set_weather_default)
        self.WPB_apply.clicked.connect(self.apply_weather_details)

        #---weather sliders---------------------
        self.set_weather_default()
        self.WHS_cloudiness.valueChanged.connect(self.WLCD_cloudiness.display)
        self.WHS_precipitation.valueChanged.connect(self.WLCD_precipitation.display)
        self.WHS_precipitation_deposits.valueChanged.connect(self.WLCD_precipitation_deposits.display)
        self.WHS_wind_intensity.valueChanged.connect(self.WLCD_wind_intensity.display)
        self.WHS_sun_azimuth_angle.valueChanged.connect(self.WLCD_sun_azimuth_angle.display)
        self.WHS_sun_altitude_angle.valueChanged.connect(self.WLCD_sun_altitude_angle.display)
        self.WHS_fog_density.valueChanged.connect(self.WLCD_fog_density.display)
        self.WHS_fog_distance.valueChanged.connect(self.WLCD_fog_distance.display)
        self.WHS_wetness.valueChanged.connect(self.WLCD_wetness.display)
        self.WHS_fog_falloff.valueChanged.connect(self.WLCD_fog_falloff.display)
        self.WHS_scattering_intensity.valueChanged.connect(self.WLCD_scattering_intensity.display)
        self.WHS_mie_scattering_scale.valueChanged.connect(self.WLCD_mie_scattering_scale.display)
        self.WHS_rayleigh_scattering_scale.valueChanged.connect(self.WLCD_rayleigh_scattering_scale.display)
        self.WHS_dust_storm.valueChanged.connect(self.WLCD_dust_storm.display)
       


       
        self.TABS.currentChanged.connect(self.on_tab_changes)
        


    def connect_server(self,hostname,port):
        client=carla.Client(hostname,port)
        client.set_timeout(5.0)
        self.world=client.get_world()
   
        
        
        

        
        
        '''for actor in self.world.get_actors().filter('vehicle.*'):
            
            if actor.attributes.get('role_name') == 'hero':
                self.hero_vehicle = actor
                break
        if self.hero_vehicle is None:
            raise RuntimeError("No vehicle with role_name 'hero' found")
        else:
            print(f"Found hero vehicle: ID={self.hero_vehicle.id}, type={self.hero_vehicle.type_id}, {type(self.hero_vehicle.id)}")'''
        
    def on_tab_changes(self):
        tab_id=int(self.TABS.currentIndex())
        print(f'current tab {tab_id}')

        if tab_id==0:
            self.get_actors()
            self.VCB_id.clear()
            self.VCB_id.addItems(self.actor_ids)
        elif tab_id==1:
            self.fetch_weather_details()

    def get_actors(self):
        self.actor_ids=[] #clean previous ones
        for actor in self.world.get_actors().filter('vehicle.*'):
            self.actor_ids.append(str(actor.id))

    def apply_changes(self):
        #connect to server
        try:
            id=int(self.VCB_id.currentText())
            print(f'selected id {id}')
            #acces phyics of ego vhielce
            self.selected_actor=self.world.get_actor(id)
            #physics = self.selected_actor.get_physics_control()
            

            
            #-----Engine-----------------    
            if self.torq_curve_list is not None:
                self.torq_curve = [carla.Vector2D(x, y) for x, y in self.torq_curve_list]
                self.current_vehilcle_physics_control.torque_curve=self.torq_curve
            
            self.current_vehilcle_physics_control.vehicle_model=self.VLE_model.text()
            self.current_vehilcle_physics_control.vehicle_make=self.VLE_maker.text()
            self.current_vehilcle_physics_control.max_rpm=float(self.VELE_mrpm.text())
            self.current_vehilcle_physics_control.moi=float(self.VELE_moi.text())
            self.current_vehilcle_physics_control.damping_rate_full_throttle=float(self.VELE_drft.text())
            self.current_vehilcle_physics_control.damping_rate_zero_throttle_clutch_engaged=float(self.VELE_drztc.text())
            self.current_vehilcle_physics_control.damping_rate_zero_throttle_clutch_disengaged=float(self.VELE_drztnc.text())

            #----body------------
            #self.VBLE_sc.setText(str(self.steering_curve))
            if self.steering_curve_list is not None:
                self.steering_curve = [carla.Vector2D(x, y) for x, y in self.steering_curve_list]
                self.current_vehilcle_physics_control.steering_curve=self.steering_curve
            
            self.current_vehilcle_physics_control.mass=float(self.VBPB_ms.text())
            self.current_vehilcle_physics_control.drag_coefficient=float(self.VBPB_dc.text())
            coords = [float(v) for v in re.findall(r"[-+]?\d*\.\d+|\d+", self.VBPB_com.text())]
            self.current_vehilcle_physics_control.center_of_mass=carla.Vector3D(*coords)
            self.current_vehilcle_physics_control.use_sweep_wheel_collision=bool(self.VBPB_swc.text())

            #----Transmission-----------------
            self.current_vehilcle_physics_control.use_gear_autobox=bool(self.VTLE_agb.text())
            self.current_vehilcle_physics_control.gear_switch_time=float(self.VTLE_gst.text())
            self.current_vehilcle_physics_control.clutch_strength=float(self.VTLE_cs.text())
            self.current_vehilcle_physics_control.final_ratio=float(self.VTLE_fgr.text())


            #-----Wheel----------------------

            #as wheel changes are saved right after changes are done,it is not done here

            self.selected_actor.apply_physics_control(self.current_vehilcle_physics_control)
            print(f'changes are applied to vehicle id {id}')
        except Exception as e:
            QMessageBox.critical(self, "Error",f"{e}cloud not apply changes, please select ID or save as xml file")
            
            
        

        # reaccess the vehicle params and print

    def list_wheels(self,wheels):
        ids=[]
        for i,_ in enumerate(wheels):
            ids.append(str(i))
        #debug
        print(f'wheels id list {ids}')
        return ids
    
    def list_gears(self,gears):
        ids=[]
        for i,_ in enumerate(gears):
            ids.append(str(i))
        #debug
        print(f'gear id list {ids}')
        return ids

  
    def fetch_vehicle_details(self):
        id=int(self.VCB_id.currentText())
        print(f'selected id {id}')
        
        self.selected_actor=self.world.get_actor(id)

        self.current_vehilcle_physics_control=self.selected_actor.get_physics_control()
        #----Engine---------------
        self.torq_curve=self.current_vehilcle_physics_control.torque_curve
        self.torq_curve_list=[]
        for pt in self.torq_curve:
            self.torq_curve_list.append((pt.x,pt.y))
        self.plot_csv(self.torq_curve_list,self.figure_torq,self.canvas_torq,'RPM vs Torque(Nm)','RPM','Torque(Nm)')
        
        self.max_rpm = self.current_vehilcle_physics_control.max_rpm
        self.moi=self.current_vehilcle_physics_control.moi
        self.damping_rate_full_throttle=self.current_vehilcle_physics_control.damping_rate_full_throttle
        self.damping_rate_zero_throttle_clutch_engaged=self.current_vehilcle_physics_control.damping_rate_zero_throttle_clutch_engaged
        self.damping_rate_zero_throttle_clutch_disengaged=self.current_vehilcle_physics_control.damping_rate_zero_throttle_clutch_disengaged
         
        
        
        #-----Body----------------
        self.steering_curve=self.current_vehilcle_physics_control.steering_curve
        self.steering_curve_list=[]
        for pt in self.steering_curve:
            self.steering_curve_list.append((pt.x,pt.y))
        print(f'steering curve {self.steering_curve_list}')
        self.plot_csv(self.steering_curve_list,self.figure_steer,self.canvas_steer,'Steering Angle curve','Angle(deg)','Speed(m/s)')
        
        self.mass=self.current_vehilcle_physics_control.mass
        self.drag_coefficient=self.current_vehilcle_physics_control.drag_coefficient
        self.center_of_mass=self.current_vehilcle_physics_control.center_of_mass
        self.use_sweep_wheel_collision=self.current_vehilcle_physics_control.use_sweep_wheel_collision


        #-------------Transmission gears--------------
        
        
        self.current_forward_gears=self.current_vehilcle_physics_control.forward_gears
        
        self.gear_list=self.list_gears(self.current_forward_gears)
        self.VCB_gears.clear()
        self.VCB_gears.addItems(self.gear_list)
        self.fetch_gear_details()
        
        self.use_gear_autobox=self.current_vehilcle_physics_control.use_gear_autobox
        self.gear_switch_time=self.current_vehilcle_physics_control.gear_switch_time
        self.clutch_strength=self.current_vehilcle_physics_control.clutch_strength
        self.final_ratio=self.current_vehilcle_physics_control.final_ratio
        
        
        
        #----------Wheels------------------------------
        
        #list all wheels and show first wheels detials by default
        self.current_wheels=self.current_vehilcle_physics_control.wheels
        self.wheel_list=self.list_wheels(self.current_wheels)
        self.VCB_wheels.clear()
        self.VCB_wheels.addItems(self.wheel_list)
        self.fetch_wheel_details()
        # 
      



        #self.vehicle_name=self.selected_actor.type_id
        _,self.vehicle_make,self.vehicle_model=self.selected_actor.type_id.split('.')

        print(self.vehicle_make,self.vehicle_model)
        
        #-----Engine--------------
        self.VLE_model.setText(self.vehicle_model)
        self.VLE_maker.setText(self.vehicle_make)
        self.VELE_mrpm.setText(str(self.max_rpm))
        self.VELE_moi.setText(str(self.moi))
        self.VELE_drft.setText(str(self.damping_rate_full_throttle))
        self.VELE_drztc.setText(str(self.damping_rate_zero_throttle_clutch_engaged))
        self.VELE_drztnc.setText(str(self.damping_rate_zero_throttle_clutch_disengaged))
        
        #----body------------
        #self.VBLE_sc.setText(str(self.steering_curve))
        self.VBPB_ms.setText(str(self.mass))
        self.VBPB_dc.setText(str(self.drag_coefficient))
        self.VBPB_com.setText(str(self.center_of_mass))
        self.VBPB_swc.setText(str(self.use_sweep_wheel_collision))


        #------Transmission--------------------
        
        
        self.VTLE_agb.setText(str(self.use_gear_autobox))
        self.VTLE_gst.setText(str(self.gear_switch_time))
        self.VTLE_cs.setText(str(self.clutch_strength))
        self.VTLE_fgr.setText(str(self.final_ratio))



        #print(f'roal name {self.vehicle_name}')

    def fetch_wheel_details(self):
        id=self.VCB_wheels.currentIndex()
        #id=self.VCB_wheels.currentIndex()
        selected_wheel=self.current_vehilcle_physics_control.wheels[id]
        
        self.tire_friction =selected_wheel.tire_friction 
        self.damping_rate=selected_wheel.damping_rate
        self.max_steer_angle=selected_wheel.max_steer_angle
        self.radius=selected_wheel.radius
        self.max_brake_torque=selected_wheel.max_brake_torque
        self.max_handbrake_torque =selected_wheel.max_handbrake_torque 
        self.position=selected_wheel.position
        self.long_stiff_value=selected_wheel.long_stiff_value
        self.lat_stiff_max_load=selected_wheel.lat_stiff_max_load
        self.lat_stiff_value=selected_wheel.lat_stiff_value

        #---set texts-----------------
        
        self.VWLE_tf.setText(str(self.current_vehilcle_physics_control.wheels[id].tire_friction))
        self.VWLE_dr.setText(str(self.current_vehilcle_physics_control.wheels[id].damping_rate))
        self.VWLE_msa.setText(str(self.current_vehilcle_physics_control.wheels[id].max_steer_angle))
        self.VWLE_rad.setText(str(self.current_vehilcle_physics_control.wheels[id].radius))
        self.VWLE_mbt.setText(str(self.current_vehilcle_physics_control.wheels[id].max_brake_torque))
        self.VWLE_mhbt.setText(str(self.current_vehilcle_physics_control.wheels[id].max_handbrake_torque))
        self.VWLE_lgsv.setText(str(self.current_vehilcle_physics_control.wheels[id].long_stiff_value))
        self.VWLE_ltsv.setText(str(self.current_vehilcle_physics_control.wheels[id].lat_stiff_value))
        self.VWLE_ltmsl.setText(str(self.current_vehilcle_physics_control.wheels[id].lat_stiff_max_load))
    
    def fetch_gear_details(self):
        id=self.VCB_gears.currentIndex()
        print(f'Selected Gear: {id}')
        
        self.VTLE_fg_ratio.setText(str(self.current_vehilcle_physics_control.forward_gears[id].ratio))
        self.VTLE_fg_up_ratio.setText(str(self.current_vehilcle_physics_control.forward_gears[id].up_ratio))
        self.VTLE_fg_down_ratio.setText(str(self.current_vehilcle_physics_control.forward_gears[id].down_ratio))
        '''self.VTLE_fg_ratio.setText(str(self.current_forward_gears[id].ratio))
        self.VTLE_fg_up_ratio.setText(str(self.current_forward_gears[id].up_ratio))
        self.VTLE_fg_down_ratio.setText(str(self.current_forward_gears[id].down_ratio))'''

    
    def plot_csv(self, data, figure, canvas,title='Plot',x_label='x',y_label='y'):
        try:
            figure.clear()
            ax = figure.add_subplot(111)

            # Check if input is a list of (x, y) tuples
            if isinstance(data, list) and all(isinstance(t, tuple) and len(t) == 2 for t in data):
                x, y = zip(*data)
                ax.plot(x, y, marker='o', linestyle='-')
                ax.set_title(title)
                ax.set_xlabel(x_label)
                ax.set_ylabel(y_label)
            else:
                # Assume it's a pandas DataFrame
                self.df = data
                self.df.plot(ax=ax)
                ax.set_title(title)
                ax.set_xlabel(x_label)
                ax.set_ylabel(y_label)

            canvas.draw()  # ✅ draw on canvas, not figure

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to plot data:\n{e}")

    def on_torq_clicked(self, event):
        if self.torq_curve_list is not None:
            self.open_full_plot_window(self.torq_curve_list,'RPM vs Torque(Nm)','RPM','Torque(Nm)')

    def on_steer_clicked(self, event):
        if self.steering_curve_list is not None:
            self.open_full_plot_window(self.steering_curve_list,'Steering Angle curve','Angle(deg)','Speed(m/s)')

    def open_full_plot_window(self, data,title='Plot',x_label='x',y_label='y'):
            """Create a new window with toolbar + full Matplotlib interactivity."""
            try:
                self.full_plot_window = QtWidgets.QMainWindow(self)
                self.full_plot_window.setWindowTitle("Interactive Plot")
                self.full_plot_window.resize(800, 600)

                central_widget = QtWidgets.QWidget()
                layout = QVBoxLayout(central_widget)

                figure = Figure()
                canvas = FigureCanvas(figure)
                toolbar = NavigationToolbar(canvas, self.full_plot_window)

                layout.addWidget(toolbar)
                layout.addWidget(canvas)

                ax = figure.add_subplot(111)

                # ✅ Handle list of (x, y) tuples or DataFrame
                if isinstance(data, list) and all(isinstance(t, tuple) and len(t) == 2 for t in data):
                    x, y = zip(*data)
                    ax.plot(x, y, marker='o', linestyle='-')
                    ax.set_title(title)
                    ax.set_xlabel(x_label)
                    ax.set_ylabel(y_label)
                else:
                    # Assume pandas DataFrame
                    self.df = data
                    self.df.plot(ax=ax)
                    ax.set_title(title)
                    ax.set_xlabel(x_label)
                    ax.set_ylabel(y_label)

                canvas.draw()

                self.full_plot_window.setCentralWidget(central_widget)
                self.full_plot_window.show()

            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to open plot window:\n{e}")

    def open_file_dialog(self, figure, canvas):
        
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select CSV File",
            "",
            "CSV Files (*.csv);;All Files (*)"
        )
        if file_path:
            print(f"Selected path: {file_path}")
            data=pd.read_csv(file_path)
            if data.shape[1] > 2:
                data = data.iloc[:, :2]

            # Convert to list of (x, y) tuples
            tuples_list = list(data.itertuples(index=False, name=None))
            
            sender = self.sender()  # returns the QObject that sent the signal
            print("Triggered by:", sender.objectName())
            if sender.objectName().endswith('torque'):
                self.torq_curve_list=tuples_list
                title,x_label,y_label=['RPM vs Torque(Nm)','RPM','Torque(Nm)']
            else:
                self.steering_curve_list=tuples_list
                title,x_label,y_label=['Steering Angle curve','Angle(deg)','Speed(m/s)']

            self.plot_csv(tuples_list, figure, canvas,title,x_label,y_label)

    
    def save_wheel(self):
        id=self.VCB_wheels.currentIndex()
        
        self.current_wheels[id].tire_friction=float(self.VWLE_tf.text())
        self.current_wheels[id].damping_rate=float(self.VWLE_dr.text())
        self.current_wheels[id].max_steer_angle=float(self.VWLE_msa.text())
        self.current_wheels[id].radius=float(self.VWLE_rad.text())
        self.current_wheels[id].max_brake_torque=float(self.VWLE_mbt.text())
        self.current_wheels[id].max_handbrake_torque=float(self.VWLE_mhbt.text())
        
        
        self.current_wheels[id].long_stiff_value=float(self.VWLE_lgsv.text())
        self.current_wheels[id].lat_stiff_max_load=float(self.VWLE_ltmsl.text())
        self.current_wheels[id].lat_stiff_value=float(self.VWLE_ltsv.text())

        self.current_vehilcle_physics_control.wheels=self.current_wheels
        #self.selected_actor.apply_physics_control(self.current_vehilcle_physics_control)
        print('wheel changes applied')
        
    def save_gear(self):
        id=self.VCB_gears.currentIndex()
        self.current_forward_gears[id].ratio=float(self.VTLE_fg_ratio.text())
        self.current_forward_gears[id].up_ratio=float(self.VTLE_fg_up_ratio.text())
        self.current_forward_gears[id].down_ratio=float(self.VTLE_fg_down_ratio.text())
        
        self.current_vehilcle_physics_control.forward_gears=self.current_forward_gears
        #self.selected_actor.apply_physics_control(self.current_vehilcle_physics_control)
        print('gear changes applied')

    def fetch_weather_details(self):
        #self.world.set_weather(carla.WeatherParameters.Default)
        self.weather=self.world.get_weather()
        self.cloudiness=self.weather.cloudiness
        self.precipitation=self.weather.precipitation
        self.precipitation_deposits=self.weather.precipitation_deposits
        self.wind_intensity=self.weather.wind_intensity
        self.sun_azimuth_angle=self.weather.sun_azimuth_angle
        self.sun_altitude_angle=self.weather.sun_altitude_angle
        self.fog_density=self.weather.fog_density
        self.fog_distance=self.weather.fog_distance
        self.wetness=self.weather.wetness
        self.fog_falloff=self.weather.fog_falloff
        self.scattering_intensity=self.weather.scattering_intensity
        self.mie_scattering_scale=self.weather.mie_scattering_scale
        self.rayleigh_scattering_scale=self.weather.rayleigh_scattering_scale
        self.dust_storm=self.weather.dust_storm
        

        
        self.WHS_cloudiness.setValue(int(self.cloudiness))
        self.WHS_precipitation.setValue(int(self.precipitation))
        self.WHS_precipitation_deposits.setValue(int(self.precipitation_deposits))
        self.WHS_wind_intensity.setValue(int(self.wind_intensity))
        self.WHS_sun_azimuth_angle.setValue(int(self.sun_azimuth_angle))
        self.WHS_sun_altitude_angle.setValue(int(self.sun_altitude_angle))
        self.WHS_fog_density.setValue(int(self.fog_density))
        self.WHS_fog_distance.setValue(int(self.fog_distance))
        self.WHS_wetness.setValue(int(self.wetness))
        self.WHS_fog_falloff.setValue(int(self.fog_falloff))
        self.WHS_scattering_intensity.setValue(int(self.scattering_intensity))
        self.WHS_mie_scattering_scale.setValue(int(self.mie_scattering_scale))
        self.WHS_rayleigh_scattering_scale.setValue(int(self.rayleigh_scattering_scale))
        self.WHS_dust_storm.setValue(int(self.dust_storm))

    def apply_weather_details(self):
        self.weather=self.world.get_weather()
        self.weather.cloudiness=self.WHS_cloudiness.value()
        self.weather_precipitation=self.WHS_precipitation.value()
        self.weather.precipitation_deposits = self.WHS_precipitation_deposits.value()
        #self.weather.wetness = self.WHS_precipitation.value()
        self.weather.wind_intensity=self.WHS_wind_intensity.value()
        self.weather.sun_azimuth_angle=self.WHS_sun_azimuth_angle.value()
        self.weather.sun_altitude_angle=self.WHS_sun_altitude_angle.value()
        self.weather.fog_density=self.WHS_fog_density.value()
        self.weather.fog_distance=self.WHS_fog_distance.value()
        self.weather.wetness=self.WHS_wetness.value()
        self.weather.fog_falloff=self.WHS_fog_falloff.value()
        self.weather.scattering_intensity=self.WHS_scattering_intensity.value()
        self.weather.mie_scattering_scale=self.WHS_mie_scattering_scale.value()
        self.weather.rayleigh_scattering_scale=self.WHS_rayleigh_scattering_scale.value()
        self.weather.dust_storm=self.WHS_dust_storm.value()
        
        
        
        
        
        print(f'applied precipitatoin value {self.weather_precipitation}')
        self.world.set_weather(self.weather)

    def set_weather_default(self):
        self.world.set_weather(carla.WeatherParameters.Default)
        print("Weather has been reset to default.")
        self.fetch_weather_details()

    #------------Data saving in xml format----------------------------------------
    def vector2d_to_dict(self,v):
        return {"x": v.x, "y": v.y}

    def vector3d_to_dict(self,v):
        return {"x": v.x, "y": v.y, "z": v.z}

    def wheel_to_dict(self,wheel):
        return {
            "tire_friction": wheel.tire_friction,
            "damping_rate": wheel.damping_rate,
            "max_steer_angle": wheel.max_steer_angle,
            "radius": wheel.radius,
            "max_brake_torque": wheel.max_brake_torque,
            "max_handbrake_torque": wheel.max_handbrake_torque,
            "position": self.vector3d_to_dict(wheel.position),
            "long_stiff_value":wheel.long_stiff_value,
            "lat_stiff_value":wheel.lat_stiff_value,
            "lat_stiff_max_load":wheel.lat_stiff_max_load
        }
    
    def gear_to_dict(self,gear):
        return{
            "ratio":gear.ratio,
            "down_ratio":gear.down_ratio,
            "up_ratio":gear.up_ratio
        }

    def physics_to_dict(self,p):
        return {
            "torque_curve": [self.vector2d_to_dict(p) for p in p.torque_curve],
            "max_rpm": p.max_rpm,
            "moi": p.moi,
            "damping_rate_full_throttle": p.damping_rate_full_throttle,
            "damping_rate_zero_throttle_clutch_engaged": p.damping_rate_zero_throttle_clutch_engaged,
            "damping_rate_zero_throttle_clutch_disengaged": p.damping_rate_zero_throttle_clutch_disengaged,
            "use_gear_autobox": p.use_gear_autobox,
            "gear_switch_time": p.gear_switch_time,
            "clutch_strength": p.clutch_strength,
            "final_ratio": p.final_ratio,
            "forward_gears": [self.gear_to_dict(gear) for gear in p.forward_gears],
            "mass": p.mass,
            "drag_coefficient": p.drag_coefficient,
            "center_of_mass": self.vector3d_to_dict(p.center_of_mass),
            "steering_curve": [self.vector2d_to_dict(s) for s in p.steering_curve],
            "use_sweep_wheel_collision": p.use_sweep_wheel_collision,
            "wheels": [self.wheel_to_dict(w) for w in p.wheels]
        }

    def dict_to_xml(self,parent, data):
        """Recursively convert a dict/list to XML nodes."""
        if isinstance(data, dict):
            for key, value in data.items():
                child = ET.SubElement(parent, key)
                self.dict_to_xml(child, value)
        elif isinstance(data, list):
            for item in data:
                item_elem = ET.SubElement(parent, "item")
                self.dict_to_xml(item_elem, item)
        else:
            parent.text = str(data)
    #------------------------------------------------------------------------------
    

    #-----Parsing XML for loading from file--------------------------------------------
    def parse_vector2d_list(self,parent):
        """Extract list of (x, y) tuples."""
        pts = []
        for item in parent.findall("item"):
            x = float(item.find("x").text)
            y = float(item.find("y").text)
            pts.append((x, y))
        return pts

    def parse_vector3d(self,node):
        """Extract x,y,z vector3 dict."""
        return carla.Vector3D(
            x= float(node.find("x").text),
            y= float(node.find("y").text),
            z= float(node.find("z").text)
        )

    def parse_float(self,node, default=0.0):
        return float(node.text) if node is not None and node.text else default
    #-----End parsing XML for loading form file-------------------------------------
    
    def save_actors_to_xml(self,file):
        '''id=int(self.VCB_id.currentText())
        print(f'selected id {id}')
        
        self.selected_actor=self.world.get_actor(id)
        self.current_vehilcle_physics_control=self.selected_actor.get_physics_control()'''

        data=self.physics_to_dict(self.current_vehilcle_physics_control)
        root = ET.Element("vehicle_physics_control")
        self.dict_to_xml(root, data)
        tree = ET.ElementTree(root)
        #  Pretty-print XML
        xml_str = ET.tostring(root, encoding="utf-8")
        pretty_xml = minidom.parseString(xml_str).toprettyxml(indent="   ")

        with open(self.current_file, "w", encoding="utf-8") as f:
            f.write(pretty_xml)
            
        self.statusBar().showMessage(f"Saved physics parameters to {self.current_file}", 3000)

    def save_file(self):
        """Triggered by 'Save' action"""
        if not self.current_file:
            self.save_file_as()
        else:
            try:
                self.save_actors_to_xml(self.current_file)
                self.statusBar().showMessage(f"Saved to {self.current_file}", 3000)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Could not save file:\n{e}")
    def save_file_as(self):
         
         """Triggered by 'Save As' action"""
         filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Actor Parameters As",
            "",
            "XML Files (*.xml);;All Files (*)"
        )

         if filename:
            self.current_file = filename
            self.save_file()
    
    def open_file(self):
        
        """Triggered by 'Open' action — optional loading logic"""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Open Actor Parameters",
            "",
            "XML Files (*.xml);;All Files (*)"
        )

        if not filename:
            return

        self.current_file = filename
        try:
            tree = ET.parse(filename)
            root = tree.getroot()
            # Here you could read data and recreate actors if desired
            QMessageBox.information(self, "Opened", f"Loaded actor data from:\n{filename}")

            #-----loading all values to display---------------
            #--- Torque Curve ---
            torque_curve_node = root.find("torque_curve")
            self.torq_curve_list = self.parse_vector2d_list(torque_curve_node)
            self.plot_csv(self.torq_curve_list, self.figure_torq, self.canvas_torq,'RPM vs Torque(Nm)', 'RPM', 'Torque(Nm)')
            self.torq_curve = [carla.Vector2D(x, y) for x, y in self.torq_curve_list]
            self.current_vehilcle_physics_control.torque_curve=self.torq_curve
            
            #--- Steering curve ---
            steering_curve_node = root.find("steering_curve")
            self.steering_curve_list = self.parse_vector2d_list(steering_curve_node)
            self.plot_csv(self.steering_curve_list, self.figure_steer, self.canvas_steer,'Steering Angle curve', 'Angle(deg)', 'Speed(m/s)')
            self.steering_curve = [carla.Vector2D(x, y) for x, y in self.steering_curve_list]
            self.current_vehilcle_physics_control.steering_curve=self.steering_curve


            #--- Engine parameters ---
            self.max_rpm = self.parse_float(root.find("max_rpm"))
            self.moi = self.parse_float(root.find("moi"))
            self.damping_rate_full_throttle = self.parse_float(root.find("damping_rate_full_throttle"))
            self.damping_rate_zero_throttle_clutch_engaged = self.parse_float(root.find("damping_rate_zero_throttle_clutch_engaged"))
            self.damping_rate_zero_throttle_clutch_disengaged = self.parse_float(root.find("damping_rate_zero_throttle_clutch_disengaged"))

            self.current_vehilcle_physics_control.max_rpm=self.max_rpm
            self.current_vehilcle_physics_control.moi=self.moi
            self.current_vehilcle_physics_control.damping_rate_full_throttle=self.damping_rate_full_throttle
            self.current_vehilcle_physics_control.damping_rate_zero_throttle_clutch_engaged=self.damping_rate_zero_throttle_clutch_engaged
            self.current_vehilcle_physics_control.damping_rate_zero_throttle_clutch_disengaged=self.damping_rate_zero_throttle_clutch_disengaged
            

            #--- Body params ---
            self.mass = self.parse_float(root.find("mass"))
            self.drag_coefficient = self.parse_float(root.find("drag_coefficient"))

            com_node = root.find("center_of_mass")
            if com_node is not None:
                self.center_of_mass = self.parse_vector3d(com_node)
            else:
                self.center_of_mass = carla.Vector3D(x= 0, y= 0, z= 0)

            swc_node = root.find("use_sweep_wheel_collision")
            self.use_sweep_wheel_collision = bool(swc_node.text) if swc_node is not None else False

            self.current_vehilcle_physics_control.mass=self.mass
            self.current_vehilcle_physics_control.drag_coefficient=self.drag_coefficient
            self.current_vehilcle_physics_control.center_of_mass=self.center_of_mass
            self.current_vehilcle_physics_control.use_sweep_wheel_collision=self.use_sweep_wheel_collision   
            

            #--- Transmission ---
            self.current_forward_gears=[]
            fgear_node = root.find("forward_gears")
            if fgear_node is not None:
                for item in fgear_node.findall("item"):
                    gear=carla.GearPhysicsControl(
                        ratio=self.parse_float(item.find("ratio")),
                        down_ratio=self.parse_float(item.find("down_ratio")),
                        up_ratio=self.parse_float(item.find("up_ratio"))
                    )
                    self.current_forward_gears.append(gear)
            else:
                self.current_forward_gears = []

            
            
            gab_node=root.find("use_gear_autobox")
            self.use_gear_autobox = bool(gab_node.text) if gab_node is not None else False
            self.gear_switch_time = self.parse_float(root.find("gear_switch_time"))
            self.clutch_strength = self.parse_float(root.find("clutch_strength"))
            self.final_ratio = self.parse_float(root.find("final_ratio"))

            self.current_vehilcle_physics_control.forward_gears=self.current_forward_gears
            self.current_vehilcle_physics_control.use_gear_autobox=self.use_gear_autobox
            self.current_vehilcle_physics_control.gear_switch_time=self.gear_switch_time
            self.current_vehilcle_physics_control.clutch_strength=self.clutch_strength
            self.current_vehilcle_physics_control.final_ratio=self.final_ratio



            #--- Wheels ---
            self.current_wheels = []
            wheel_nodes = root.find("wheels")
            if wheel_nodes is not None:
                for item in wheel_nodes.findall("item"):
                    #print('wheel item found')
                    wheel = carla.WheelPhysicsControl(
                        tire_friction= self.parse_float(item.find("tire_friction")),
                        damping_rate= self.parse_float(item.find("damping_rate")),
                        max_steer_angle= self.parse_float(item.find("max_steer_angle")),
                        radius=self.parse_float(item.find("radius")),
                        max_brake_torque= self.parse_float(item.find("max_brake_torque")),
                        max_handbrake_torque= self.parse_float(item.find("max_handbrake_torque")),
                        lat_stiff_max_load= self.parse_float(item.find("lat_stiff_max_load")),
                        lat_stiff_value= self.parse_float(item.find("lat_stiff_value")),
                        long_stiff_value= self.parse_float(item.find("long_stiff_value")),
                        position= self.parse_vector3d(item.find("position"))
                    )
                    #print(f'tire friction {wheel.tire_friction}')
                    self.current_wheels.append(wheel)
            else:
                print('no wheels found')

            print(f'wheels len {len(self.current_wheels)}')
            self.current_vehilcle_physics_control.wheels=self.current_wheels
            
            
            #--- Populate combo boxes ---
            
            

            self.gear_list = self.list_gears(self.current_vehilcle_physics_control.forward_gears)
            self.VCB_gears.clear()
            self.VCB_gears.addItems(self.gear_list)
            self.fetch_gear_details()

            self.wheel_list=self.list_wheels(self.current_vehilcle_physics_control.wheels)
            self.VCB_wheels.clear()
            self.VCB_wheels.addItems(self.wheel_list)
            self.fetch_wheel_details()

            # ============================
            # Update GUI fields
            # ============================
            #self.VLE_model.setText(self.vehicle_model if self.vehicle_model else "")
            #self.VLE_maker.setText(self.vehicle_make if self.vehicle_make else "")

            self.VELE_mrpm.setText(str(self.max_rpm))
            self.VELE_moi.setText(str(self.moi))
            self.VELE_drft.setText(str(self.damping_rate_full_throttle))
            self.VELE_drztc.setText(str(self.damping_rate_zero_throttle_clutch_engaged))
            self.VELE_drztnc.setText(str(self.damping_rate_zero_throttle_clutch_disengaged))

            self.VBPB_ms.setText(str(self.mass))
            self.VBPB_dc.setText(str(self.drag_coefficient))
            self.VBPB_com.setText(str(self.center_of_mass))
            self.VBPB_swc.setText(str(self.use_sweep_wheel_collision))

            self.VTLE_agb.setText(str(self.use_gear_autobox))
            self.VTLE_gst.setText(str(self.gear_switch_time))
            self.VTLE_cs.setText(str(self.clutch_strength))
            self.VTLE_fgr.setText(str(self.final_ratio))

        

            #---------------------END Loading values to display----------------------------
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Could not open file:\n{e}")
    def new_file(self):
        None
    def exit(self):
        None

   
            




if __name__=="__main__":
   
  
    app=QApplication(sys.argv)


    window=MainWindow()
    window.show()
    app.exec_()


