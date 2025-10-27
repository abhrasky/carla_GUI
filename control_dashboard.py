#!/usr/bin/env python
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication,QMainWindow,QWidget,QAction,QLineEdit,QComboBox
from PyQt5.QtCore import *


import sys
import glob
import argparse
import json

import carla
import re



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
        
        
        
        self.connect_server('localhost',2000)
        self.on_tab_changes()
        

        self.VPB_apply.clicked.connect(self.apply_changes)
        self.VCB_id.activated.connect(self.fetch_vehicle_details)
        self.VCB_wheels.activated.connect(self.fetch_wheel_details)
        
        
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
        tab_id=self.TABS.currentIndex()
        print(f'current tab {tab_id}')

        if tab_id==0:
            self.get_actors()
            self.VCB_id.addItems(self.actor_ids)
        elif tab_id==1:
            self.fetch_weather_details()


    def get_actors(self):
        self.actor_ids=[] #clean previous ones
        for actor in self.world.get_actors().filter('vehicle.*'):
            self.actor_ids.append(str(actor.id))


    def apply_changes(self):
        #connect to server
        id=int(self.VCB_id.currentText())
        print(f'selected id {id}')
        #acces phyics of ego vhielce
        self.selected_actor=self.world.get_actor(id)
        physics = self.selected_actor.get_physics_control()

        #-----Engine-----------------    
        physics.vehicle_model=self.VLE_model.text()
        physics.vehicle_make=self.VLE_maker.text()
        physics.max_rpm=float(self.VELE_mrpm.text())
        physics.moi=float(self.VELE_moi.text())
        physics.damping_rate_full_throttle=float(self.VELE_drft.text())
        physics.damping_rate_zero_throttle_clutch_engaged=float(self.VELE_drztc.text())
        physics.damping_rate_zero_throttle_clutch_disengaged=float(self.VELE_drztnc.text())

        #----body------------
        #self.VBLE_sc.setText(str(self.steering_curve))
        physics.mass=float(self.VBPB_ms.text())
        physics.drag_coefficient=float(self.VBPB_dc.text())
        coords = [float(v) for v in re.findall(r"[-+]?\d*\.\d+|\d+", self.VBPB_com.text())]
        physics.center_of_mass=carla.Vector3D(*coords)
        physics.use_sweep_wheel_collision=bool(self.VBPB_swc.text())

        #----Transmission-----------------



        #-----Wheel----------------------


        self.selected_actor.apply_physics_control(physics)
        

        # reaccess the vehicle params and print
    def list_wheels(self,wheels):
        ids=[]
        for i,_ in enumerate(wheels):
            ids.append(str(i))
        #debug
        print(f'wheels id list {ids}')
        return ids
    

    def fetch_wheel_details(self):
        id=int(self.VCB_wheels.currentText())
        selected_wheel=self.wheels[id]
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
        
        self.VWLE_tf.setText(str(self.tire_friction))
        self.VWLE_dr.setText(str(self.damping_rate))
        self.VWLE_msa.setText(str(self.max_steer_angle))
        self.VWLE_mbt.setText(str(self.max_brake_torque))
        self.VWLE_mhbt.setText(str(self.max_handbrake_torque))
        self.VWLE_lgsv.setText(str(self.long_stiff_value))
        self.VWLE_ltsv.setText(str(self.lat_stiff_value))
        self.VWLE_ltmsl.setText(str(self.lat_stiff_max_load))
     

    def fetch_vehicle_details(self):
        id=int(self.VCB_id.currentText())
        print(f'selected id {id}')
        
        self.selected_actor=self.world.get_actor(id)

        self.vehilcle_physics_control=self.selected_actor.get_physics_control()
        #----Engine---------------
        self.max_rpm = self.vehilcle_physics_control.max_rpm
        self.moi=self.vehilcle_physics_control.moi
        self.damping_rate_full_throttle=self.vehilcle_physics_control.damping_rate_full_throttle
        self.damping_rate_zero_throttle_clutch_engaged=self.vehilcle_physics_control.damping_rate_zero_throttle_clutch_engaged
        self.damping_rate_zero_throttle_clutch_disengaged=self.vehilcle_physics_control.damping_rate_zero_throttle_clutch_disengaged
         
        
        
        #-----Body----------------
        self.steering_curve=self.vehilcle_physics_control.steering_curve
        self.mass=self.vehilcle_physics_control.mass
        self.drag_coefficient=self.vehilcle_physics_control.drag_coefficient
        self.center_of_mass=self.vehilcle_physics_control.center_of_mass
        self.use_sweep_wheel_collision=self.vehilcle_physics_control.use_sweep_wheel_collision


        #-------------Transmission--------------
        self.forward_gears=self.vehilcle_physics_control.forward_gears
        self.use_gear_autobox=self.vehilcle_physics_control.use_gear_autobox
        self.gear_switch_time=self.vehilcle_physics_control.gear_switch_time
        self.clutch_strength=self.vehilcle_physics_control.clutch_strength
        self.final_ratio=self.vehilcle_physics_control.final_ratio
        
        
        
        #----------Wheels------------------------------
        
        #list all wheels and show first wheels detials by default
        self.wheels=self.vehilcle_physics_control.wheels
        self.wheel_list=self.list_wheels(self.wheels)
        
        self.VCB_wheels.addItems(self.wheel_list)
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







if __name__=="__main__":
   
  
    app=QApplication(sys.argv)


    window=MainWindow()
    window.show()
    app.exec_()

