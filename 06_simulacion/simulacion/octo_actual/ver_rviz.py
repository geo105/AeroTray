"""RViz desde Gazebo o reproducción del TXT. Tópicos exclusivos /dsm_sim."""
import argparse
import csv
import json
import math
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node as RosNode
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped, Point
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from control import ROOT, CFG, real_rows


def quaternion(rpy):
    r,p,y=np.array(rpy)/2; cr,cp,cy=np.cos([r,p,y]); sr,sp,sy=np.sin([r,p,y])
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


class Viewer(RosNode):
    def __init__(self, source, speed):
        super().__init__('octo_sim_visualizacion')
        self.source=source; self.speed=speed; self.start=time.monotonic(); self.latest=None
        self.lock=threading.Lock(); self.tf=TransformBroadcaster(self)
        self.pub=self.create_publisher(MarkerArray,'/dsm_sim/modelo',10)
        self.imu_pub=self.create_publisher(Imu,'/dsm_sim/imu',10)
        self.range_pub=self.create_publisher(Range,'/dsm_sim/tof',10)
        self.numeric=self.create_publisher(Float64MultiArray,'/dsm_sim/valores',10)
        self.status=self.create_publisher(String,'/dsm_sim/estado_texto',10)
        self.rows=None
        if source=='real': self.rows=real_rows()
        elif source!='gazebo':
            with open(source) as f: self.rows=list(csv.DictReader(f))
        else:
            from gz.transport13 import Node
            from gz.msgs10.stringmsg_pb2 import StringMsg
            self.gz=Node()
            if not self.gz.subscribe(StringMsg,'/dsm_sim/estado',self.on_gazebo):
                raise RuntimeError('No se pudo suscribir a Gazebo')
        self.create_timer(.05,self.tick)
        self.get_logger().info('Fuente '+source+'; no se accede a hardware')

    def on_gazebo(self, message):
        with self.lock: self.latest=json.loads(message.data)

    def transform(self, child, parent, xyz, rpy, stamp):
        t=TransformStamped(); t.header.stamp=stamp; t.header.frame_id=parent; t.child_frame_id=child
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyz)
        q=quaternion(rpy)
        t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
        return t

    def current(self):
        if self.source=='gazebo':
            with self.lock: return self.latest
        elapsed=(time.monotonic()-self.start)*self.speed
        if self.source=='real':
            duration=(self.rows[-1,0]-self.rows[0,0])/1000+.1
            idx=min(int((elapsed%duration)/.1),len(self.rows)-1); a=self.rows[idx]
            return {'modo':'REPRODUCCION REAL; altura no registrada','t':(a[0]-self.rows[0,0])/1000,
                'z':0.,'rpy':np.deg2rad(a[1:4]).tolist(),'angles':np.deg2rad(a[1:4]).tolist(),
                'omega':np.deg2rad(a[4:7]).tolist(),'pwm':a[9:17].astype(int).tolist(),
                'scale':float(a[7]),'cut':'','tof_valid':False,'tof':None,'height':None,'accel_g':float(a[17])}
        duration=float(self.rows[-1]['t_s'])+.02
        times=np.array([float(x['t_s']) for x in self.rows]); i=min(int(np.searchsorted(times,elapsed%duration)),len(times)-1)
        a=self.rows[i]
        state={'modo':'REPRODUCCION SIMULADA','t':float(a['t_s']),'z':float(a['z_m']),
            'rpy':np.deg2rad([float(a[k+'_deg']) for k in ['roll','pitch','yaw']]).tolist(),
            'angles':np.deg2rad([float(a['est_roll_deg']),float(a['est_pitch_deg']),float(a['yaw_deg'])]).tolist(),
            'omega':np.deg2rad([float(a[k+'_dps']) for k in ['p','q','r']]).tolist(),
            'pwm':[int(a[f'M{i}_us']) for i in range(1,9)],'scale':float(a['escala_RP']),
            'cut':a['corte'],'tof_valid':a['tof_valido']=='1','tof':float(a['tof_m']) if a['tof_valido']=='1' else None,
            'height':float(a['z_est_m']),'accel_g':float(a['accel_g'])}
        if 'bola_x_m' in a:
            state.update(ball=[float(a['bola_x_m']),float(a['bola_y_m'])],
                plate=[float(a['placa_x_rad']),float(a['placa_y_rad'])],ball_lost=a['bola_perdida']=='1')
        return state

    def marker(self, ident, frame, kind, stamp):
        m=Marker(); m.header.frame_id=frame; m.header.stamp=stamp; m.id=ident; m.ns='octo_actual'
        m.type=kind; m.action=Marker.ADD; m.pose.orientation.w=1.; m.color.a=1.
        return m

    def tick(self):
        s=self.current()
        if s is None: return
        stamp=self.get_clock().now().to_msg(); base=CFG['pivot_altura_m']
        self.tf.sendTransform([
            self.transform('dsm_base','dsm_world',[0,0,base],[0,0,0],stamp),
            self.transform('dsm_carro','dsm_world',[0,0,base+s['z']],[0,0,0],stamp),
            self.transform('dsm_dron','dsm_world',[0,0,base+s['z']],s['rpy'],stamp),
            self.transform('dsm_imu8','dsm_dron',CFG['sensores']['imu_pos_m'],[0,0,0],stamp),
            self.transform('dsm_tof1','dsm_dron',CFG['sensores']['tof_pos_m'],[0,math.pi/2,0],stamp)])
        if 'plate' in s:
            self.tf.sendTransform(self.transform('dsm_placa','dsm_dron',[0,0,CFG['ball_plate']['altura_placa_m']],
                [-s['plate'][1],s['plate'][0],0],stamp))
        markers=[]
        for i,(group,frame) in enumerate([('carrito_fijo','dsm_base'),('carro_vertical','dsm_carro'),('conjunto_rotatorio','dsm_dron')]):
            m=self.marker(i,frame,Marker.MESH_RESOURCE,stamp)
            m.mesh_resource=(ROOT/'mallas'/f'{group}.stl').as_uri(); m.mesh_use_embedded_materials=False
            m.scale.x=m.scale.y=m.scale.z=1.; m.color.r=.38; m.color.g=.50; m.color.b=.64; markers.append(m)
        if 'ball' in s:
            m=self.marker(4,'dsm_placa',Marker.MESH_RESOURCE,stamp)
            m.mesh_resource=(ROOT/'mallas/placa_movil.stl').as_uri(); m.scale.x=m.scale.y=m.scale.z=1.
            m.color.r=m.color.g=m.color.b=.55; markers.append(m)
            m=self.marker(5,'dsm_placa',Marker.SPHERE,stamp)
            m.pose.position.x,m.pose.position.y=map(float,s['ball']); m.pose.position.z=CFG['ball_plate']['diametro_bola_m']/2
            m.scale.x=m.scale.y=m.scale.z=CFG['ball_plate']['diametro_bola_m']; m.color.b=1.; m.color.g=.3; markers.append(m)
        for i,pos in enumerate(CFG['motores_flu_m']):
            m=self.marker(10+i,'dsm_dron',Marker.TEXT_VIEW_FACING,stamp)
            m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,np.array(pos)+[0,0,.04])
            m.scale.z=.022; m.color.r=1.; m.color.g=.8; m.color.b=.15
            pulse=s['pwm'][i]; m.text=f'M{i+1}\n'+(f'{pulse}us' if pulse>=0 else 'ideal'); markers.append(m)
        for i,(frame,label,color) in enumerate([('dsm_imu8','IMU8 (?)',(1.,.3,.2)),('dsm_tof1','ToF1',(.2,1.,.3))]):
            m=self.marker(30+i,frame,Marker.SPHERE,stamp); m.scale.x=m.scale.y=m.scale.z=.025
            m.color.r,m.color.g,m.color.b=color; markers.append(m)
            m=self.marker(35+i,'dsm_dron',Marker.TEXT_VIEW_FACING,stamp)
            pos=CFG['sensores']['imu_pos_m' if i==0 else 'tof_pos_m']; m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,np.array(pos)+[0,0,.09])
            m.scale.z=.022; m.text=label; m.color.r,m.color.g,m.color.b=color; markers.append(m)
        if s['tof_valid'] and s['tof'] is not None:
            m=self.marker(40,'dsm_tof1',Marker.LINE_LIST,stamp); m.scale.x=.004; m.color.g=1.
            m.points=[Point(),Point(x=float(s['tof']))]; markers.append(m)
        else:
            m=self.marker(40,'dsm_tof1',Marker.LINE_LIST,stamp); m.action=Marker.DELETE; markers.append(m)
        m=self.marker(50,'dsm_world',Marker.TEXT_VIEW_FACING,stamp)
        m.pose.position.z=.97; m.scale.z=.025; m.color.r=m.color.g=m.color.b=1.
        a=np.rad2deg(s['angles'])
        m.text=(f"{s['modo']} | t={s['t']:.1f}s\nroll {a[0]:.2f} / pitch {a[1]:.2f} deg | z {s['z']*1000:.1f} mm\n"
                f"RP {s['scale']:.3f} | {s['cut'] or 'sin corte'}")
        if 'ball' in s:
            m.text+=f"\nPID Pi/PS3 Eye | bola {1000*s['ball'][0]:.1f}, {1000*s['ball'][1]:.1f} mm"
            if s.get('ball_lost'): m.text+=' | FUERA DE PLACA'
        else:
            m.text+='\nSin medidas de pelota en este registro'
        status_text=m.text
        # Evitar el avance excesivo del glifo espacio en algunas fuentes Ogre.
        m.text=m.text.replace(' ','')
        for item in markers:
            if item.type==Marker.TEXT_VIEW_FACING: item.text=item.text.replace(' ','')
        markers.append(m); self.pub.publish(MarkerArray(markers=markers))
        imu=Imu(); imu.header.stamp=stamp; imu.header.frame_id='dsm_imu8'
        imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w=map(float,quaternion(s['angles']))
        imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z=map(float,s['omega'])
        if 'accel' in s:
            imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z=map(float,np.array(s['accel'])*9.81)
        else: imu.linear_acceleration_covariance[0]=-1.
        self.imu_pub.publish(imu)
        if s['tof_valid'] and s['tof'] is not None:
            r=Range(); r.header.stamp=stamp; r.header.frame_id='dsm_tof1'; r.radiation_type=Range.INFRARED
            r.field_of_view=.15; r.min_range=.03; r.max_range=2.; r.range=float(s['tof']); self.range_pub.publish(r)
        self.numeric.publish(Float64MultiArray(data=[float(s['t']),*map(float,a),float(s['z']),float(s['scale'])]))
        self.status.publish(String(data=status_text))


def main():
    p=argparse.ArgumentParser(); p.add_argument('--fuente',default='gazebo',help='gazebo, real o ruta a CSV simulado')
    p.add_argument('--velocidad',type=float,default=1.); a=p.parse_args()
    if a.velocidad<=0: p.error('La velocidad debe ser positiva')
    rclpy.init(); node=Viewer(a.fuente,a.velocidad)
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
