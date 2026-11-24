"""AeroTray model parameters with provenance. Units SI unless stated.
Sources: [FW] firmware v9; [CAD] Fusion/STEP assembly; [DS] datasheet; [MEAS] measured; [ID] identified here; [EST] estimate."""
import numpy as np
g=9.81
P=dict(
 # --- mobile base
 r=0.0493,            # wheel radius [CAD/FW: 0.2151 mm per count *1440/(2pi)]
 b=0.3794,            # track width [FW LabVIEW]
 Lwb=0.220,           # wheelbase driven axle -> omni axle [MEAS 23-nov, tape]
 kappa_enc=0.2151e-3, # m per encoder count [FW]
 N_enc=1440,          # counts per wheel rev [DS]
 tau_s=4.94,          # TorqueNADO stall torque at 12 V, N m (700 oz-in) [DS]
 w0=100*2*np.pi/60,   # no-load speed 100 rpm [DS]
 i_stall=8.7,         # A [DS]
 Vbat=12.0,
 Crr=0.02,            # rolling resistance [EST]
 M_base=6.8,          # everything except the 3.6 kg aerial module (total 10.4 kg with all batteries) [MEAS 23-nov]
 M_total=10.4,        # whole robot with batteries [MEAS 23-nov, bathroom scale]
 xG_base=0.088, hG_base=0.081,   # base CoM [ID 23-nov from tip angles 21.5/30.5 deg, module as point mass at joint]
 tip_climb_meas=21.5, tip_desc_meas=30.5,   # static tip angles, module locked [MEAS 23-nov]
 lJ=0.088,            # mast axis ahead of driven axle [CAD Lab3]
 hJ0=0.4945,          # joint centre above ground, carriage at rest [CAD]
 d_max=0.140,         # guide travel [MEAS 23-nov; CAD said 0.0766]
 # --- aerial module
 m=3.3, mc=0.3,       # rotating assembly / carriage [MEAS total 3.6, split CAD-MuJoCo]
 J=np.array([0.12,0.12,0.20]),   # about joint [CAD-MuJoCo, provisional]
 hc=0.14, hc_cad=0.10, hc_sd=0.06,  # CoM above joint [ID moment balance] / [CAD]
 dx=0.0095, dx_sd=0.0018, dy=0.0019, dy_sd=0.0015,  # CoM offset [ID]
 Lr=0.350, hr=0.0955,   # rotor radius, rotor plane above joint [CAD]
 kF_fw=5.3955, u0=1140., n_th=1.6, uspan=860.,      # firmware thrust curve [FW]
 kF_id=(7.17,8.18),   # from lift-off 1775/1725 us [ID]
 cQ=0.020,            # yaw torque / thrust [FW]
 Dprop=9.4*0.0254,    # 9450 propeller diameter
 rho_air=1.20,
 tau_m=0.07,          # motor+ESC time constant [EST]
 T_esc=0.020,         # 50 Hz ESC frame + IMU DLPF + 1 control period [FW]
 # firmware gains (v9, in flight)
 kp_th=4.62, kd_th=0.12, ki_th_bench=0.8, ki_th_fl=2.0, tf_d=0.025,
 kp_w=0.848, ki_w=0.100,
 wlim_bench=np.radians(25), wlim_fl=np.radians(60), Ilim_bench=0.3, Ilim_fl=3.0,
 # --- tray
 hP=0.1038,           # gimbal (cross) centre above joint [CAD: 92.3 mm - (-11.5 mm)]
 eps=0.0369,          # ball centre above gimbal centre, 27 mm ball [CAD]
 rb=0.0135,           # ball radius [MEAS]
 plate=(0.250,0.220),
 r_arm=0.020, l_rod=0.036, R_horn=0.04585, S=(0.02585,0.012), Gp=(0.0,0.048),  # linkage [CAD v5]
 servo_deg_per_us=45/400,   # Futaba S3003 [DS]
 servo_speed=60/0.23,        # deg/s at 4.8 V [DS]
 u_lim=220.,                 # us [FW]
 kp_b=1.5/3, ki_b=0.8/3, kd_b=4.5/3,  # us per mm, us per mm s, us per mm/s  (firmware units of 3 mm) [FW]
 Tv=0.030,                   # LS velocity window 60 ms -> 30 ms lag [FW]
 cam_px_per_mm=0.8, fps=187., H_cam=0.20,
)
P['kalpha']=np.radians(P['servo_deg_per_us'])*P['r_arm']/P['R_horn']   # rad plate per us
P['Kb']=5/7*g*P['kalpha']*1000     # mm s^-2 per us
P['kappa']=P['eps']+0.4*P['rb']
