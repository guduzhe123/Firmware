**position control simulink test 2017/11/06**

input:
```$xslt
_pos_sp_triplet.current.type: 0

control_auto
dt:0.006924
prev_sp: [ 0.010043	-0.009413	-49.950958	]
next_sp: [ 234.780365	-199.551117	-49.951385	]
curr_pos_sp: [ 0.348132	-200.409622	-49.951385	]
pos: [ 0.561133	-28.360605	-50.073399	]
vel: [ -0.164547	-4.123904	0.132859	]
att_euler:[ 0.012112	-0.454604	-1.575729	]
pos_sp_prev:[ 0.066686	-33.584118	-49.951385	]
_vel_sp_prev:[ -0.470636	-4.977800	0.120098	]
_att_sp.yaw_body :-1.572101

calculate_thrust_setpoint
vel_err_d:[ -0.123662	0.166068	-0.043227	]
thrust_int_prev:[ 0.034167	-0.224848	-0.087394	]

control_attitude
dt: 0.004781
rates_prev: [ -0.012411	-0.008419	0.001262	]
rate_int_prev: [ 0.003815	-0.041016	-0.000669	]
attitude_vel: [ -0.010549	0.002112	0.003795    ]

```


px4 gazebo result:
```
control_auto
pos_sp:[ 0.066714	-33.600487	-49.951385	]
vel_sp:[ -0.470636	-4.977800	0.120098	]

calculate_thrust_setpoint
vel_err:[ -0.305152	-0.853983	-0.010846	]
thrust_sp:[ 0.005466	-0.300046	-0.593902	]
R_z :[  -0.0099 0.44    0.9 ]
F :[    0.0055  -0.3    -0.59   ]
thrust_body_z : 0.665356
_att_sp:[0.008803,-0.467798,-1.572101]
_att_sp.thrust: 0.665356

control_attitude
_rates_sp: [ -0.009661	-0.078900	0.009454	]
att_control: [ 0.002824	-0.063826	0.000463	]

```

matlab simulink result:
```
control_auto
pos_sp:[ 0.0667	-33.60	-49.95	]
vel_sp:[ -0.4706	-4.978	0.1201	]

calculate_thrust_setpoint
vel_err:[ -0.3052	-0.854	-0.01084	]
thrust_sp:[ 0.005467	-0.3	-0.5939	]
R_z :[  -0.0099 0.4391    0.8984 ]
F :[    0.005467  -0.3    -0.5939   ]
thrust_body_z : 0.6654
_att_sp:[0.008804,-0.4678,-1.572]
_att_sp.thrust: 0.6654

control_attitude
_rates_sp: [ -0.009661	-0.0789	0.009452	]
att_control: [ 0.0028274	-0.06383	0.0004624	]
```
