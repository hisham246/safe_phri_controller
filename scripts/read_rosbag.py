#!/usr/bin/env python

import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sea
import numpy as np

bag = bagreader('/home/robohub/demos/demo_2024-09-17-09-12-09.bag')

traj = bag.message_by_topic('/franka_state_controller/O_T_EE')

df_traj = pd.read_csv(traj)

df_traj = df_traj.drop(df_traj.columns[:5], axis=1)

df_traj.to_csv('/home/robohub/demos/demo_2024-09-17-09-12-09.csv', index=False)