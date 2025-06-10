# -*- coding: utf-8 -*-
"""
example on how to plot decoded sensor data from crazyflie
@author: jsschell
"""
# TODO: Refactor this code to improve readability and maintainability and using easyplots
import cfusdlog
import matplotlib.pyplot as plt
import re
import argparse
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("filename")
args = parser.parse_args()

logData = cfusdlog.decode(args.filename)

#only focus on regular logging
logData = logData['fixedFrequency']

plt.rcParams['figure.facecolor'] = 'w'
    
plotCols = 1
plotRows = 1

# let's see which keys exists in current data set
keys = ""
for k, v in logData.items():
    keys += k

# print(f"Available keys in log data: {keys}")

# get plot config from user
plotGyro = 0
if re.search('gyro', keys):
    inStr = input("plot gyro data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotGyro = 1
        plotRows += 1

plotAccel = 0
if re.search('acc', keys):
    inStr = input("plot accel data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotAccel = 1
        plotRows += 1

plotBaro = 0
if re.search('baro', keys):
    inStr = input("plot barometer data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotBaro = 1
        plotRows += 1

plotCtrl = 0
if re.search('ctrltarget', keys):
    inStr = input("plot control data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotCtrl = 1
        plotRows += 1

plotStab = 0
if re.search('stabilizer', keys):
    inStr = input("plot stabilizer data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotStab = 1
        plotRows += 1

plotCtrlNN = 0
if re.search('ctrlNN', keys):
    inStr = input("plot control NN data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotCtrlNN = 1
        plotRows += 1

plotPID = 0
if re.search('powerDistLog', keys):
    inStr = input("plot control PID data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotPID = 1
        plotRows += 1
    
# current plot for simple subplot usage
plotCurrent = 0

# new figure
# plt.figure(0)

if plotGyro:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['timestamp'], logData['gyro.x'], '-', label='X')
    plt.plot(logData['timestamp'], logData['gyro.y'], '-', label='Y')
    plt.plot(logData['timestamp'], logData['gyro.z'], '-', label='Z')
    plt.xlabel('timestamp [ms]')
    plt.ylabel('Gyroscope [°/s]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)
 
if plotAccel:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['timestamp'], logData['acc.x'], '-', label='X')
    plt.plot(logData['timestamp'], logData['acc.y'], '-', label='Y')
    plt.plot(logData['timestamp'], logData['acc.z'], '-', label='Z')
    plt.xlabel('timestamp [ms]')
    plt.ylabel('Accelerometer [g]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)
 
if plotBaro:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['timestamp'], logData['baro.pressure'], '-')
    plt.xlabel('timestamp [ms]')
    plt.ylabel('Pressure [hPa]')
    
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['timestamp'], logData['baro.temp'], '-')
    plt.xlabel('timestamp [ms]')
    plt.ylabel('Temperature [degC]')

if plotCtrl:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['timestamp'], logData['ctrltarget.roll'], '-', label='roll')
    plt.plot(logData['timestamp'], logData['ctrltarget.pitch'], '-', label='pitch')
    plt.plot(logData['timestamp'], logData['ctrltarget.yaw'], '-', label='yaw')
    plt.xlabel('timestamp [ms]')
    plt.ylabel('Control')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

if plotStab:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['timestamp'], logData['stabilizer.roll'], '-', label='roll')
    plt.plot(logData['timestamp'], logData['stabilizer.pitch'], '-', label='pitch')
    plt.plot(logData['timestamp'], logData['stabilizer.yaw'], '-', label='yaw')
    plt.plot(logData['timestamp'], logData['stabilizer.thrust'], '-', label='thrust')
    plt.xlabel('timestamp [ms]')
    plt.ylabel('Stabilizer')
    plt.legend(loc=9, ncol=4, borderaxespad=0.)

if plotCtrlNN:
    # plotCurrent += 1
    _, axis = plt.subplots(3, 2, sharex=True)
    axis[0][0].plot(logData['timestamp'], logData['ctrlNN.ob_x'], '-', label='position x')
    axis[0][0].plot(logData['timestamp'], logData['ctrlNN.ob_y'], '-', label='position y')
    axis[0][0].plot(logData['timestamp'], logData['ctrlNN.ob_z'], '-', label='position z')
    axis[0][0].grid(visible=True, which='both', axis='both', color='gray', alpha=0.35, linestyle='--')
    axis[0][0].set_xlabel('timestamp [ms]')
    axis[0][0].set_ylabel('Position')
    axis[0][0].legend(loc=9, ncol=3, borderaxespad=0.)

    axis[0][1].plot(logData['timestamp'], np.rad2deg(logData['ctrlNN.ob_roll']), '-', label='roll')
    axis[0][1].plot(logData['timestamp'], np.rad2deg(logData['ctrlNN.ob_pitch']), '-', label='pitch')
    axis[0][1].plot(logData['timestamp'], np.rad2deg(logData['ctrlNN.ob_yaw']), '-', label='yaw')
    axis[0][1].grid(visible=True, which='both', axis='both', color='gray', alpha=0.35, linestyle='--')
    axis[0][1].set_xlabel('timestamp [ms]')
    axis[0][1].set_ylabel('orientation')
    axis[0][1].legend(loc=9, ncol=3, borderaxespad=0.)

    axis[1][0].plot(logData['timestamp'], logData['ctrlNN.motor_pwm_0'], '-', label='motor PWM 0')
    axis[1][0].grid(visible=True, which='both', axis='both', color='gray', alpha=0.35, linestyle='--')
    axis[1][0].set_ylabel('Control PWM')
    axis[1][0].legend(loc=9, ncol=4, borderaxespad=0.)
    axis[1][1].plot(logData['timestamp'], logData['ctrlNN.motor_pwm_1'], '-', label='motor PWM 1')
    axis[1][1].grid(visible=True, which='both', axis='both', color='gray', alpha=0.35, linestyle='--')
    axis[1][1].set_ylabel('Control PWM')
    axis[1][1].legend(loc=9, ncol=4, borderaxespad=0.)
    axis[2][0].plot(logData['timestamp'], logData['ctrlNN.motor_pwm_2'], '-', label='motor PWM 2')
    axis[2][0].grid(visible=True, which='both', axis='both', color='gray', alpha=0.35, linestyle='--')
    axis[2][0].set_xlabel('timestamp [ms]')
    axis[2][0].set_ylabel('Control PWM')
    axis[2][0].legend(loc=9, ncol=4, borderaxespad=0.)
    axis[2][1].plot(logData['timestamp'], logData['ctrlNN.motor_pwm_3'], '-', label='motor PWM 3')
    axis[2][1].grid(visible=True, which='both', axis='both', color='gray', alpha=0.35, linestyle='--')
    axis[2][1].set_xlabel('timestamp [ms]')
    axis[2][1].set_ylabel('Control PWM')
    axis[2][1].legend(loc=9, ncol=4, borderaxespad=0.)

if plotPID:
    # plotCurrent += 1
    # plt.subplot(plotRows, plotCols, plotCurrent)
    _, axis = plt.subplots(4, 1, sharex=True)
    axis[0].plot(logData['timestamp'], logData['powerDistLog.motor_pwm_0'], '-', label='motor PWM 0')
    axis[0].grid(visible=True, which='both', axis='both', color='gray', alpha=0.35, linestyle='--')
    axis[0].set_ylabel('Control PWM')
    axis[0].legend(loc=9, ncol=4, borderaxespad=0.)
    axis[1].plot(logData['timestamp'], logData['powerDistLog.motor_pwm_1'], '-', label='motor PWM 1')
    axis[1].grid(visible=True, which='both', axis='both', color='gray', alpha=0.35, linestyle='--')
    axis[1].set_ylabel('Control PWM')
    axis[1].legend(loc=9, ncol=4, borderaxespad=0.)
    axis[2].plot(logData['timestamp'], logData['powerDistLog.motor_pwm_2'], '-', label='motor PWM 2')
    axis[2].grid(visible=True, which='both', axis='both', color='gray', alpha=0.35, linestyle='--')
    axis[2].set_ylabel('Control PWM')
    axis[2].legend(loc=9, ncol=4, borderaxespad=0.)
    axis[3].plot(logData['timestamp'], logData['powerDistLog.motor_pwm_3'], '-', label='motor PWM 3')
    axis[3].grid(visible=True, which='both', axis='both', color='gray', alpha=0.35, linestyle='--')
    axis[3].set_xlabel('timestamp [ms]')
    axis[3].set_ylabel('Control PWM')
    axis[3].legend(loc=9, ncol=4, borderaxespad=0.)

plt.show()