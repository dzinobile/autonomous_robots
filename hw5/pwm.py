from matplotlib import pyplot as plt
import numpy as np

duty_cycle_1 = 30
duty_cycle_2 = 70
cycle_duration = 0.1
cycles = 10
period = 0.001

t = np.arange(0, cycles*cycle_duration, period)

pwm1 = (t%cycle_duration<cycle_duration*duty_cycle_1/100)*3
pwm2 = (t%cycle_duration<cycle_duration*duty_cycle_2/100)*3
z = np.zeros(len(t))
fig,ax = plt.subplots(2)
ax[0].plot(t, pwm1)
ax[0].plot(t,z)
ax[0].set_title('Forward 30%')
ax[0].legend(['A', 'B'])
ax[0].set_ylabel('Voltage [V]')
ax[1].plot(t,z)
ax[1].plot(t, pwm2)
ax[1].set_title('Backward 70%')
ax[1].legend(['A', 'B'])
ax[1].set_ylabel('Voltage [V]')
plt.xlabel('Time [sec]')
plt.subplots_adjust(hspace=0.5)
plt.show()
