std_msgs/Header header

#Watchdog signal
uint8 LOW = 0
uint8 HIGHT = 1
uint8 watchdog_signal

#System/Node State definition
uint8 INITIALIZATION = 10
uint8 READY = 20
uint8 DOWNGRADED_READY = 25
uint8 ENABLE = 30
uint8 DOWNGRADED_ENABLE = 35
uint8 ACTIVE = 40
uint8 DOWNGRADED_ACTIVE = 45
uint8 ERROR = 60

#BSD System State
uint8 bsd_state

#ICA System State
uint8 ica_state

#Trigger state
uint8 OFF = 10
uint8 ON = 20
uint8 trigger_state

#BSD System State to HMI
uint8 OTHER_BSD_HMI = 0
uint8 DOWNGRADE_BSD_HMI = 1
uint8 ERROR_BSD_HMI = 2
uint8 bsd_state_hmi

#ICA System State to HMI
uint8 OTHER_ICA_HMI = 0
uint8 DOWNGRADE_ICA_HMI = 1
uint8 ERROR_ICA_HMI = 2
uint8 ica_state_hmi

#Detailed system state to HMI
#uint8 INITIALIZATION = 10
#uint8 READY = 20
#uint8 DOWNGRADED_READY = 25
#uint8 ENABLE = 30
#uint8 DOWNGRADED_ENABLE = 35
#uint8 ACTIVE = 40
#uint8 DOWNGRADED_ACTIVE = 45
#uint8 ERROR = 60
uint8 cpu_state
uint8 gpu_state
uint8 memory_state
uint8 disk_state
uint8[] watchdog_state_list
uint8[] node_state_list
uint8 sw_bsd_state
uint8 sw_ica_state

#uint8 OFF = 10
#uint8 ON = 20
uint8 trigger_state_hmi

