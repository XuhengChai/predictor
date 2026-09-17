std_msgs/Header header

#Watchdog signal
uint8 LOW = 0
uint8 HIGHT = 1
uint8 watchdog_signal

#Node State
uint8 INITIALIZATION = 10
uint8 ENABLED = 30
uint8 DOWNGRADE = 50
uint8 ERROR = 60
uint8 node_state

#Warning state from Planning to BSD Statechart
uint8 OFF_BSD = 10
uint8 ON_BSD = 20
uint8 warning_state

#Active state from Control to ICA Statechart
uint8 OFF_ICA = 10
uint8 ON_ICA = 20
uint8 active_state