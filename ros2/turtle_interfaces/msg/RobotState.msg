uint8 MODE_MANUAL=0
uint8 MODE_AUTO=1

std_msgs/Header header
uint8 mode
bool running
bool estop
bool stm32_connected
uint16 mcu_error_flags
string message
