# Driver challenge is echoed; no cross-host monotonic clock comparison.
string session
uint64 challenge
string owner
string operation_id
uint64 sequence
uint8 STOP=0
uint8 POSITION=1
uint8 SWEEP=2
uint8 mode
uint16 servo_id
int32 target
int32 minimum
int32 maximum
uint16 speed
uint8 acceleration
uint16 lead
uint16 tolerance
# Generic protective load stop, zero disables it.
float64 load_stop_percent
