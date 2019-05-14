# Msg for '/payload/status' topic for ATM by payload
# Event-based msg
# Detailed explanations pls refer to 'https://github.com/RMFHOPE/delivery_compartment' readme 

# Unique Payload ID
string payload_id

# Unique Task ID
string task_id

# Payload Status, -1: error, 0: idle, 1: executing task, 2: Done
#                  3: ready to close, 4: ready to open
int8 status


int8 PLD_ERROR=-1
int8 PLD_IDLE=0
int8 PLD_EXEC_TASK=1
int8 PLD_TASK_DONE=2
int8 PLD_DONE_OPEN=3
int8 PLD_DONE_CLOSE=4