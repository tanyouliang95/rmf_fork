
# Msg for '/payload/command' topic for ATM to control the payload

# Unique Payload ID
string payload_id

# Unique Task ID
string task_id

# AGV state, 0: moving, 1: stopped at Nurse, 2: stopped at Patient, 3: FM Control
uint8 state

# Only for State 3: provide rfid of the patient to access payload
string patient_rfid

# To control Payload Door, 1: left open, 2: total close, 3: right open, 9: idle state
uint8 control_command
