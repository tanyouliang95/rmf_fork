# Event based message

string beverage_machine_id
int8 preparing               # error: -1, 0: nothing, 1: preparing
bool ready                  # True: drink is ready, False: not ready; `True` will last 5s
